"""Level 2: jump detection drives drone UP in Pysimverse body follower mini game.

Uses body_follow_mediapipe.py on a mirrored webcam feed.  When the player
jumps, the drone ascends; when they land, the drone returns to its original
altitude via a downward RC pulse.

Usage:
    python body_follower_lvls.py

Controls:
    Jump  -> drone flies up, then comes back down
    Q     -> land and quit

Architecture mirrors hand_gesture_levels.py:
    Drone() -> connect() -> take_off() -> RC loop -> land()

Detection Methods (composite vote, all four active):
    1. HIP_RISE   - hips (lm 23/24) rise sharply in one frame
    2. ANKLE_LIFT - both ankles above calibrated floor baseline
    3. KNEE_BEND  - crouch-then-straighten launch pattern
    4. VELOCITY   - upward velocity spike of hip midpoint

Docs:
    https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker
"""

from __future__ import annotations

import sys
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path
from typing import Optional

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from pysimverse import Drone

# ---------------------------------------------------------------------------
# Path setup — mirrors the pattern in hand_gesture_levels.py
# ---------------------------------------------------------------------------
_PYSIMVERSE_TESTS = Path(__file__).resolve().parent.parent
if str(_PYSIMVERSE_TESTS) not in sys.path:
    sys.path.insert(0, str(_PYSIMVERSE_TESTS))

from templates.body_follow_mediapipe import (
    _MODEL_PATH,
    draw_landmarks_on_image,
    mirror_camera_frame,
)

# ---------------------------------------------------------------------------
# Landmark indices (BlazePose 33-point model)
# ---------------------------------------------------------------------------
LM_LEFT_HIP    = 23
LM_RIGHT_HIP   = 24
LM_LEFT_KNEE   = 25
LM_RIGHT_KNEE  = 26
LM_LEFT_ANKLE  = 27
LM_RIGHT_ANKLE = 28

# ---------------------------------------------------------------------------
# Jump detection tuning
# ---------------------------------------------------------------------------
HIP_RISE_THRESHOLD:      float = 0.04    # normalised-Y units per frame pair
ANKLE_LIFT_THRESHOLD:    float = 0.03    # above calibrated floor
KNEE_BEND_RATIO:         float = 0.12    # knee below hip ratio for "bent"
JUMP_VELOCITY_THRESHOLD: float = -0.35   # normalised-Y / second (negative = up)
COMPOSITE_THRESHOLD:     float = 0.55    # weighted vote to confirm jump
JUMP_COOLDOWN_S:         float = 0.8     # minimum gap between jump events
CALIBRATION_FRAMES:      int   = 30      # frames to measure floor baseline
VELOCITY_WINDOW:         int   = 5       # rolling window for velocity estimate

VOTE_WEIGHTS: dict[str, float] = {
    "hip_rise":   0.35,
    "ankle_lift": 0.30,
    "knee_bend":  0.15,
    "velocity":   0.20,
}

# ---------------------------------------------------------------------------
# Drone ascent / descent tuning
# ---------------------------------------------------------------------------
RC_HZ:              float = 30.0   # RC send rate (Hz)
ASCENT_SPEED:       int   = 50     # up_down value while rising  (0–100)
DESCENT_SPEED:      int   = -40    # up_down value while returning to floor
ASCENT_DURATION_S:  float = 0.6    # how long to push upward after jump
DESCENT_DURATION_S: float = 0.7    # how long to push downward to return

WINDOW_NAME  = "Body Follower — Jump Detection (pysimverse)"
FLASH_FRAMES = 20                   # frames to show the jump flash overlay

# ---------------------------------------------------------------------------
# Internal data classes
# ---------------------------------------------------------------------------

@dataclass
class JumpEvent:
    timestamp: float
    strength:  float   # 0–1 proxy for jump intensity
    velocity:  float
    hip_y:     float


@dataclass
class _LandmarkSnapshot:
    hip_mid_y:     float
    left_ankle_y:  float
    right_ankle_y: float
    left_knee_y:   float
    right_knee_y:  float
    left_hip_y:    float
    right_hip_y:   float
    timestamp:     float


class DroneState(Enum):
    HOVER      = auto()
    ASCENDING  = auto()
    DESCENDING = auto()


# ---------------------------------------------------------------------------
# Jump detector
# ---------------------------------------------------------------------------

class JumpDetector:
    def __init__(self) -> None:
        self._calibrated    = False
        self._calib_frames: list[float] = []
        self._floor_y       = 0.85
        self._history: deque[_LandmarkSnapshot] = deque(maxlen=VELOCITY_WINDOW + 2)
        self._last_jump     = 0.0
        self._knee_was_bent = False
        self.debug_votes: dict[str, float] = {}
        self.debug_composite: float = 0.0

    # ---- public ----

    def update(self, detection_result) -> Optional[JumpEvent]:
        snap = self._extract(detection_result)
        if snap is None:
            return None

        if not self._calibrated:
            self._calib_frames.append((snap.left_ankle_y + snap.right_ankle_y) / 2)
            if len(self._calib_frames) >= CALIBRATION_FRAMES:
                self._floor_y = float(np.percentile(self._calib_frames, 90))
                self._calibrated = True
            return None

        self._history.append(snap)

        if snap.timestamp - self._last_jump < JUMP_COOLDOWN_S:
            return None

        votes = {
            "hip_rise":   self._check_hip_rise(),
            "ankle_lift": self._check_ankle_lift(snap),
            "knee_bend":  self._check_knee_bend(snap),
            "velocity":   self._check_velocity(),
        }
        self.debug_votes     = votes
        composite            = sum(VOTE_WEIGHTS[k] * v for k, v in votes.items())
        self.debug_composite = composite

        if composite < COMPOSITE_THRESHOLD:
            return None

        self._last_jump = snap.timestamp
        vel      = self._hip_velocity()
        strength = min(1.0, abs(vel) / abs(JUMP_VELOCITY_THRESHOLD) * 0.5
                       + composite * 0.5)
        return JumpEvent(
            timestamp=snap.timestamp,
            strength=round(strength, 3),
            velocity=round(vel, 4),
            hip_y=snap.hip_mid_y,
        )

    @property
    def calibrated(self) -> bool:
        return self._calibrated

    @property
    def calib_progress(self) -> float:
        return len(self._calib_frames) / CALIBRATION_FRAMES

    @property
    def floor_y(self) -> float:
        return self._floor_y

    # ---- extraction ----

    def _extract(self, detection_result) -> Optional[_LandmarkSnapshot]:
        if not detection_result.pose_landmarks:
            return None
        lm = detection_result.pose_landmarks[0]
        try:
            return _LandmarkSnapshot(
                hip_mid_y      = (lm[LM_LEFT_HIP].y   + lm[LM_RIGHT_HIP].y)   / 2,
                left_ankle_y   = lm[LM_LEFT_ANKLE].y,
                right_ankle_y  = lm[LM_RIGHT_ANKLE].y,
                left_knee_y    = lm[LM_LEFT_KNEE].y,
                right_knee_y   = lm[LM_RIGHT_KNEE].y,
                left_hip_y     = lm[LM_LEFT_HIP].y,
                right_hip_y    = lm[LM_RIGHT_HIP].y,
                timestamp      = time.time(),
            )
        except IndexError:
            return None

    # ---- detection methods ----

    def _check_hip_rise(self) -> float:
        if len(self._history) < 2:
            return 0.0
        prev, curr = self._history[-2], self._history[-1]
        delta = prev.hip_mid_y - curr.hip_mid_y   # positive = rising
        return min(1.0, delta / (HIP_RISE_THRESHOLD * 2)) if delta >= HIP_RISE_THRESHOLD else 0.0

    def _check_ankle_lift(self, snap: _LandmarkSnapshot) -> float:
        ll = self._floor_y - snap.left_ankle_y
        rl = self._floor_y - snap.right_ankle_y
        if ll >= ANKLE_LIFT_THRESHOLD and rl >= ANKLE_LIFT_THRESHOLD:
            return min(1.0, ((ll + rl) / 2) / (ANKLE_LIFT_THRESHOLD * 3))
        return 0.0

    def _check_knee_bend(self, snap: _LandmarkSnapshot) -> float:
        hip_y  = (snap.left_hip_y  + snap.right_hip_y)  / 2
        knee_y = (snap.left_knee_y + snap.right_knee_y) / 2
        bent   = (knee_y - hip_y) >= KNEE_BEND_RATIO
        if self._knee_was_bent and not bent:
            self._knee_was_bent = False
            return 1.0
        if bent:
            self._knee_was_bent = True
        return 0.0

    def _check_velocity(self) -> float:
        vel = self._hip_velocity()
        return min(1.0, abs(vel) / abs(JUMP_VELOCITY_THRESHOLD)) if vel <= JUMP_VELOCITY_THRESHOLD else 0.0

    def _hip_velocity(self) -> float:
        if len(self._history) < 2:
            return 0.0
        prev, curr = self._history[-2], self._history[-1]
        dt = curr.timestamp - prev.timestamp
        return (curr.hip_mid_y - prev.hip_mid_y) / dt if dt > 0 else 0.0


# ---------------------------------------------------------------------------
# HUD drawing
# ---------------------------------------------------------------------------

def _draw_hud(frame: np.ndarray, detector: JumpDetector,
              drone_state: DroneState, jump_flash: int) -> np.ndarray:
    h, w    = frame.shape[:2]
    overlay = frame.copy()

    # Calibration bar
    if not detector.calibrated:
        bar_w = int(w * detector.calib_progress)
        cv2.rectangle(overlay, (0, h - 20), (bar_w, h), (0, 200, 255), -1)
        cv2.putText(overlay,
                    f"Calibrating... {int(detector.calib_progress * 100)}%",
                    (10, h - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)
        return overlay

    # Vote bars
    colours = {
        "hip_rise":   (0, 200, 100),
        "ankle_lift": (0, 140, 255),
        "knee_bend":  (200, 80, 200),
        "velocity":   (255, 180, 0),
    }
    y = 30
    for key, col in colours.items():
        val = detector.debug_votes.get(key, 0.0)
        cv2.rectangle(overlay, (10, y - 12), (10 + int(150 * val), y + 2), col, -1)
        cv2.putText(overlay, f"{key}: {val:.2f}",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
        y += 20

    comp = detector.debug_composite
    cv2.rectangle(overlay, (10, y - 12), (10 + int(150 * comp), y + 2), (255, 255, 80), -1)
    cv2.putText(overlay, f"composite: {comp:.2f}  (thr {COMPOSITE_THRESHOLD})",
                (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 80), 1)

    # Floor line
    fy = int(detector.floor_y * h)
    cv2.line(overlay, (0, fy), (w, fy), (0, 255, 200), 1)
    cv2.putText(overlay, "floor", (4, fy - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 200), 1)

    # Drone state label (top-right)
    state_colour = {
        DroneState.HOVER:      (200, 200, 200),
        DroneState.ASCENDING:  (0, 255, 80),
        DroneState.DESCENDING: (0, 160, 255),
    }
    cv2.putText(overlay, f"drone: {drone_state.name}",
                (w - 220, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                state_colour[drone_state], 2)

    # Jump flash
    if jump_flash > 0:
        cv2.rectangle(overlay, (0, 0), (w, h), (0, 255, 100), 6)
        cv2.putText(overlay, "JUMP!  -> DRONE UP",
                    (w // 2 - 150, h // 2),
                    cv2.FONT_HERSHEY_DUPLEX, 1.1, (0, 255, 80), 3)

    return overlay


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main(camera_index: int = 0) -> None:
    if not _MODEL_PATH.is_file():
        raise FileNotFoundError(
            f"Pose landmarker model not found: {_MODEL_PATH}\n"
            "Download pose_landmarker.task into the models/ directory."
        )

    # ---- MediaPipe ----
    base_options = python.BaseOptions(model_asset_path=str(_MODEL_PATH))
    options = vision.PoseLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.VIDEO,
        output_segmentation_masks=False,
    )

    # ---- Camera ----
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {camera_index}")

    # ---- Drone — same connect/takeoff pattern as hand_gesture_levels.py ----
    drone = Drone()
    drone.connect()
    time.sleep(0.5)
    drone.take_off()

    # ---- State ----
    detector    = JumpDetector()
    drone_state = DroneState.HOVER
    state_until = 0.0       # time.time() when the current timed state ends
    jump_flash  = 0
    period_s    = 1.0 / RC_HZ
    start_time  = time.time()

    print(__doc__)

    try:
        with vision.PoseLandmarker.create_from_options(options) as landmarker:
            while cap.isOpened():
                loop_start = time.monotonic()

                success, frame = cap.read()
                if not success:
                    break

                # ---- Pose detection ----
                frame    = mirror_camera_frame(frame)
                rgb      = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                ts_ms    = int((time.time() - start_time) * 1000)
                result   = landmarker.detect_for_video(mp_image, ts_ms)

                # ---- Jump detection ----
                jump_event = detector.update(result)
                if jump_event and drone_state is DroneState.HOVER:
                    drone_state = DroneState.ASCENDING
                    state_until = time.time() + ASCENT_DURATION_S
                    jump_flash  = FLASH_FRAMES
                    print(
                        f"[JUMP] strength={jump_event.strength:.2f}  "
                        f"velocity={jump_event.velocity:.3f}  "
                        f"hip_y={jump_event.hip_y:.3f}"
                    )

                # ---- Drone state machine ----
                now = time.time()

                if drone_state is DroneState.ASCENDING and now >= state_until:
                    # Finished rising — descend back to original altitude
                    drone_state = DroneState.DESCENDING
                    state_until = now + DESCENT_DURATION_S

                elif drone_state is DroneState.DESCENDING and now >= state_until:
                    # Back at original altitude — hover
                    drone_state = DroneState.HOVER

                # ---- RC command — mirrors drone.send_rc_control in hand_gesture_levels.py ----
                if drone_state is DroneState.ASCENDING:
                    up_down = ASCENT_SPEED
                elif drone_state is DroneState.DESCENDING:
                    up_down = DESCENT_SPEED
                else:
                    up_down = 0

                drone.send_rc_control(0, 0, up_down, 0)

                # ---- Render ----
                annotated = draw_landmarks_on_image(rgb, result)
                annotated = _draw_hud(annotated, detector, drone_state, jump_flash)
                cv2.imshow(WINDOW_NAME, cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR))

                if jump_flash > 0:
                    jump_flash -= 1

                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                    break

                # ---- Rate-limit to RC_HZ (same pattern as hand_gesture_levels.py) ----
                elapsed = time.monotonic() - loop_start
                if elapsed < period_s:
                    time.sleep(period_s - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        # Always stop movement and land cleanly — same finally block pattern
        cap.release()
        cv2.destroyAllWindows()
        drone.send_rc_control(0, 0, 0, 0)
        drone.land()
        time.sleep(1)


if __name__ == "__main__":
    main()
    sys.exit(0)
