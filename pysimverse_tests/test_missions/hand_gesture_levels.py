"""Level 1: hand gestures drive drone left/right in Pysimverse.

Uses templates/hand_gesture_mediapipe.py on a mirrored webcam feed. The preview is
split into left, deadzone, and right regions. Hold your hand in the left or right
zone to move the drone sideways (RC, same path as templates/rc_control.py). The
deadzone sends neutral RC and stops lateral movement. Press Q to land and quit.
"""

from __future__ import annotations

import sys
import time
from enum import Enum
from pathlib import Path

import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from pysimverse import Drone

# test_missions/ -> parent is pysimverse_tests/ (contains templates/ package)
_PYSIMVERSE_TESTS = Path(__file__).resolve().parent.parent
if str(_PYSIMVERSE_TESTS) not in sys.path:
    sys.path.insert(0, str(_PYSIMVERSE_TESTS))

from templates.hand_gesture_mediapipe import (
    _MODEL_PATH,
    draw_landmarks_on_image,
    mirror_camera_frame,
)

# Normalized x (0–1): wrist left of LEFT_BOUND -> left; right of RIGHT_BOUND -> right.
LEFT_BOUND = 0.35
RIGHT_BOUND = 0.65
WINDOW_NAME = "Hand Gesture Level 1"
RC_HZ = 30.0


class Zone(Enum):
    LEFT = "left"
    RIGHT = "right"
    DEAD = "dead"


def classify_wrist_x(x: float) -> Zone:
    if x < LEFT_BOUND:
        return Zone.LEFT
    if x > RIGHT_BOUND:
        return Zone.RIGHT
    return Zone.DEAD


def wrist_x_from_result(detection_result) -> float | None:
    if not detection_result.hand_landmarks:
        return None
    return detection_result.hand_landmarks[0][0].x


def rc_from_zone(zone: Zone | None, drone: Drone) -> tuple[int, int, int, float]:
    """Build (left_right, forward_backward, up_down, yaw) from hand zone."""
    speed = int(drone.speed)
    left_right = 0
    if zone is Zone.LEFT:
        left_right = -speed
    elif zone is Zone.RIGHT:
        left_right = speed
    return left_right, 0, 0, 0.0


def draw_zone_overlay(bgr_frame, active_zone: Zone | None = None) -> None:
    height, width = bgr_frame.shape[:2]
    left_x = int(LEFT_BOUND * width)
    right_x = int(RIGHT_BOUND * width)

    overlay = bgr_frame.copy()
    cv2.rectangle(overlay, (0, 0), (left_x, height), (80, 80, 200), -1)
    cv2.rectangle(overlay, (right_x, 0), (width, height), (80, 200, 80), -1)
    cv2.rectangle(overlay, (left_x, 0), (right_x, height), (60, 60, 60), -1)
    cv2.addWeighted(overlay, 0.25, bgr_frame, 0.75, 0, bgr_frame)

    cv2.line(bgr_frame, (left_x, 0), (left_x, height), (0, 255, 255), 2)
    cv2.line(bgr_frame, (right_x, 0), (right_x, height), (0, 255, 255), 2)

    cv2.putText(
        bgr_frame, "LEFT", (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2
    )
    cv2.putText(
        bgr_frame,
        "DEADZONE",
        (left_x + 8, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (200, 200, 200),
        2,
    )
    cv2.putText(
        bgr_frame,
        "RIGHT",
        (right_x + 12, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 255, 255),
        2,
    )

    if active_zone is not None and active_zone is not Zone.DEAD:
        cv2.putText(
            bgr_frame,
            f"RC: {active_zone.value}",
            (12, height - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )


def main(camera_index: int = 0) -> None:
    if not _MODEL_PATH.is_file():
        raise FileNotFoundError(f"Hand landmarker model not found: {_MODEL_PATH}")

    base_options = python.BaseOptions(model_asset_path=str(_MODEL_PATH))
    options = vision.HandLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.VIDEO,
        num_hands=1,
    )

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {camera_index}")

    drone = Drone()
    drone.connect()
    time.sleep(0.5)
    drone.take_off()

    start_time = time.time()
    last_zone: Zone | None = None
    period_s = 1.0 / RC_HZ

    print(__doc__)
    print(f"Zones: x < {LEFT_BOUND} = left, x > {RIGHT_BOUND} = right, else deadzone.")

    try:
        with vision.HandLandmarker.create_from_options(options) as detector:
            while cap.isOpened():
                loop_start = time.monotonic()

                success, frame = cap.read()
                if not success:
                    break

                frame = mirror_camera_frame(frame)
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
                frame_timestamp_ms = int((time.time() - start_time) * 1000)
                detection_result = detector.detect_for_video(mp_image, frame_timestamp_ms)

                annotated_rgb = draw_landmarks_on_image(rgb_frame, detection_result)
                display = cv2.cvtColor(annotated_rgb, cv2.COLOR_RGB2BGR)

                wrist_x = wrist_x_from_result(detection_result)
                rc_zone: Zone | None = None
                if wrist_x is not None:
                    rc_zone = classify_wrist_x(wrist_x)
                    cx = int(wrist_x * display.shape[1])
                    cy = int(detection_result.hand_landmarks[0][0].y * display.shape[0])
                    cv2.circle(display, (cx, cy), 8, (0, 0, 255), -1)

                    if rc_zone is not Zone.DEAD and rc_zone != last_zone:
                        print(rc_zone.value)
                    last_zone = rc_zone
                else:
                    last_zone = None
                    cv2.putText(
                        display,
                        "Show one hand",
                        (12, display.shape[0] - 48),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 255, 255),
                        2,
                    )

                draw_zone_overlay(display, rc_zone)

                left_right, forward_backward, up_down, yaw = rc_from_zone(rc_zone, drone)
                drone.send_rc_control(left_right, forward_backward, up_down, yaw)

                cv2.imshow(WINDOW_NAME, display)
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                    break

                elapsed = time.monotonic() - loop_start
                if elapsed < period_s:
                    time.sleep(period_s - elapsed)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        drone.send_rc_control(0, 0, 0, 0)
        drone.land()
        time.sleep(1)


if __name__ == "__main__":
    main()
    sys.exit(0)
