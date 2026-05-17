"""Line Follower Level — Pysimverse drone programming trainer.

The drone takes off, streams its downward-facing camera, detects a colored
line on the ground via HSV masking, finds the line's centroid in the frame,
and steers toward it using RC control.

Steering logic (all values are small to stay smooth):
  - The frame is divided into LEFT / CENTER / RIGHT zones by the centroid X.
  - If the centroid is left of centre  -> small negative yaw (rotate left)
  - If the centroid is right of centre -> small positive yaw (rotate right)
  - The drone always creeps forward at a low constant speed.
  - If no line is detected the drone hovers in place and waits.

RC axes used (from rc_control.py reference):
    send_rc_control(left_right, forward_backward, up_down, yaw, roll)
    left_right        : lateral strafe  (not used here, kept 0)
    forward_backward  : constant creep forward
    up_down           : 0 (altitude held by pysimverse)
    yaw               : small ± rotation to steer onto line
    roll              : 0

Press Q to land and quit.

Reference files:
    color_detection.py        — HSV color detection with cvzone
    videoCam_color_detection.py — drone stream + color detection pattern
    rc_control.py             — send_rc_control usage
"""

from __future__ import annotations

import time

import cv2
import cvzone
import numpy as np
from cvzone.ColorModule import ColorFinder
from pysimverse import Drone

# ---------------------------------------------------------------------------
# HSV color values for the line.
# Default targets the RED line shown in the pysimverse Line Follower arena.
# Set trackBar=True to tune values live, then paste them here and set False.
# ---------------------------------------------------------------------------
myColorFinder = ColorFinder(trackBar=False)

# Red line HSV range.  Red wraps around 0/179 in HSV so we use a broad range
# that captures the bright red used in the pysimverse arena.
hsvVals = {'hmin': 0, 'smin': 120, 'vmin': 80, 'hmax': 10, 'smax': 255, 'vmax': 255}

# ---------------------------------------------------------------------------
# RC control tuning  — keep all values small for smooth, safe flight
# ---------------------------------------------------------------------------
FORWARD_SPEED:    int   = 8    # constant creep forward (forward_backward axis)
YAW_MAX:          int   = 12   # maximum yaw correction applied each frame
YAW_SENSITIVITY:  float = 0.08 # scales centroid error (0–1) → yaw command
TAKEOFF_HEIGHT:   int   = 30   # cm — low hover so downward cam sees the line
RC_HZ:            float = 30.0 # RC send rate

# ---------------------------------------------------------------------------
# Frame geometry
# ---------------------------------------------------------------------------
FRAME_W = 640
FRAME_H = 480

# Centroid X dead-zone: fraction of half-width treated as "on-centre"
# e.g. 0.10 means ±10 % of half-width is ignored (no yaw correction)
CENTRE_DEAD_ZONE: float = 0.10

# Minimum contour area to treat as a valid line detection (filters noise)
MIN_CONTOUR_AREA: int = 500

# How many consecutive frames with no detection before stopping forward motion
NO_DETECT_STOP_FRAMES: int = 10

WINDOW_NAME = "Line Follower — Drone View"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def find_line_centroid(mask: np.ndarray) -> tuple[int, int] | None:
    """Return (cx, cy) of the largest contour in the binary mask, or None."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_CONTOUR_AREA:
        return None
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return cx, cy


def centroid_to_yaw(cx: int, frame_w: int) -> int:
    """Convert centroid X position to a yaw RC value.

    Returns a small integer in [-YAW_MAX, +YAW_MAX].
    Positive yaw = rotate right (clockwise from above).
    """
    centre = frame_w / 2
    error  = cx - centre                        # pixels: + = centroid right of centre
    normalised = error / (frame_w / 2)          # range roughly -1 to +1
    dead_half  = CENTRE_DEAD_ZONE
    if abs(normalised) < dead_half:
        return 0
    # Scale outside dead-zone
    scaled = (normalised - dead_half * np.sign(normalised)) / (1.0 - dead_half)
    yaw = int(np.clip(scaled * YAW_MAX / YAW_SENSITIVITY, -YAW_MAX, YAW_MAX))
    return yaw


def draw_hud(frame: np.ndarray, cx: int | None, cy: int | None,
             yaw: int, forward: int, no_detect_count: int) -> np.ndarray:
    """Draw centroid cross-hair and RC readout onto the frame."""
    h, w = frame.shape[:2]
    out  = frame.copy()

    # Centre guide line
    cv2.line(out, (w // 2, 0), (w // 2, h), (0, 255, 255), 1)

    # Dead-zone band
    dz = int(w / 2 * CENTRE_DEAD_ZONE)
    cv2.rectangle(out, (w // 2 - dz, 0), (w // 2 + dz, h), (60, 60, 60), 1)

    if cx is not None and cy is not None:
        # Centroid dot
        cv2.circle(out, (cx, cy), 8, (0, 0, 255), -1)
        cv2.line(out, (cx, cy), (w // 2, cy), (0, 200, 255), 2)
        cv2.putText(out, f"cx={cx}", (cx + 10, cy - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

    # RC readout
    status = "NO LINE" if no_detect_count >= NO_DETECT_STOP_FRAMES else "FOLLOWING"
    colour = (0, 80, 255) if status == "NO LINE" else (0, 255, 80)
    cv2.putText(out, status,            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
    cv2.putText(out, f"yaw:     {yaw:+4d}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    cv2.putText(out, f"forward: {forward:+4d}", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

    return out


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    # ---- Drone setup (mirrors videoCam_color_detection.py pattern) ----
    drone = Drone()
    drone.connect()
    time.sleep(1)
    drone.streamon()
    drone.take_off(takeoff_height=TAKEOFF_HEIGHT)

    period_s        = 1.0 / RC_HZ
    no_detect_count = 0

    print(__doc__)

    try:
        while True:
            loop_start = time.monotonic()

            # ---- Grab frame ----
            frame, is_success = drone.get_frame()
            if not is_success or frame is None:
                print("Warning: failed to grab drone frame, retrying...")
                drone.send_rc_control(0, 0, 0, 0, 0)
                continue

            # ---- Color / line detection (same pattern as color_detection.py) ----
            imgColor, mask = myColorFinder.update(frame, hsvVals)

            # Convert mask to single-channel uint8 for findContours
            if len(mask.shape) == 3:
                mask_gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
            else:
                mask_gray = mask
            _, mask_bin = cv2.threshold(mask_gray, 127, 255, cv2.THRESH_BINARY)

            # ---- Find line centroid ----
            centroid = find_line_centroid(mask_bin)

            # ---- Compute RC values ----
            if centroid is not None:
                cx, cy         = centroid
                no_detect_count = 0
                yaw     = centroid_to_yaw(cx, frame.shape[1])
                forward = FORWARD_SPEED
            else:
                cx = cy = None
                no_detect_count += 1
                yaw     = 0
                # Stop moving forward if line has been lost for several frames
                forward = 0 if no_detect_count >= NO_DETECT_STOP_FRAMES else FORWARD_SPEED

            # ---- Send RC (matches rc_control.py signature) ----
            # send_rc_control(left_right, forward_backward, up_down, yaw, roll)
            drone.send_rc_control(0, forward, 0, yaw, 0)

            # ---- Display ----
            hud_frame = draw_hud(frame, cx, cy, yaw, forward, no_detect_count)

            # Stack: live feed with HUD | colour mask | binary mask
            mask_bgr  = cv2.cvtColor(mask_bin, cv2.COLOR_GRAY2BGR)
            imgStack  = cvzone.stackImages([hud_frame, imgColor, mask_bgr], 3, 0.5)
            cv2.imshow(WINDOW_NAME, imgStack)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # ---- Rate limit ----
            elapsed = time.monotonic() - loop_start
            if elapsed < period_s:
                time.sleep(period_s - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        drone.send_rc_control(0, 0, 0, 0, 0)   # stop all movement
        drone.land()
        time.sleep(3)


if __name__ == "__main__":
    main()
