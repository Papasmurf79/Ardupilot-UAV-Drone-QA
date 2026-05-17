"""Level 1: keyboard flight with live camera and Z to save screenshots.

Combines keyboard RC control (see templates/keyboard_controls.py) with streaming
(templates/drone_video_stream.py). Hold flight keys as in the keyboard template;
press Z once to save the last good camera frame; Esc lands and exits.

After a successful save, the script notifies Pysimverse over the same ZMQ command
channel (extra JSON field `takepicture`) so the sim can show the mission-complete
flow, then holds neutral RC for a few seconds so you can read the UI before
landing. If your build uses a different field name, check CVZone docs and adjust
`_notify_sim_capture_objective`.

Z is registered as a global hotkey (so it works while the sim or any window is
focused, not only the OpenCV preview). It uses suppress=True so the key is not
passed through to the focused app when captured.

Screenshots are written under pysimverse_tests/mission_captures/.
"""

import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import keyboard
from pysimverse import Drone

_SCREENSHOT_DIR = Path(__file__).resolve().parent.parent / "mission_captures"


def _any_pressed(*names: str) -> bool:
    return any(keyboard.is_pressed(n) for n in names)


def rc_from_keys(drone: Drone) -> tuple[int, int, int, float]:
    speed = int(drone.speed)
    vertical_speed = max(speed, 25)
    yaw_mag = drone.rotation_speed / 70.0

    lr = int(keyboard.is_pressed("d")) - int(keyboard.is_pressed("a"))
    fb = int(keyboard.is_pressed("w")) - int(keyboard.is_pressed("s"))
    ud = int(_any_pressed("up", "t", "r")) - int(_any_pressed("down", "g", "f"))
    yv = int(keyboard.is_pressed("e")) - int(keyboard.is_pressed("q"))

    return (
        lr * speed,
        fb * speed,
        ud * vertical_speed,
        yv * yaw_mag,
    )


def _notify_sim_capture_objective(drone: Drone) -> None:
    """Signal the sim that the photo objective is done (ZMQ JSON extension)."""
    streaming = bool(getattr(drone, "stream_on", False))
    data = {
        "left_right": 0,
        "forward_backward": 0,
        "up_down": 0,
        "yaw": 0,
        "cameraangle": 0,
        "streamon": True if streaming else None,
        "color": {"red": None, "green": None, "blue": None},
        "takepicture": True,
    }
    try:
        drone.command_socket.send_json(data)
        print(f"Simulator mission signal: {data}")
    except Exception as exc:
        print(f"Simulator mission signal failed: {exc}")


def main() -> None:
    print(__doc__)
    _SCREENSHOT_DIR.mkdir(parents=True, exist_ok=True)

    drone = Drone()
    drone.connect()
    time.sleep(1)
    drone.streamon()
    drone.take_off()

    window = "Drone Camera Stream"
    last_good_frame = None
    capture_requested = threading.Event()

    def _request_capture() -> None:
        capture_requested.set()

    remove_z_hotkey = None
    try:
        remove_z_hotkey = keyboard.add_hotkey("z", _request_capture, suppress=True)
        while True:
            if keyboard.is_pressed("esc"):
                break

            frame, is_success = drone.get_frame()
            if is_success and frame is not None:
                last_good_frame = frame.copy()
                cv2.imshow(window, frame)

            if capture_requested.is_set():
                capture_requested.clear()
                if last_good_frame is not None:
                    name = datetime.now().strftime("capture_%Y%m%d_%H%M%S_%f")[:-3] + ".png"
                    out_path = _SCREENSHOT_DIR / name
                    cv2.imwrite(str(out_path), last_good_frame)
                    print(f"Saved screenshot: {out_path}")
                    _notify_sim_capture_objective(drone)
                    deadline = time.monotonic() + 4.0
                    while time.monotonic() < deadline:
                        drone.send_rc_control(0, 0, 0, 0)
                        cv2.waitKey(1)
                        time.sleep(1.0 / 30.0)
                    break
                else:
                    print("Capture skipped: no frame received yet.")

            left_right, forward_backward, up_down, yaw = rc_from_keys(drone)
            drone.send_rc_control(left_right, forward_backward, up_down, yaw)

            cv2.waitKey(1)
            time.sleep(1.0 / 30.0)
    except KeyboardInterrupt:
        pass
    finally:
        if remove_z_hotkey is not None:
            remove_z_hotkey()
        drone.send_rc_control(0, 0, 0, 0)
        try:
            drone.streamoff()
        except Exception:
            pass
        drone.land()
        time.sleep(1)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
    sys.exit(0)
