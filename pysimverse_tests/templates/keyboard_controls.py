"""Keyboard RC control in Pysimverse using the `keyboard` library.

Connect, take off, then fly with RC commands (same path as `rc_control.py`).
Press Esc to land and exit.

Key map (hold keys to move; release to stop that axis):
  W / S     forward / backward
  A / D     left / right
  Up / Down arrows, or T / G, or R / F   up / down (several bindings so keys work if the sim eats some)
  Q / E     yaw left / yaw right

On Windows, global keyboard hooks may require running the terminal as Administrator
if keys are not detected.

The fifth argument to `send_rc_control` in pysimverse is camera angle, not roll;
this script keeps it at 0 unless you extend it.
"""

import sys
import time

import keyboard
from pysimverse import Drone


def _any_pressed(*names: str) -> bool:
    return any(keyboard.is_pressed(n) for n in names)


def rc_from_keys(drone: Drone) -> tuple[int, int, int, float]:
    """Build (left_right, forward_backward, up_down, yaw) from current key states."""
    speed = int(drone.speed)
    # take_off() uses a separate climb rate; many builds ignore small vertical RC
    # while horizontal still responds at default speed (see pysimverse Drone defaults).
    vertical_speed = max(speed, 25)
    # Match rotate(): small normalized yaw commands in the simulator
    yaw_mag = drone.rotation_speed / 70.0

    lr = int(keyboard.is_pressed("d")) - int(keyboard.is_pressed("a"))
    fb = int(keyboard.is_pressed("w")) - int(keyboard.is_pressed("s"))
    # R/F are often bound in the sim (reset, interact); arrows and T/G are safer fallbacks.
    ud = int(_any_pressed("up", "t", "r")) - int(_any_pressed("down", "g", "f"))
    yv = int(keyboard.is_pressed("e")) - int(keyboard.is_pressed("q"))

    return (
        lr * speed,
        fb * speed,
        ud * vertical_speed,
        yv * yaw_mag,
    )


def main() -> None:
    print(__doc__)

    drone = Drone()
    drone.connect()
    time.sleep(0.5)
    drone.take_off()

    period_s = 1.0 / 30.0  # ~30 Hz RC updates
    try:
        while True:
            if keyboard.is_pressed("esc"):
                break

            left_right, forward_backward, up_down, yaw = rc_from_keys(drone)
            drone.send_rc_control(left_right, forward_backward, up_down, yaw)

            time.sleep(period_s)
    except KeyboardInterrupt:
        pass
    finally:
        drone.send_rc_control(0, 0, 0, 0)
        drone.land()
        time.sleep(1)


if __name__ == "__main__":
    main()
    sys.exit(0)
