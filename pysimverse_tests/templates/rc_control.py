from pysimverse import Drone
import time

drone = Drone()
drone.connect()
drone.take_off()

# Variable initialization for rc_control
left_right = 0
forward_backward = 0
up_down = 0
yaw = 0
roll = 0

# Will create the rc_control to move the drone in a square pattern
while True:
    drone.send_rc_control(left_right, forward_backward, 
                          up_down, yaw, roll)

drone.land()
time.sleep(3)

