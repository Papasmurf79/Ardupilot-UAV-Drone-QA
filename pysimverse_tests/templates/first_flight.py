from pysimverse import Drone
import time

""" # This template demonstrates basic movement commands to 
control the drone in a square pattern. """

# Connect to the drone and take off in Pysimverse UI
drone = Drone()
drone.connect()
drone.take_off()

# Basic movement commands to move the drone in a square pattern
drone.move_down(20)
time.sleep(2)
drone.move_left(20)
time.sleep(3)
drone.move_up(30)
time.sleep(2)
drone.move_right(20)
time.sleep(3)

# Set speed and move forward and backward
# Note: number parameters for movement commands are in centimeters (cm)
# Note 2: time duration is set to miliseconds (ms) and is used to control how long the drone will perform the movement command

drone.set_speed(50)
drone.move_forward(30)
time.sleep(3)
drone.move_backward(30)
time.sleep(3)

# Land the drone after the movements and complete the flight
drone.land()
time.sleep(3)