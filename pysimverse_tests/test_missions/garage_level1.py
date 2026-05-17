from pysimverse import Drone
import time

drone = Drone()
drone.connect()
drone.take_off()

# Note: number parameters for movement commands are in centimeters (cm)

drone.set_speed(90)
drone.move_forward(275)
time.sleep(1)
drone.move_right(265)
time.sleep(1)

drone.land()
time.sleep(3)