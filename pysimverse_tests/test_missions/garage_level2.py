from pysimverse import Drone
import time

drone = Drone()
drone.connect()
drone.take_off()

# Note: number parameters for movement commands are in centimeters (cm)

drone.set_speed(80)

# First Target Point (Left Target in the Garage)
drone.move_forward(75)
time.sleep(1.5)
drone.move_left(215)
time.sleep(1.5)

# Second Target Point (Middle Target in the Garage)
drone.move_forward(120)
time.sleep(1.5)
drone.move_right(220)
time.sleep(1.5)

# Third Target Point (Right Target in the Garage)
drone.move_forward(110)
time.sleep(1.5)
drone.move_right(235)
time.sleep(1)

# Land Drone and Complete Mission
drone.land()
time.sleep(3)