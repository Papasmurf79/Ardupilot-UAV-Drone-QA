from pysimverse import Drone
import time

drone = Drone()
drone.connect()
drone.take_off()

# Note: number parameters for movement commands are in centimeters (cm)

drone.set_speed(100)

# First Target Point (Upper Left Target in the Garage)
drone.move_forward(340)
time.sleep(1.5)
drone.move_left(225)
time.sleep(1.5)

# Second Target Point (Right Target Under the Table)
drone.move_backward(140)
time.sleep(1.5)
drone.move_right(400)
time.sleep(1.5)
drone.move_down(70)
time.sleep(1)
drone.move_right(75)
time.sleep(1)

# Land Drone and Complete Mission
drone.land()
time.sleep(3)