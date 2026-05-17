""" This test will introduce the image capture and video streaming command in Pysimverse. 
The drone will take off, capture an image or stream a video, and then land. 
This test is designed to verify that the image capture and video streaming functionality is 
working correctly in the Pysimverse environment. """

import cv2
import cv2
from pysimverse import Drone
import time


# Connect to the drone and take off in Pysimverse UI
drone = Drone()
drone.connect()
time.sleep(1)
drone.streamon()
drone.take_off()

# Capture an image using the drone's camera
while True:
    frame, is_success = drone.get_frame()
    
    cv2.imshow("Drone Camera Stream", frame)
    cv2.waitKey(1)
    

drone.land()
time.sleep(3)