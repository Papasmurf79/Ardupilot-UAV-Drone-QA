"""Color detection on the Pysimverse drone's live video stream.

Combines color_detection.py and drone_video_stream.py:
the drone takes off and streams its camera feed, which is then
passed through cvzone's ColorFinder for real-time HSV color detection.
If you want to recalibrate the HSV values for a different color, set trackBar=True to find the right values, then
paste them into the hsvVals dictionary below and set trackBar=False.

Use the trackbar to tune HSV values for your target color, then set
trackBar=False and paste the final values into hsvVals below.

Press Q to land and quit.
"""

import time

import cv2
import cvzone
from cvzone.ColorModule import ColorFinder
from pysimverse import Drone

# Create an instance of the ColorFinder class with trackBar set to True.
# Use the trackbar to adjust HSV values for color detection in real-time.
# For debugging purposes. Once happy with the values, set trackBar=False
# and update the hsvVals dictionary directly.
myColorFinder = ColorFinder(trackBar=False)

# Custom HSV color values. Adjust these to detect your target color.
# Use the trackbar to find the right values, then paste them here.
# 'hmin', 'smin', 'vmin' are the minimum values for Hue, Saturation, and Value.
# 'hmax', 'smax', 'vmax' are the maximum values for Hue, Saturation, and Value.
hsvVals = {'hmin': 0, 'smin': 100, 'vmin': 0, 'hmax': 179, 'smax': 255, 'vmax': 255}

# Connect to the drone and start streaming in Pysimverse.
drone = Drone()
drone.connect()
time.sleep(1)
drone.streamon()
drone.take_off(takeoff_height=30) # Set Take off meters height to 30 cm for better visibility of the ground and more stable flight.

# Main loop to continuously get frames from the drone camera.
try:
    while True:
        # Read the current frame from the drone's video stream.
        frame, is_success = drone.get_frame()

        # Guard: skip this frame if the drone failed to return an image.
        # This prevents a crash in cvtColor when frame is None or empty.
        if not is_success or frame is None:
            print("Warning: failed to grab drone frame, retrying...")
            continue

        # Use the update method from the ColorFinder class to detect the color.
        # It returns the masked color image and a binary mask.
        imgColor, mask = myColorFinder.update(frame, hsvVals)

        # Stack the original frame, the masked color image, and the binary mask.
        imgStack = cvzone.stackImages([frame, imgColor, mask], 2, 0.5)

        # Show the stacked images.
        cv2.imshow("Drone Color Detection", imgStack)

        # Land and quit if the 'q' key is pressed.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cv2.destroyAllWindows()
    drone.land()
    time.sleep(3)
