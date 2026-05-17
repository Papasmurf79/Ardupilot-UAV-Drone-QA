import cvzone
from cvzone.ColorModule import ColorFinder 
import cv2


# Create an instance of the ColorFinder class with trackBar set to True.
# Use trackbar to adjust the HSV values for color detection in real-time.
# and for degugging purposes. Otherwise set trackBar to False and adjust the hsvVals dictionary directly.
myColorFinder = ColorFinder(trackBar=True)

# Initialize the video capture using OpenCV.
# Index 0 = default/built-in camera. Change to 1, 2, etc. if using an external webcam.
# If you are unsure which index to use, run the following snippet once to find active cameras:
#   for i in range(5):
#       cap = cv2.VideoCapture(i)
#       if cap.read()[0]: print(f"Camera found at index {i}")
#       cap.release()
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    raise RuntimeError(
        "Could not open camera at index 0. "
        "Try changing the index above (1, 2, ...) to match your webcam."
    )

# Set the dimensions of the camera feed to 640x480.
cap.set(3, 640)
cap.set(4, 480)

# Custom color values for detecting orange.
# Use and adjust these HSV values to detect different colors. 
# You can use the trackbar to find the right values for your target color.
# 'hmin', 'smin', 'vmin' are the minimum values for Hue, Saturation, and Value.
# 'hmax', 'smax', 'vmax' are the maximum values for Hue, Saturation, and Value.
hsvVals = {'hmin': 60, 'smin': 88, 'vmin': 0, 'hmax': 139, 'smax': 255, 'vmax': 255}

# Main loop to continuously get frames from the camera.
while True:
    # Read the current frame from the camera.
    success, img = cap.read()

    # Guard: skip this frame if the camera failed to return an image.
    # This prevents a crash in cvtColor when img is None.
    if not success or img is None:
        print("Warning: failed to grab frame, retrying...")
        continue

    # Use the update method from the ColorFinder class to detect the color.
    # It returns the masked color image and a binary mask.
    imgOrange, mask = myColorFinder.update(img, hsvVals)

    # Stack the original image, the masked color image, and the binary mask.
    imgStack = cvzone.stackImages([img, imgOrange, mask], 3, 0.5)

    # Show the stacked images.
    cv2.imshow("Image Stack", imgStack)

    # Break the loop if the 'q' key is pressed.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
