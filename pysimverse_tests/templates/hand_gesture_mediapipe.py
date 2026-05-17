"""Hand landmark detection on a live OpenCV camera feed using MediaPipe.

Uses the Hand Landmarker task (not static images). Press Q to quit.

Model: templates/models/hand_landmarker.task
Docs: https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker/python
"""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

_SCRIPT_DIR = Path(__file__).resolve().parent
_MODEL_PATH = _SCRIPT_DIR / ".." / "models" / "hand_landmarker.task"

# CAMERA_INDEX = 0 saving this global variable for users to easily change the camera index if they have multiple cameras or want to use an external webcam.
MARGIN = 10
FONT_SIZE = 1
FONT_THICKNESS = 1
HANDEDNESS_TEXT_COLOR = (88, 205, 54)

mp_hands = mp.tasks.vision.HandLandmarksConnections
mp_drawing = mp.tasks.vision.drawing_utils
mp_drawing_styles = mp.tasks.vision.drawing_styles


def mirror_camera_frame(frame: np.ndarray) -> np.ndarray:
    """Flip horizontally so the preview matches a mirror (your left on the left)."""
    return cv2.flip(frame, 1)


def mirror_handedness_label(category_name: str) -> str:
    """Swap Left/Right labels so text matches the mirrored preview."""
    if category_name == "Left":
        return "Right"
    if category_name == "Right":
        return "Left"
    return category_name


def draw_landmarks_on_image(rgb_image: np.ndarray, detection_result) -> np.ndarray:
    hand_landmarks_list = detection_result.hand_landmarks
    handedness_list = detection_result.handedness
    annotated_image = np.copy(rgb_image)

    for idx in range(len(hand_landmarks_list)):
        hand_landmarks = hand_landmarks_list[idx]
        handedness = handedness_list[idx]

        mp_drawing.draw_landmarks(
            annotated_image,
            hand_landmarks,
            mp_hands.HAND_CONNECTIONS,
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style(),
        )

        height, width, _ = annotated_image.shape
        x_coordinates = [landmark.x for landmark in hand_landmarks]
        y_coordinates = [landmark.y for landmark in hand_landmarks]
        text_x = int(min(x_coordinates) * width)
        text_y = int(min(y_coordinates) * height) - MARGIN

        label = mirror_handedness_label(handedness[0].category_name)
        cv2.putText(
            annotated_image,
            label,
            (text_x, text_y),
            cv2.FONT_HERSHEY_DUPLEX,
            FONT_SIZE,
            HANDEDNESS_TEXT_COLOR,
            FONT_THICKNESS,
            cv2.LINE_AA,
        )

    return annotated_image


def main(camera_index: int = 0) -> None:
    if not _MODEL_PATH.is_file():
        raise FileNotFoundError(f"Hand landmarker model not found: {_MODEL_PATH}")

    base_options = python.BaseOptions(model_asset_path=str(_MODEL_PATH))
    options = vision.HandLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.VIDEO,
        num_hands=2,
    )

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {camera_index}")

    start_time = time.time()

    with vision.HandLandmarker.create_from_options(options) as detector:
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                break

            frame = mirror_camera_frame(frame)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            frame_timestamp_ms = int((time.time() - start_time) * 1000)

            detection_result = detector.detect_for_video(mp_image, frame_timestamp_ms)
            annotated_image = draw_landmarks_on_image(rgb_frame, detection_result)

            cv2.imshow(
                "Hand Gesture (MediaPipe)",
                cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR),
            )

            if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
