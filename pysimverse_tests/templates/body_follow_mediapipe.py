"""Pose landmark detection on a live OpenCV camera feed using MediaPipe.

Uses the Pose Landmarker task (not static images). Draws skeleton landmarks and
optionally overlays a segmentation mask. Press Q to quit.

Model: pysimverse_tests/models/pose_landmarker.task
Docs: https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker/python
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
_MODEL_PATH = _SCRIPT_DIR / ".." / "models" / "pose_landmarker.task"

MASK_ALPHA = 0.4
MASK_WINDOW_NAME = "Pose Segmentation Mask"
POSE_WINDOW_NAME = "Body Follow (MediaPipe)"

mp_drawing = mp.tasks.vision.drawing_utils
mp_drawing_styles = mp.tasks.vision.drawing_styles


def mirror_camera_frame(frame: np.ndarray) -> np.ndarray:
    """Flip horizontally so the preview matches a mirror (your left on the left)."""
    return cv2.flip(frame, 1)


def draw_landmarks_on_image(rgb_image: np.ndarray, detection_result) -> np.ndarray:
    pose_landmarks_list = detection_result.pose_landmarks
    annotated_image = np.copy(rgb_image)

    pose_landmark_style = mp_drawing_styles.get_default_pose_landmarks_style()
    pose_connection_style = mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2)

    for pose_landmarks in pose_landmarks_list:
        mp_drawing.draw_landmarks(
            image=annotated_image,
            landmark_list=pose_landmarks,
            connections=vision.PoseLandmarksConnections.POSE_LANDMARKS,
            landmark_drawing_spec=pose_landmark_style,
            connection_drawing_spec=pose_connection_style,
        )

    return annotated_image


def segmentation_mask_to_rgb(detection_result) -> np.ndarray | None:
    """Convert the first pose segmentation mask to a 3-channel uint8 image."""
    if not detection_result.segmentation_masks:
        return None

    segmentation_mask = detection_result.segmentation_masks[0].numpy_view()
    segmentation_mask = np.squeeze(segmentation_mask)
    visualized_mask = (segmentation_mask * 255).astype(np.uint8)
    return np.stack([visualized_mask] * 3, axis=-1)


def overlay_segmentation_mask(rgb_image: np.ndarray, detection_result) -> np.ndarray:
    """Blend a green segmentation mask over the pose image."""
    mask_rgb = segmentation_mask_to_rgb(detection_result)
    if mask_rgb is None:
        return rgb_image

    mask = detection_result.segmentation_masks[0].numpy_view()
    mask = np.squeeze(mask)
    condition = np.stack([mask] * 3, axis=-1) > 0.1
    tint = np.zeros_like(rgb_image)
    tint[:, :, 1] = 255
    blended = np.where(
        condition,
        (rgb_image * (1 - MASK_ALPHA) + tint * MASK_ALPHA).astype(np.uint8),
        rgb_image,
    )
    return blended


def main(camera_index: int = 0, show_mask_window: bool = True) -> None:
    if not _MODEL_PATH.is_file():
        raise FileNotFoundError(
            f"Pose landmarker model not found: {_MODEL_PATH}\n"
            "Download pose_landmarker.task into pysimverse_tests/models/."
        )

    base_options = python.BaseOptions(model_asset_path=str(_MODEL_PATH))
    options = vision.PoseLandmarkerOptions(
        base_options=base_options,
        running_mode=vision.RunningMode.VIDEO,
        output_segmentation_masks=True,
    )

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {camera_index}")

    start_time = time.time()

    with vision.PoseLandmarker.create_from_options(options) as detector:
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
            annotated_image = overlay_segmentation_mask(annotated_image, detection_result)

            cv2.imshow(
                POSE_WINDOW_NAME,
                cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR),
            )

            if show_mask_window:
                mask_rgb = segmentation_mask_to_rgb(detection_result)
                if mask_rgb is not None:
                    cv2.imshow(
                        MASK_WINDOW_NAME,
                        cv2.cvtColor(mask_rgb, cv2.COLOR_RGB2BGR),
                    )

            if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q")):
                break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
