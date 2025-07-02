import logging
import os
from typing import Dict
from typing import List
from typing import Optional
from urllib.parse import urlparse

import cv2
import numpy as np
import supervision as sv
import requests
from PIL import Image
from io import BytesIO

from dds_cloudapi_sdk.rle_util import rle_to_array

# Define body keypoint connections (COCO format with 17 keypoints)
COCO_KEYPOINTS = [
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle"
]

# Define keypoint connection relationships
COCO_SKELETON = [
    # Head connections
    (0, 1), (0, 2),  # nose to eyes
    (1, 3), (2, 4),  # eyes to ears
    # Torso connections
    (5, 6),  # shoulders
    (5, 7), (7, 9),  # left arm
    (6, 8), (8, 10),  # right arm
    (5, 11), (6, 12),  # shoulders to hips
    (11, 12),  # hips
    # Leg connections
    (11, 13), (13, 15),  # left leg
    (12, 14), (14, 16),  # right leg
]

# Define hand keypoint connections (MediaPipe format with 21 keypoints)
HAND_SKELETON = [
    # Palm connections
    (0, 1), (1, 2), (2, 3), (3, 4),  # thumb
    (0, 5), (5, 6), (6, 7), (7, 8),  # index finger
    (0, 9), (9, 10), (10, 11), (11, 12),  # middle finger
    (0, 13), (13, 14), (14, 15), (15, 16),  # ring finger
    (0, 17), (17, 18), (18, 19), (19, 20),  # pinky
    # Inner palm connections
    (0, 5), (0, 9), (0, 13), (0, 17),  # palm center to fingers
]


class ResultVisualizer:
    """Visualize detection results with boxes, masks, labels and keypoints"""

    def __init__(self, class_names: List[str]):
        """
        Initialize visualizer with class names

        Args:
            class_names: List of class names
        """
        self.classes = [x.strip().lower() for x in class_names if x]
        self.class_name_to_id = {name: id for id, name in enumerate(self.classes)}
        self.class_id_to_name = {id: name for name, id in self.class_name_to_id.items()}

    def _prepare_detections(self, objects: List[Dict]) -> sv.Detections:
        """Convert objects to supervision Detections format"""
        boxes = []
        masks = []
        self.confidences = []
        self.class_names = []
        self.class_ids = []
        self.objects = objects  # Store original objects list for label generation

        for obj in objects:
            # Safely get bounding box
            bbox = obj.get("bbox") or obj.get('region')
            if bbox is None:
                logging.warning(f"Object missing both 'bbox' and 'region': {obj}")
                continue
            boxes.append(bbox)
            if "mask" in obj:
                masks.append(
                    rle_to_array(
                        obj["mask"]["counts"],
                        obj["mask"]["size"][0] * obj["mask"]["size"][1]
                    ).reshape(obj["mask"]["size"])
                )
            self.confidences.append(obj.get("score", 1.0))
            if "category" in obj:
                cls_name = obj["category"].lower().strip()
            elif "caption" in obj:
                cls_name = obj["caption"].lower().strip()
            elif "category_id" in obj:
                cls_name = str(obj["category_id"])
            else:
                cls_name = "unknown"

            # If category not in predefined list, use default category ID (0)
            class_id = self.class_name_to_id.get(cls_name, 0)
            self.class_names.append(cls_name)
            self.class_ids.append(class_id)

        return sv.Detections(
            xyxy=np.array(boxes),
            mask=np.array(masks).astype(bool) if masks else None,
            class_id=np.array(self.class_ids),
        )

    def _get_labels(self) -> List[str]:
        """Generate labels with class names and confidences"""
        logging.info(f"class_names: {self.class_names}, confidences: {self.confidences}")
        labels = []
        for i, (class_name, confidence) in enumerate(zip(self.class_names, self.confidences)):
            label_parts = [f"{class_name} {confidence:.2f}"]

            # Add roc information (if exists)
            if hasattr(self, 'objects') and i < len(self.objects):
                obj = self.objects[i]
                if "roc" in obj and obj["roc"]:
                    label_parts.append(f"ROC: {obj['roc']}")
                if "ocr" in obj and obj["ocr"]:
                    label_parts.append(f"OCR: {obj['ocr']}")

            labels.append(" | ".join(label_parts))
        return labels

    def _draw_pose(self, frame: np.ndarray, pose: List[float]) -> None:
        """
        Draw pose keypoints and skeleton

        Args:
            frame: Image to draw on
            pose: List of keypoints [x1,y1,conf1, x2,y2,conf2, ...]
        """
        if pose is None:
            return

        # Define colors for left/right limbs (BGR format)
        left_color = (0, 0, 255)    # Red - left limb
        right_color = (0, 255, 0)   # Green - right limb
        center_color = (255, 0, 0)  # Blue - center line (torso)

        # Draw keypoints
        for i in range(17):
            kp = pose[i * 4:(i + 1) * 4]
            if kp[2] > 0.5:  # Confidence threshold
                x = int(kp[0])
                y = int(kp[1])
                # Select color based on keypoint position
                if i in [0, 1, 3, 5, 7, 9, 11, 13, 15]:  # Left keypoints
                    cv2.circle(frame, (x, y), 3, left_color, -1)
                elif i in [2, 4, 6, 8, 10, 12, 14, 16]:  # Right keypoints
                    cv2.circle(frame, (x, y), 3, right_color, -1)

        # Draw skeleton connections
        for start_idx, end_idx in COCO_SKELETON:
            start_kp = pose[start_idx * 4:(start_idx + 1) * 4]
            end_kp = pose[end_idx * 4:(end_idx + 1) * 4]

            # Only draw line if both keypoints have confidence above threshold
            if start_kp[2] > 0.5 and end_kp[2] > 0.5:
                start_point = (int(start_kp[0]), int(start_kp[1]))
                end_point = (int(end_kp[0]), int(end_kp[1]))

                # Select color based on connection type
                if (start_idx, end_idx) in [(5, 6), (11, 12), (5, 11), (6, 12)
                                            ]:  # Torso connections (shoulders, hips, shoulders to hips)
                    line_color = center_color
                elif start_idx in [0, 1, 3, 5, 7, 9, 11, 13, 15] or end_idx in [0, 1, 3, 5, 7, 9, 11, 13, 15]:  # Left connections
                    line_color = left_color
                else:  # Right connections
                    line_color = right_color

                cv2.line(frame, start_point, end_point, line_color, 2)

    def _draw_hand(self, frame: np.ndarray, hand: List[float]) -> None:
        """
        Draw hand keypoints and skeleton

        Args:
            frame: Image to draw on
            hand: List of keypoints [x1,y1,conf1, x2,y2,conf2, ...]
        """
        if hand is None:
            return

        # Define colors for each finger (BGR format)
        finger_colors = {
            'thumb': (0, 0, 255),      # Red - thumb
            'index': (0, 255, 0),      # Green - index finger
            'middle': (255, 0, 0),     # Blue - middle finger
            'ring': (255, 255, 0),     # Cyan - ring finger
            'pinky': (255, 0, 255)     # Purple - pinky
        }

        # Define keypoint ranges for each finger
        finger_ranges = {
            'thumb': (0, 4),    # 0-4
            'index': (5, 8),    # 5-8
            'middle': (9, 12),  # 9-12
            'ring': (13, 16),   # 13-16
            'pinky': (17, 20)   # 17-20
        }

        # Draw keypoints and connections
        for finger, (start_idx, end_idx) in finger_ranges.items():
            color = finger_colors[finger]

            # Draw keypoints
            for i in range(start_idx, end_idx + 1):
                kp = hand[i * 4:(i + 1) * 4]
                if kp[2] > 0.5:  # Confidence threshold
                    x = int(kp[0])
                    y = int(kp[1])
                    cv2.circle(frame, (x, y), 3, color, -1)  # Solid circle

            # Draw finger connections
            for i in range(start_idx, end_idx):
                start_kp = hand[i * 4:(i + 1) * 4]
                end_kp = hand[(i + 1) * 4:(i + 2) * 4]

                if start_kp[2] > 0.5 and end_kp[2] > 0.5:
                    start_point = (int(start_kp[0]), int(start_kp[1]))
                    end_point = (int(end_kp[0]), int(end_kp[1]))
                    cv2.line(frame, start_point, end_point, color, 2)

            # Draw connection to palm center
            if start_idx > 0:  # Not thumb
                palm_kp = hand[0:4]  # Palm center point
                finger_kp = hand[start_idx * 4:(start_idx + 1) * 4]

                if palm_kp[2] > 0.5 and finger_kp[2] > 0.5:
                    palm_point = (int(palm_kp[0]), int(palm_kp[1]))
                    finger_point = (int(finger_kp[0]), int(finger_kp[1]))
                    cv2.line(frame, palm_point, finger_point, color, 2)

    def visualize(
        self,
        image_path: str,
        objects: List[Dict],
        output_dir: str,
        show_mask: bool = True,
        show_box: bool = True,
        show_label: bool = True,
        show_pose: bool = True
    ) -> str:
        """
        Visualize detection results on image

        Args:
            image_path: Path to input image or image URL
            objects: List of detection objects
            output_dir: Directory to save output image
            show_mask: Whether to show masks
            show_box: Whether to show boxes
            show_label: Whether to show labels
            show_pose: Whether to show pose keypoints and skeleton

        Returns:
            str: Path to saved image
        """
        # Read image from local file or URL
        if urlparse(image_path).scheme in ('http', 'https'):
            try:
                response = requests.get(image_path)
                response.raise_for_status()
                # Read image using PIL
                pil_image = Image.open(BytesIO(response.content))
                # Ensure image is in RGB mode
                if pil_image.mode != 'RGB':
                    pil_image = pil_image.convert('RGB')
                # Convert to numpy array
                img = np.array(pil_image)
                # Convert to BGR format (OpenCV format)
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            except Exception as e:
                raise ValueError(f"Failed to read image from URL: {image_path}, error: {str(e)}")
        else:
            img = cv2.imread(image_path)
            if img is None:
                raise ValueError(f"Failed to read image: {image_path}")

        # Prepare detections
        detections = self._prepare_detections(objects)
        annotated_frame = img.copy()

        # Draw boxes and labels
        if show_box:
            box_annotator = sv.BoxAnnotator()
            annotated_frame = box_annotator.annotate(scene=annotated_frame, detections=detections)

        if show_label:
            label_annotator = sv.LabelAnnotator()
            labels = self._get_labels()
            annotated_frame = label_annotator.annotate(
                scene=annotated_frame,
                detections=detections,
                labels=labels
            )

        # Draw masks
        if show_mask:
            mask_annotator = sv.MaskAnnotator()
            annotated_frame = mask_annotator.annotate(scene=annotated_frame, detections=detections)

        # Draw pose keypoints and skeleton
        if show_pose:
            for obj in objects:
                if 'pose' in obj:
                    self._draw_pose(annotated_frame, obj['pose'])

        # Draw hand keypoints and skeleton
        if show_pose:
            for obj in objects:
                if 'hand' in obj:
                    self._draw_hand(annotated_frame, obj['hand'])

        # Save result
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "annotated_image.jpg")
        cv2.imwrite(output_path, annotated_frame)

        return output_path


def visualize_result(
    image_path: str,
    result: Dict,
    output_dir: str,
    class_names: Optional[List[str]] = None,
    show_mask: bool = True,
    show_box: bool = True,
    show_label: bool = True
) -> str:
    """
    Convenience function to visualize detection results

    Args:
        image_path: Path to input image or image URL
        result: Detection result dictionary
        output_dir: Directory to save output image
        class_names: List of class names, if None will be extracted from result
        show_mask: Whether to show masks
        show_box: Whether to show boxes
        show_label: Whether to show labels

    Returns:
        str: Path to saved image
    """
    # Extract objects from result
    objects = result.get('objects', [])
    if not objects:
        raise ValueError("No objects found in result")

    # Extract class names if not provided
    if class_names is None:
        class_names = list(set(
            obj['category'] if 'category' in obj else str(obj.get('category_id', 'unknown'))
            for obj in objects
        ))

    # Create visualizer and visualize
    visualizer = ResultVisualizer(class_names)
    return visualizer.visualize(
        image_path=image_path,
        objects=objects,
        output_dir=output_dir,
        show_mask=show_mask,
        show_box=show_box,
        show_label=show_label
    )
