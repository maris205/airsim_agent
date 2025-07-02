import logging
from typing import Any
from typing import Dict
from typing import List

import cv2

from dds_cloudapi_sdk.image_resizer import image_to_base64
from dds_cloudapi_sdk.image_resizer import resize_image
from dds_cloudapi_sdk.rle_util import mask_to_rle
from dds_cloudapi_sdk.rle_util import rle_to_array
from dds_cloudapi_sdk.tasks.base import BaseTask


class V2Task(BaseTask):
    _api_path = None
    _api_body = None
    result = None
    _resize_info = None
    resizable_targets = (
        "bbox",
        "mask",
        "pose_keypoints",
        "hand_keypoints",
    )

    def __init__(
        self,
        api_path: str,
        api_body: dict = None,
        resize_info: dict = None,
    ):
        super().__init__()
        self._api_path = api_path
        self._api_body = api_body
        self._resize_info = resize_info

    @property
    def api_path(self):
        return self._api_path

    @property
    def api_body(self):
        return self._api_body or {}

    def format_result(self, result: dict) -> dict:
        try:
            if not self._resize_info:
                return result

            logging.debug(f"resize original result: {result}")
            for item in result['objects']:
                if item.get('bbox'):
                    item['bbox'] = self.resize_bbox(item['bbox'])
                if item.get('mask'):
                    item['mask'] = self.resize_mask(item['mask'])
                if item.get('pose'):
                    item['pose'] = self.resize_keypoints(item['pose'])
                if item.get('hand'):
                    item['hand'] = self.resize_keypoints(item['hand'])
            return result
        except Exception as e:
            logging.exception(
                f"Error formatting result: {result}, "
                f"resize_info: {self._resize_info}, error: {e}"
            )
            return result

    def resize_bbox(self, bbox: list) -> list:
        return [int(coord / self._resize_info['ratio']) for coord in bbox]

    def resize_mask(self, mask: dict) -> dict:
        img = rle_to_array(
            mask['counts'],
            mask['size'][0] * mask['size'][1]
        ).reshape(mask['size'])
        img = cv2.resize(
            img,
            (self._resize_info['original_width'], self._resize_info['original_height'])
        )
        return {
            'counts': mask_to_rle(img, encode=True),
            'size': [
                self._resize_info['original_height'],
                self._resize_info['original_width']
            ]
        }

    def resize_keypoints(self, keypoints: list) -> list:
        return [
            int(v / self._resize_info['ratio']) if i % 4 <= 1 else v
            for i, v in enumerate(keypoints)
        ]

    @property
    def result(self):
        return self._result

    @property
    def api_trigger_url(self):
        if self.config.endpoint.startswith("http"):
            return f"{self.config.endpoint}{self.api_path}"
        else:
            return f"https://{self.config.endpoint}{self.api_path}"

    @property
    def api_check_url(self):
        if self.config.endpoint.startswith("http"):
            return f"{self.config.endpoint}/v2/task_status/{self.task_uuid}"
        else:
            return f"https://{self.config.endpoint}/v2/task_status/{self.task_uuid}"

    @classmethod
    def is_resizable(cls, targets: List[str]) -> bool:
        return targets and all(target in cls.resizable_targets for target in targets)

    @classmethod
    def image_max_size(cls, api_path: str) -> int:
        if api_path == "/v2/task/trex/detection":
            return 1333
        elif api_path == "/v2/task/application/change_cloth_color":
            return 2048
        else:
            return 1536


def create_task_with_local_image_auto_resize(
    api_path: str,
    api_body_without_image: Dict[str, Any],
    image_path: str,
    max_size: int = None,
) -> V2Task:
    api_body = api_body_without_image or {}

    if V2Task.is_resizable(api_body.get('targets')):
        max_size = max_size or V2Task.image_max_size(api_path)
        image_data, resize_info = resize_image(image_path, max_size)
        api_body['image'] = image_to_base64(image_data)
    else:
        api_body['image'] = image_to_base64(image_path)
        resize_info = None

    return V2Task(api_path, api_body, resize_info)
