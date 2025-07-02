# 1. Initialize the client with your API token.
import logging

import requests

from dds_cloudapi_sdk import Client
from dds_cloudapi_sdk import Config
from dds_cloudapi_sdk.tasks.v2_task import V2Task
from dds_cloudapi_sdk.tasks.v2_task import create_task_with_local_image_auto_resize
from dds_cloudapi_sdk.visualization_util import visualize_result

token = "Your API Token Here"
config = Config(token)
client = Client(config)


def test_v2_grounding_dino_detection():
    task = V2Task(api_path="/v2/task/grounding_dino/detection", api_body={
        "image": "https://dds-frontend.oss-accelerate.aliyuncs.com/static_files/playground/grounding_DINO-1.6/02.jpg",
        "prompt": {
            "type": "text",
            "text": "wolf.dog.butterfly",
        },
        "model": "GroundingDino-1.6-Pro",
        "targets": ["bbox"],
    })
    client.run_task(task)
    print(task.task_uuid, task.status)
    return task


def test_v2_trex_detection():
    task = V2Task(api_path="/v2/task/trex/detection", api_body={
        "model": "T-Rex-2.0",
        "image": "https://dds-frontend.oss-accelerate.aliyuncs.com/static_files/playground/ivp/v2/right_17.jpeg",
        "targets": ["bbox", "embedding"],
        "prompt": {
            "type": "visual_images",
            "visual_images": [
                {
                    "image": "https://dds-frontend.oss-accelerate.aliyuncs.com/static_files/playground/ivp/v2/left_17.jpeg",
                    "interactions": [
                        {
                            "type": "rect",
                            "category_id": 12,
                            "rect": [600.0954692556635, 386.2447411003236, 688.690938511327, 474.8402103559871],
                        },
                    ]
                }
            ]
        }
    })
    client.run_task(task)
    print(task.task_uuid, task.status)
    # print(task.result)
    visualize_result(
        image_path="https://dds-frontend.oss-accelerate.aliyuncs.com/static_files/playground/ivp/v2/right_17.jpeg",
        result=task.result, output_dir="images/ivp_output")
    return task


def test_v2_dinox_detection():
    task = V2Task(api_path="/v2/task/dinox/detection", api_body={
        "model": "DINO-X-1.0",
        "image": "https://dds-frontend.oss-accelerate.aliyuncs.com/static_files/playground/grounding_DINO-1.6/02.jpg",
        "prompt": {
            "type": "text",
            "text": "wolf.dog.butterfly"
        },
        "targets": ["bbox"],
        "bbox_threshold": 0.25,
        "iou_threshold": 0.8
    })
    client.run_task(task)
    print(task.task_uuid, task.status)
    # print(task.result)
    visualize_result(
        image_path="https://dds-frontend.oss-accelerate.aliyuncs.com/static_files/playground/grounding_DINO-1.6/02.jpg",
        result=task.result, output_dir="images/output")
    return task


def test_v2_dino_xseek_detection():
    task = V2Task(api_path="/v2/task/dino_xseek/detection", api_body={
        "model": "DINO-XSeek-1.0",
        "image": "https://dds-frontend.oss-accelerate.aliyuncs.com/static_files/playground/grounding_DINO-1.6/02.jpg",
        "prompt": {
            "type": "text",
            "text": "butterfly"
        },
        "targets": ["bbox"],
        "bbox_threshold": 0.25,
        "iou_threshold": 0.8
    })
    client.run_task(task)
    print(task.task_uuid, task.status)
    # print(task.result)
    visualize_result(
        image_path="https://dds-frontend.oss-accelerate.aliyuncs.com/static_files/playground/grounding_DINO-1.6/02.jpg",
        result=task.result, output_dir="images/output")
    return task


def test_v2_dino_xseek_detection2():
    image_path = "https://dds-frontend.oss-accelerate.aliyuncs.com/static_files/playground/dino-x2/referring/11.png"
    task = V2Task(api_path="/v2/task/dino_xseek/detection", api_body={
        "model": "DINO-XSeek-1.0",
        "image": image_path,
        "prompt": {
            "type": "text",
            "text": "apple"
        },
        "targets": ["bbox"],
        "bbox_threshold": 0.25,
        "iou_threshold": 0.8
    })
    client.run_task(task)
    print(task.task_uuid, task.status)
    # print(task.result)
    visualize_result(
        image_path=image_path,
        result=task.result, output_dir="images/output")
    return task


def test_v2_dinox_detection_local_image_scale():
    image_path = "images/333.jpg"
    output_dir = "images/333_output"
    task = create_task_with_local_image_auto_resize(
        api_path="/v2/task/dinox/detection",
        api_body_without_image={
            "model": "DINO-X-1.0",
            "prompt": {
                "type": "text",
                "text": "person.hand"
            },
            "targets": ["bbox", "mask", "pose_keypoints", "hand_keypoints"],
            "bbox_threshold": 0.25,
            "iou_threshold": 0.8
        },
        image_path=image_path, max_size=1333
    )
    client.run_task(task)
    print(task.task_uuid, task.status)
    # print(task.result)
    visualize_result(image_path=image_path, result=task.result, output_dir=output_dir)
    return task


def test_v2_dinox_region_vl():
    task = V2Task(api_path="/v2/task/dinox/region_vl", api_body={
        "model": "DINO-X-1.0",
        "image": "https://dds-frontend.oss-accelerate.aliyuncs.com/static_files/playground/grounding_DINO-1.6/02.jpg",
        "prompt": {
            "type": "text",
            "text": "cat.dog.person"
        },
        "regions": [
            [54.97325134277344, 18.49270629882812, 252.85134887695312, 195.4311981201172],
            [59.78119507908616, 186.52658172231986, 237.2996485061512, 359.2963532513181]
        ],
        "targets": ["caption", "roc", "ocr"],
    })
    client.run_task(task)
    print(task.task_uuid, task.status)
    print(task.result)
    visualize_result(
        image_path="https://dds-frontend.oss-accelerate.aliyuncs.com/static_files/playground/grounding_DINO-1.6/02.jpg",
        result=task.result,
        output_dir="images/output")
    return task


def test_v2_application_change_cloth_color():
    task = V2Task(
        api_path="/v2/task/application/change_cloth_color",
        api_body={
            "image": "https://algosplt.oss-cn-shenzhen.aliyuncs.com/test_files/9_changed.png",
            "original_rgb": [
                100,
                100,
                100
            ],  # [r_int, g_int, b_int],
            "target_rgb": [
                0,
                100,
                100
            ],  # [r_int, g_int, b_int],
            "cloth_category": "Q",
        })
    client.run_task(task)
    print(task.task_uuid, task.status)
    # print(task.result)
    return task


def test_v2_application_change_cloth_color_with_local_image():
    image_path = "images/9_changed.png"
    output_dir = "images/9_changed_output"
    task = create_task_with_local_image_auto_resize(
        api_path="/v2/task/application/change_cloth_color",
        api_body_without_image={
            "original_rgb": [
                100,
                100,
                100
            ],  # [r_int, g_int, b_int],
            "target_rgb": [
                0,
                100,
                100
            ],  # [r_int, g_int, b_int],
            "cloth_category": "Q",
        }, image_path=image_path)
    client.run_task(task)
    print(task.task_uuid, task.status)
    # print(task.result)
    return task


logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)


if __name__ == "__main__":
    # test_v2_grounding_dino_detection()
    test_v2_trex_detection()
    # test_v2_dinox_detection()
    # test_v2_dino_xseek_detection()
    # test_v2_dino_xseek_detection2()
    # test_v2_dinox_region_vl()
    # test_v2_application_change_cloth_color()
    # test_v2_application_change_cloth_color_with_local_image()
    # test_v2_dinox_detection_local_image_scale()
    pass
