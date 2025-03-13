# -*- coding: utf-8 -*-
# @Time    : 2025/2/24  17:05
# @Author  : mariswang@rflysim
# @File    : chat_img.py
# @Software: PyCharm
# @Describe: 
# -*- encoding:utf-8 -*-
#pip install --upgrade "openai>=1.0"

import os
from openai import OpenAI

client = OpenAI(
    api_key="cb4ba9cd-bd9d-45d1-bcd0-e6a724dc5b02",
    base_url="https://ark.cn-beijing.volces.com/api/v3",
)

# Image input:
response = client.chat.completions.create(
    model="doubao-1-5-vision-pro-32k-250115",
    messages=[
        {
            "role": "user",
            "content": [
                {"type": "text", "text": "图片中有无人吗？"},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": "https://yt-shanghai.tos-cn-shanghai.volces.com/tello.jpg"
                    }
                },
            ],
        }
    ],
)

print(response.choices[0])