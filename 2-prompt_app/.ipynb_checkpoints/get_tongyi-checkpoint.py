# -*- coding: utf-8 -*-
# @Time    : 2025/1/2  15:23
# @Author  : mariswang@rflysim
# @File    : get_tongyi.py
# @Software: PyCharm
# @Describe: 
# -*- encoding:utf-8 -*-

import os
from openai import OpenAI

client = OpenAI(
    # 若没有配置环境变量，请用百炼API Key将下行替换为：api_key="sk-xxx",
    api_key = "sk-7390f325a55841c3b99952f822634eb9",
    base_url = "https://dashscope.aliyuncs.com/compatible-mode/v1",
)
completion = client.chat.completions.create(
    model="qwen-plus",  # 模型列表：https://help.aliyun.com/zh/model-studio/getting-started/models
    messages=[
        {'role': 'user',
         'content': '你是谁？'}],
        temperature=1.9,
    )

print(completion.choices[0].message.content)
#print(completion.model_dump_json())