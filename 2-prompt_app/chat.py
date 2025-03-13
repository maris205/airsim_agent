# -*- coding: utf-8 -*-
# @Time    : 2025/2/24  16:38
# @Author  : mariswang@rflysim
# @File    : chat.py
# @Software: PyCharm
# @Describe: 
# -*- encoding:utf-8 -*-

from openai import OpenAI
client = OpenAI(
    base_url="https://ark.cn-beijing.volces.com/api/v3",
    # # 获取并配置方舟API Key，并从环境变量中读取。https://www.volcengine.com/docs/82379/1399008#_1-%E8%8E%B7%E5%8F%96%E5%B9%B6%E9%85%8D%E7%BD%AE-api-key
    api_key="cb4ba9cd-bd9d-45d1-bcd0-e6a724dc5b02"
)
completion = client.chat.completions.create(
    # 替换 <Model> 为模型 ID。获取模型 ID：https://www.volcengine.com/docs/82379/1330310
    model="doubao-1-5-pro-32k-250115",
    messages = [
        {"role": "user", "content": "你好"},
    ]
)
print(completion.choices[0].message.content)