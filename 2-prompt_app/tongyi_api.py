# -*- coding: utf-8 -*-
# @Time    : 2025/1/2  15:23
# @Author  : mariswang@rflysim
# @File    : get_tongyi.py
# @Software: PyCharm
# @Describe:
# -*- encoding:utf-8 -*-
import json
import os
from openai import OpenAI


def clean_code_markers(text):
    # 定义需要去除的前缀和后缀
    prefix = '```json'
    suffix = '```'

    # 检查字符串是否以指定的前缀开头，以指定的后缀结尾
    if text.startswith(prefix) and text.endswith(suffix):
        # 去除前缀和后缀
        cleaned_text = text[len(prefix):-len(suffix)]
    else:
        # 如果没有指定的前缀或后缀，则返回原始字符串
        cleaned_text = text

    return cleaned_text

def chat(content):
    """
    调用通义api
    :param content:
    :return:
    """
    client = OpenAI(
        # 若没有配置环境变量，请用百炼API Key将下行替换为：api_key="sk-xxx",
        api_key = "sk-7390f325a55841c3b99952f822634eb9",
        base_url = "https://dashscope.aliyuncs.com/compatible-mode/v1",
    )
    completion = client.chat.completions.create(
        model="qwen-plus",  # 模型列表：https://help.aliyun.com/zh/model-studio/getting-started/models
        messages=[
            {'role': 'user',
             'content': content}],
        temperature=1.9,
    )

    #print(completion.choices[0].message.content)
    #print(completion.model_dump_json())
    content = completion.choices[0].message.content
    return content

def get_yuliao(category="出行"):
    """
    获取语料
    :param category:
    :return:
    """
    content = f"""
        请帮我生成源语言为粤语，目标语言为中文普通话的句子对

        要求为：
        1 生成的内容要地道的广东话，请用广州本地人常用的口语化粤语来表达
        2 句子的字数不少于60字
        3 类别为{category}相关的
    """+"""
        请生成10对这样的语料
        输出格式为json格式，最外层为数组，内容为一行一个句子对，具体如下：
        [
        {“source_text”:”这里是粤语1”，“target_text”:”这里是普通话1”},
        {“source_text”:”这里是粤语2”，“target_text”:”这里是普通话2”},
        {“source_text”:”这里是粤语3”，“target_text”:”这里是普通话3”},
        {“source_text”:”这里是粤语4”，“target_text”:”这里是普通话4”},
        ]
        请只输出json字符串即可，无需其他额外的字符内容，否则json无法解析
        请确保json的格式正确，否则无法入库
    """
    output = chat(content)
    print(output)
    output = clean_code_markers(output)
    return json.loads(output)

if __name__ == '__main__':
    data = get_yuliao()
    print(data)