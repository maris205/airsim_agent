# -*- coding: utf-8 -*-
# @Time    : 2023/11/18  23:12
# @Author  : mariswang@rflysim
# @File    : ernie_airsim.py
# @Software: PyCharm
# @Describe: 
# -*- encoding:utf-8 -*-
import time

import erniebot
import re
from airsim_wrapper import *

#api key
erniebot.api_type = "aistudio"
erniebot.access_token = "85e4da127a707245f3d26ec617cc4d982bcc5a6c" # "your access token
MY_MODEL = "ernie-4.0" #ernie-bot",ernie-bot-turbo,ernie-bot-4,ernie-bot-8k,ernie-text-embedding,ernie-vilg-v2
aw = AirSimWrapper() #初始化airsim，airsim基本功能调用


class ErnieAirSim:
    def __init__(self, system_prompts="system_prompts/airsim_basic_cn.txt", prompt="prompts/airsim_basic_cn.txt", chat_history=[]):
        # 系统提示
        self.sysprompt = open(system_prompts, "r", encoding="utf8").read()
        knowledge_prompt = open(prompt, "r", encoding="utf8").read()

        # 先加入样例对话，全局变量
        self.chat_history = []

        example_msg = [
            {
                "role": "user",
                "content": "向上移动 10 个单位"
            },
            {
                "role": "assistant",
                "content": """```python
        aw.fly_to([aw.get_drone_position()[0], aw.get_drone_position()[1], aw.get_drone_position()[2]+10])
        ```

        此代码使用“fly_to()”函数将无人机移动到距当前位置 10 个单位的新位置。它通过使用 get_drone_position() 获取无人机的当前位置，然后创建一个具有相同 X 和 Y 坐标但 Z 坐标增加 10 的新列表来实现这一点。然后无人机将使用以下命令飞到这个新位置： `fly_to()`。 """}
        ]
        self.chat_history.extend(example_msg)

        # 通过聊天历史，加入知识库
        self.ask(knowledge_prompt)

    # 调用ernie的chat api，包含历史记录，多轮对话
    def ask(self, prompt):
        # 加入用户输入的prompt
        self.chat_history.append(
            {
                "role": "user",
                "content": prompt,
            }
        )
        completion = erniebot.ChatCompletion.create(
            model=MY_MODEL,
            messages=self.chat_history,
            system=self.sysprompt,  # 系统提示
            # temperature=0.000001,
            top_p=0,
        )

        #print("completion-------------------:", completion)

        # 加入机器人回复，相当于保存全部的历史记录，多轮对话
        self.chat_history.append(
            {
                "role": "assistant",
                "content": completion.get_result(),
            }
        )

        # print("chat_history-------------------:", json.dumps(chat_history))
        return self.chat_history[-1]["content"]

    # 解析python代码
    def extract_python_code(self, content):
        """
        Extracts the python code from a response.
        :param content:
        :return:
        """
        code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)
        code_blocks = code_block_regex.findall(content)
        if code_blocks:
            full_code = "\n".join(code_blocks)

            if full_code.startswith("python"):
                full_code = full_code[7:]

            return full_code
        else:
            return None

    def process(self, command):
        #step 1, ask ernie
        response = self.ask(command)

        #step 2, extract python code
        python_code = self.extract_python_code(response)

        #step 3, exec python code
        if python_code:
            exec(python_code)
        return python_code

if __name__=="__main__":
    ernie_airsim = ErnieAirSim()
    command = "起飞"
    python_code = ernie_airsim.process(command)
    print("python_code:", python_code)

    time.sleep(5)
    command = "飞到汽车上方"
    python_code = ernie_airsim.process(command)
    print("python_code:", python_code)
