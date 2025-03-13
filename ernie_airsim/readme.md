# 文心大模型ernie驱动的airsim
## 1. 项目简介
本项目是基于ernie的airsim驱动， 

让airsim的达到原论文"工程师"在环---->"用户在环"的目标

参考论文：
ChatGPT for Robotics: Design Principles and Model Abilities
[链接](https://www.microsoft.com/en-us/research/group/autonomous-systems-group-robotics/articles/chatgpt-for-robotics/)

## 2. 项目结构
需要安装的库：
pip install numpy  

sys_prompts: 背景知识

prompts: 回话历史，当做短期知识库/short memory,message history

langchain：向量索引知识库

airsim_wrapper: airsim api的封装

ernie_airsim: ernie驱动airsim的主程序

## 3. 使用方法
参考几个jupyter文件

以及文心社区的项目
[链接](https://aistudio.baidu.com/projectdetail/7158159?sUid=358971&shared=1&ts=1701667214445)
