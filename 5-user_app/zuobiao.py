# -*- coding: utf-8 -*-
# @Time    : 2024/9/4  11:31
# @Author  : cxq213@foxmail.com
# @File    : zuobiao.py
# @Software: PyCharm
# @Describe: 
# -*- encoding:utf-8 -*-
import sys
sys.path.append('../external-libraries')
import airsim
import math
import numpy as np

"""
功能:按照场景中的，物体名字获取坐标
"""

# 创建 AirSim 客户端
import time
import cv2
import matplotlib.pyplot as plt

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()


# 获取场景中所有对象的名称
all_objects = client.simListSceneObjects()

# 假设圆球的名称包含 "Ball"（通常情况下，名称可能包含 "Ball" 字样）
# object_name = "Ball"
object_name = "HouseForAirsim_C_1"
# object_name = "SK_West_Tank_M1A1Abrams2_25"
for obj_name in all_objects:
    if object_name in obj_name:
        pose = client.simGetObjectPose(obj_name)
        print(f"Object Name: {obj_name}")
        print(f"Position: x = {pose.position.x_val}, y = {pose.position.y_val}, z = {pose.position.z_val}")
        print(f"Position: {pose.position.x_val},{pose.position.y_val}, {pose.position.z_val}")
        print(f"Orientation: pitch = {pose.orientation.x_val}, roll = {pose.orientation.y_val}, yaw = {pose.orientation.z_val}")
