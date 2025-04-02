import sys
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

print("场景中所有对象的名称:", all_objects)