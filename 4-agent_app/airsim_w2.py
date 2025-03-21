import sys
sys.path.append('../external-libraries')
import airsim
import math
import numpy as np
import cv2
import base64
import os
from openai import OpenAI
from gdino import GroundingDINOAPIWrapper, visualize
from PIL import Image
import uuid
from smolagents import tool


@tool
def takeoff() -> str:
    """
    起飞无人机。返回为字符串，表示动作是否成功。
    """
    client = airsim.MultirotorClient()#run in some machine of airsim,otherwise,set ip="" of airsim
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()

    return "成功"



@tool
def land() -> str:
    """
    降落无人机。返回为字符串，表示动作是否成功。
    """
    client = airsim.MultirotorClient()#run in some machine of airsim,otherwise,set ip="" of airsim
    client.landAsync().join()

    return "成功"