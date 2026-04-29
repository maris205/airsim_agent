"""
airsim_tools.py — 多无人机协同工具模块（纯Python，无框架依赖）

提供 AirSim 无人机控制函数和 LLM 调用封装。
"""

import sys
sys.path.append('../external-libraries')

import os
import subprocess
import time
import airsim
from openai import OpenAI

# ============================================================
# AirSim 场景管理
# ============================================================

def restart_airsim(wait=10):
    """
    重启 AirSim 仿真器。

    Args:
        wait: 等待启动的秒数，默认10秒
    """
    os.system("sudo pkill -9 -f BlocksV2")
    time.sleep(2)
    cmd = "/home/gpx/start_services_v4.sh"
    subprocess.Popen(cmd, shell=True)
    print(f"正在重启 AirSim，等待 {wait} 秒...")
    time.sleep(wait)
    print("AirSim 已重启")

def restart_airsim_7(wait=10):
    """
    重启 AirSim 仿真器。

    Args:
        wait: 等待启动的秒数，默认10秒
    """
    os.system("sudo pkill -9 -f BlocksV2")
    time.sleep(2)
    cmd = "/home/gpx/start_services_v5.sh"
    subprocess.Popen(cmd, shell=True)
    print(f"正在重启 AirSim，等待 {wait} 秒...")
    time.sleep(wait)
    print("AirSim 已重启")

def apply_settings(settings_file, restart=True):
    """
    应用指定的 AirSim 配置文件并重启仿真器。

    Args:
        settings_file: settings.json 文件路径（如 '1-6-settings.json'）
        restart: 是否自动重启 AirSim，默认 True
    """
    target = os.path.expanduser("~/Documents/AirSim/settings.json")
    os.makedirs(os.path.dirname(target), exist_ok=True)

    with open(settings_file, 'r') as f:
        content = f.read()
    with open(target, 'w') as f:
        f.write(content)
    print(f"已复制 {settings_file} → {target}")

    if restart:
        restart_airsim()


# ============================================================
# AirSim 连接与控制
# ============================================================

def connect_airsim():
    """连接到 AirSim 模拟器，返回客户端对象。"""
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("已连接到 AirSim 模拟器")
    return client


def takeoff(client, drone_id):
    """
    命令指定无人机起飞。

    Args:
        client: AirSim 客户端
        drone_id: 无人机名称，如 'Drone1'
    """
    client.enableApiControl(True, vehicle_name=drone_id)
    client.armDisarm(True, vehicle_name=drone_id)
    client.takeoffAsync(vehicle_name=drone_id).join()
    print(f"[{drone_id}] 起飞完成")


def fly_to(client, drone_id, x, y, z, speed=5):
    """
    命令无人机飞往指定 NED 坐标。

    Args:
        client: AirSim 客户端
        drone_id: 无人机名称
        x, y, z: 目标坐标（NED坐标系，z为负表示向上）
        speed: 飞行速度，默认5米/秒
    """
    client.moveToPositionAsync(x, y, z, speed, vehicle_name=drone_id).join()
    print(f"[{drone_id}] 已到达 ({x}, {y}, {z})")


def get_state(client, drone_id):
    """
    获取无人机当前位置。

    Returns:
        dict: {"x": float, "y": float, "z": float}
    """
    state = client.getMultirotorState(vehicle_name=drone_id)
    pos = state.kinematics_estimated.position
    return {"x": round(pos.x_val, 2), "y": round(pos.y_val, 2), "z": round(pos.z_val, 2)}


# ============================================================
# LLM 调用封装（使用 OpenAI SDK 调用豆包模型）
# ============================================================

# 豆包模型配置
LLM_BASE_URL = "https://ark.cn-beijing.volces.com/api/v3"
LLM_API_KEY = "80e68c38-22cb-4f71-9377-0768c4d7fe15"
LLM_MODEL = "doubao-seed-2-0-pro-260215"  # 替换为你的模型 ID


def call_llm(prompt, system="你是一个无人机任务规划助手，请简洁回答。"):
    """
    调用 LLM 并返回文本响应。

    Args:
        prompt: 用户提示
        system: 系统提示（可选）

    Returns:
        str: LLM 的回复文本
    """
    client = OpenAI(base_url=LLM_BASE_URL, api_key=LLM_API_KEY)
    response = client.chat.completions.create(
        model=LLM_MODEL,
        temperature=0.1,
        messages=[
            {"role": "system", "content": system},
            {"role": "user", "content": prompt}
        ]
    )
    return response.choices[0].message.content
