import sys
sys.path.append('../external-libraries')

import airsim
from langchain_core.tools import tool
from typing import Dict

# 初始化一个全局的AirSim客户端
client = airsim.MultirotorClient()
client.confirmConnection()

@tool
def takeoff(drone_id: str) -> str:
    """
    命令指定的无人机起飞并悬停。
    :param drone_id: 要控制的无人机的名称 (例如, 'Drone1').
    """
    print(f"Executing: takeoff for {drone_id}")
    # 必须先解锁API控制
    client.enableApiControl(True, vehicle_name=drone_id)
    client.armDisarm(True, vehicle_name=drone_id)
    # 异步起飞并等待完成
    client.takeoffAsync(vehicle_name=drone_id).join()
    return f"无人机 {drone_id} 已经成功起飞。"

@tool
def fly_to_position(drone_id: str, x: float, y: float, z: float) -> str:
    """
    命令指定的无人机以5米/秒的速度飞往一个NED坐标。
    Z坐标是负数表示向上。
    :param drone_id: 要控制的无人机的名称。
    :param x: 目标位置的X坐标。
    :param y: 目标位置的Y坐标。
    :param z: 目标位置的Z坐标 (负数代表高度)。
    """
    print(f"Executing: fly_to_position for {drone_id} to ({x}, {y}, {z})")
    client.moveToPositionAsync(x, y, z, 5, vehicle_name=drone_id).join()
    return f"无人机 {drone_id} 已到达目标位置 ({x}, {y}, {z})。"

@tool
def get_drone_state(drone_id: str) -> Dict:
    """
    获取指定无人机的当前状态，包括位置和姿态。
    :param drone_id: 要查询的无人机的名称。
    """
    print(f"Executing: get_drone_state for {drone_id}")
    state = client.getMultirotorState(vehicle_name=drone_id)
    # 返回一个字典，方便LLM处理
    return {
        "position": {
            "x_val": state.kinematics_estimated.position.x_val,
            "y_val": state.kinematics_estimated.position.y_val,
            "z_val": state.kinematics_estimated.position.z_val,
        },
        "orientation": {
            "w_val": state.kinematics_estimated.orientation.w_val,
            "x_val": state.kinematics_estimated.orientation.x_val,
            "y_val": state.kinematics_estimated.orientation.y_val,
            "z_val": state.kinematics_estimated.orientation.z_val,
        }
    }
