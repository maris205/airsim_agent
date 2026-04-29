# -*- coding: utf-8 -*-
"""AirSim Hover Environment — Gymnasium interface for DreamerV3."""
import sys
sys.path.append('../../external-libraries')

import numpy as np
import airsim
import gymnasium as gym
from gymnasium import spaces
from PIL import Image as PILImage


class AirSimHoverEnv(gym.Env):
    """无人机定点悬停任务。

    目标：在指定位置 (target_x, target_y, target_z) 保持稳定悬停。
    观测：64x64 RGB图像 + 6维状态向量 (position xyz + velocity xyz)
    动作：连续4维 (pitch, roll, yaw_rate, throttle)，范围 [-1, 1]
    奖励：靠近目标 +1，姿态倾斜 -0.1，坠机 -10
    """

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, target=(0, 0, -5), max_steps=300, img_size=64):
        super().__init__()
        self.target = np.array(target, dtype=np.float32)
        self.max_steps = max_steps
        self.img_size = img_size
        self.step_count = 0

        self.observation_space = spaces.Dict({
            "image": spaces.Box(0, 255, (img_size, img_size, 3), dtype=np.uint8),
            "state": spaces.Box(-np.inf, np.inf, (6,), dtype=np.float32),
        })
        self.action_space = spaces.Box(-1.0, 1.0, (4,), dtype=np.float32)

        self.client = None
        self.drone = "Drone1"

    def _connect(self):
        if self.client is None:
            self.client = airsim.MultirotorClient()
            self.client.confirmConnection()

    def _get_obs(self):
        state = self.client.getMultirotorState(vehicle_name=self.drone)
        pos = state.kinematics_estimated.position
        vel = state.kinematics_estimated.linear_velocity
        state_vec = np.array([
            pos.x_val, pos.y_val, pos.z_val,
            vel.x_val, vel.y_val, vel.z_val
        ], dtype=np.float32)

        responses = self.client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
        ], vehicle_name=self.drone)

        if responses and responses[0].width > 0:
            img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
            img = img.reshape(responses[0].height, responses[0].width, 3)
            img = np.array(PILImage.fromarray(img).resize((self.img_size, self.img_size)))
        else:
            img = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)

        return {"image": img, "state": state_vec}

    def _compute_reward(self, state_vec):
        pos = state_vec[:3]
        dist = np.linalg.norm(pos - self.target)

        collision = self.client.simGetCollisionInfo(vehicle_name=self.drone)
        if collision.has_collided:
            return -10.0, True

        if abs(pos[2]) < 0.3:
            return -10.0, True

        ori = self.client.getMultirotorState(vehicle_name=self.drone).kinematics_estimated.orientation
        tilt = abs(ori.x_val) + abs(ori.y_val)

        reward = max(0, 1.0 - dist / 5.0)
        reward -= 0.1 * tilt

        return float(reward), False

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._connect()
        self.client.reset()
        self.client.enableApiControl(True, vehicle_name=self.drone)
        self.client.armDisarm(True, vehicle_name=self.drone)
        self.client.takeoffAsync(vehicle_name=self.drone).join()
        self.client.moveToPositionAsync(
            self.target[0], self.target[1], self.target[2],
            3, vehicle_name=self.drone
        ).join()
        self.step_count = 0
        obs = self._get_obs()
        return obs, {}

    def step(self, action):
        action = np.clip(action, -1.0, 1.0)
        pitch, roll, yaw_rate, throttle = action
        self.client.moveByRollPitchYawrateThrottleAsync(
            float(roll) * 0.3,
            float(pitch) * 0.3,
            float(yaw_rate) * 0.5,
            float(throttle) * 0.5 + 0.5,
            duration=0.1,
            vehicle_name=self.drone
        ).join()

        obs = self._get_obs()
        reward, crashed = self._compute_reward(obs["state"])
        self.step_count += 1
        terminated = crashed
        truncated = self.step_count >= self.max_steps

        return obs, reward, terminated, truncated, {}

    def render(self):
        obs = self._get_obs()
        return obs["image"]

    def close(self):
        if self.client:
            self.client.armDisarm(False, vehicle_name=self.drone)
            self.client.enableApiControl(False, vehicle_name=self.drone)
