# -*- coding: utf-8 -*-
"""AirSim Obstacle Avoidance Environment — Gymnasium interface for DreamerV3."""
import sys
sys.path.append('../../external-libraries')

import numpy as np
import airsim
import gymnasium as gym
from gymnasium import spaces
from PIL import Image as PILImage


class AirSimAvoidanceEnv(gym.Env):
    """无人机走廊避障任务。

    目标：在场景中向前飞行，避开建筑物和设施。
    观测：64x64 RGB图像 + 64x64 深度图 + 6维状态向量
    动作：连续4维 (pitch, roll, yaw_rate, throttle)
    奖励：前进 +1，靠近障碍 -1，碰撞 -100
    """

    metadata = {"render_modes": ["rgb_array"]}

    def __init__(self, max_steps=500, img_size=64, forward_dir=(1, 0, 0)):
        super().__init__()
        self.max_steps = max_steps
        self.img_size = img_size
        self.forward_dir = np.array(forward_dir, dtype=np.float32)
        self.step_count = 0
        self.prev_pos = None
        self.start_pos = np.array([0, 0, -8], dtype=np.float32)

        self.observation_space = spaces.Dict({
            "image": spaces.Box(0, 255, (img_size, img_size, 3), dtype=np.uint8),
            "depth": spaces.Box(0, 1, (img_size, img_size, 1), dtype=np.float32),
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
            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
            airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True),
        ], vehicle_name=self.drone)

        if responses and responses[0].width > 0:
            img = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
            img = img.reshape(responses[0].height, responses[0].width, 3)
            img = np.array(PILImage.fromarray(img).resize((self.img_size, self.img_size)))
        else:
            img = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)

        if len(responses) > 1 and responses[1].width > 0:
            depth = airsim.list_to_2d_float_array(
                responses[1].image_data_float, responses[1].width, responses[1].height
            )
            depth = np.clip(depth / 50.0, 0, 1).astype(np.float32)
            depth = depth[:, :, np.newaxis]
        else:
            depth = np.zeros((self.img_size, self.img_size, 1), dtype=np.float32)

        return {"image": img, "depth": depth, "state": state_vec}

    def _compute_reward(self, state_vec):
        pos = state_vec[:3]
        collision = self.client.simGetCollisionInfo(vehicle_name=self.drone)

        if collision.has_collided:
            return -100.0, True

        if abs(pos[2]) < 0.5 or pos[2] < -30:
            return -100.0, True

        forward_progress = 0.0
        if self.prev_pos is not None:
            delta = pos - self.prev_pos
            forward_progress = np.dot(delta, self.forward_dir)
        self.prev_pos = pos.copy()

        reward = forward_progress * 2.0

        dist_info = self.client.getDistanceSensorData(vehicle_name=self.drone)
        if hasattr(dist_info, 'distance') and dist_info.distance < 3.0:
            reward -= 1.0

        return float(reward), False

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._connect()
        self.client.reset()
        self.client.enableApiControl(True, vehicle_name=self.drone)
        self.client.armDisarm(True, vehicle_name=self.drone)
        self.client.takeoffAsync(vehicle_name=self.drone).join()
        self.client.moveToPositionAsync(
            self.start_pos[0], self.start_pos[1], self.start_pos[2],
            3, vehicle_name=self.drone
        ).join()
        self.step_count = 0
        self.prev_pos = self.start_pos.copy()
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
