# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Multi-agent drone collaboration system using AirSim. Part of a course on "Drones + LLMs" (Chapter 7). The course teaches two coordination patterns (centralized & distributed) with pure Python first, then introduces CrewAI framework.

## Prerequisites & Setup

```bash
pip install openai airsim   # 基础部分（笔记本1-4）
pip install crewai           # 框架部分（笔记本5-6）
```

AirSim simulator must be running before executing notebooks. Copy `settings.json` to `~/Documents/AirSim/` to configure 2 drones (Drone1, Drone2).

The AirSim Python SDK is loaded from `../external-libraries/` via `sys.path.append`.

## Running

Notebooks are meant to be run sequentially:
1. `1-multi_agent_intro.ipynb` — 理论：中心化 vs 分布式协同模式
2. `2-agent_env_build.ipynb` — 环境搭建 + airsim_tools.py 工具模块
3. `3-centralized_demo.ipynb` — 中心化协同（纯Python，LLM指挥官）
4. `4-distributed_demo.ipynb` — 分布式协同（纯Python，消息板+独立决策）
5. `5-crewai_intro.ipynb` — CrewAI框架入门（Agent/Task/Crew）
6. `6-crewai_airsim.ipynb` — CrewAI + AirSim 多无人机协同

## Architecture

### Pure Python approach (Notebooks 3-4)
- `airsim_tools.py` — Pure functions: `takeoff()`, `fly_to()`, `get_state()`, `call_llm()`
- No agent framework dependency, uses OpenAI SDK directly for LLM calls
- Centralized: one LLM commander decomposes tasks → for-loop execution
- Distributed: shared `message_board` list → each drone has its own `drone_brain()` with independent LLM

### CrewAI approach (Notebooks 5-6)
- `@tool` decorator wraps AirSim functions as Agent-callable tools
- MissionPlanner Agent (plans) + Scout Agent (executes with tools)
- `Crew(agents, tasks).kickoff()` orchestrates the workflow

### LLM Configuration
Uses Doubao (Volcengine) model via OpenAI-compatible API. Low temperature (0.1) for deterministic task decomposition.

```python
# Pure Python
from openai import OpenAI
client = OpenAI(base_url="https://ark.cn-beijing.volces.com/api/v3", api_key="...")

# CrewAI
from crewai import LLM
llm = LLM(model="doubao-seed-2-0-pro-260215", base_url="...", api_key="...")
```

## AirSim Image Capture (Claude Code 视觉能力)

AirSim 的图像获取是**通过 API 直接从 UE4 渲染引擎内部拿数据**，不是截屏。完全不需要截屏权限。

```python
import airsim

client = airsim.MultirotorClient()

# 从 UE4 渲染管线直接读取，不是操作系统截图
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),       # RGB画面
    airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True),         # 深度图
    airsim.ImageRequest("0", airsim.ImageType.Segmentation, False, False), # 语义分割
], vehicle_name="Drone1")

# 保存为 PNG，然后 Claude Code 用 Read 工具查看
with open("drone_view.png", "wb") as f:
    f.write(responses[0].image_data_uint8)
```

关键点：
- **不需要桌面/截图权限**，窗口最小化也能拿到图像
- 能获取 RGB、深度图、语义分割等截屏拿不到的数据
- Claude Code 可以通过 Read 工具直接查看 PNG 图片（多模态能力）
- 完整闭环：**拍照 → 看图 → 判断 → 操作无人机 → 再拍照...**

### Windows 部署方案

```
Windows PC 上同时跑：
├── AirSim + UE4（仿真环境，自动监听 localhost:41451）
├── Claude Code（CLI / VS Code扩展 / 桌面版）
└── Python + airsim 包

airsim.MultirotorClient() 默认连 localhost:41451，开箱即用。
```

## Key Design Decisions

- **Pure Python first, framework second**: Students learn coordination principles with ~50 lines of code before using CrewAI abstractions.
- **NED coordinate system**: AirSim uses North-East-Down; negative z values represent altitude above ground (z=-10 means 10m high).
- **Fallback mechanism**: In CrewAI notebooks, if LLM fails to generate tool calls, tasks still complete via framework retry.
