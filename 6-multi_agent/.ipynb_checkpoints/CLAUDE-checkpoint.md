# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Multi-agent drone collaboration system using LangGraph and AirSim. Part of a course on "Drones + LLMs" (Chapter 6 of 7). Multiple LLM-powered agents coordinate drone swarms for search/reconnaissance tasks in the AirSim simulator.

## Prerequisites & Setup

```bash
pip install langchain langchain_core langchain_openai langgraph airsim numpy
```

AirSim simulator must be running before executing notebooks. Copy `settings.json` to `~/Documents/AirSim/` to configure 2 drones (Drone1, Drone2).

The AirSim Python SDK is loaded from `../external-libraries/` via `sys.path.append`.

## Running

Notebooks are meant to be run sequentially:
1. `1-multi_agent_intro.ipynb` — Multi-agent concepts overview
2. `2-agent_env_build.ipynb` — Environment setup & drone validation
3. `3-langgraph_brief.ipynb` — LangGraph framework tutorial
4. `4-multi_agent_langgraph.ipynb` — Full multi-agent drone system

## Architecture

### Agent Roles (3-agent hierarchy)
- **MissionPlanner (Supervisor)** — Decomposes high-level goals into sub-tasks via LLM
- **ScoutAgent (Worker)** — Executes drone flight tasks using AirSim tools
- **AnalystAgent (Expert)** — Analyzes ambiguous findings

### LangGraph State Machine
The core pattern in `4-multi_agent_langgraph.ipynb`:
- `GraphState(TypedDict)` with `mission_goal`, `task_queue`, `completed_tasks`, `messages`
- `task_queue` uses plain `List[Dict]` (full replacement on each update)
- `completed_tasks` and `messages` use `Annotated[..., operator.add]` (accumulate)
- Nodes: `supervisor_node` → `scout_node` → `tool_executor_node` with conditional `router`

### AirSim Tools (`airsim_agent.py`)
Three `@tool`-decorated functions wrapping AirSim API via a global `MultirotorClient`:
- `takeoff(drone_id)` — Arm and lift off
- `fly_to_position(drone_id, x, y, z)` — Move to NED coordinates (negative z = altitude)
- `get_drone_state(drone_id)` — Get position and orientation

### LLM Configuration
Uses Doubao (fire volcano) model via OpenAI-compatible API. Low temperature (0.01–0.1) for deterministic task decomposition.

## Key Design Decisions

- **LangGraph over AutoGen/CrewAI**: Chosen for deterministic control flow and explicit routing, better suited for production drone coordination than conversation-driven alternatives.
- **ReAct pattern**: Each agent node reasons (LLM decides which tools) then acts (tool functions execute on AirSim).
- **Fallback mechanism**: If LLM fails to generate tool calls, manual execution simulates the tools to prevent workflow blocking.
- **NED coordinate system**: AirSim uses North-East-Down; negative z values represent altitude above ground.
