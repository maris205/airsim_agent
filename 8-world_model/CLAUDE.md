# CLAUDE.md

## Project Overview

World Model + AirSim drone course (Chapter 8). Teaches DreamerV3 world model for autonomous drone control.

## Structure

- Notebooks 1-2: Theory (no AirSim needed)
- Notebooks 3-4: Inference with pretrained weights (AirSim needed)
- Notebooks 5-6: Full training from scratch (GPU needed)

## Prerequisites

```bash
conda activate drone
pip install torch gymnasium
```

AirSim must be running for notebooks 3-6. Copy `settings.json` to `~/Documents/AirSim/`.

Camera is 64x64 (small for DreamerV3 input). Single drone (Drone1).

## Key Files

- `world_model_tools.py` — Shared utilities (AirSim wrappers, model helpers)
- `envs/airsim_hover_env.py` — Gymnasium env for hover task
- `envs/airsim_avoidance_env.py` — Gymnasium env for obstacle avoidance
- `models/` — Pretrained checkpoints (download separately)

## AirSim SDK

Loaded from `../external-libraries/` via `sys.path.append`.

## LLM

Not used in this chapter. This chapter is pure RL / world model.
