# Drone Swarm Exploration System

An intelligent drone swarm simulation system for autonomous exploration with human-in-the-loop target verification.

## Purpose

This program simulates multiple drones moving in a 2D environment (grid/map). Each drone can:
- Move around to explore regions
- Detect targets within a configurable radius
- Avoid collisions with other drones
- Communicate findings to a human operator
- Visualize their movement and exploration path in real-time using Tkinter GUI + Pillow (PIL)

## Features

- **Autonomous Exploration**: 20 drones systematically explore a 50×50 grid environment
- **Target Detection**: Drones detect targets within a configurable radius
- **Human Verification**: Human operator manually verifies detected targets
- **Real-time Visualization**: Live UI showing drone positions, explored regions, and targets
- **Battery Management**: Drones have limited power with low-power warnings
- **Collision Avoidance**: Drones avoid clustering using repulsive forces
- **Smart Region Assignment**: Drones are assigned to nearest unexplored regions

## Requirements

- Python 3.7+
- NumPy
- Pillow (PIL)
- tkinter (usually comes with Python)

## Installation

```bash
pip install numpy pillow
```

## Usage

Run the simulation:

```bash
python "# DRONE SWARM EXPLORATION SYSTEM.py"
```

## Configuration

Edit the `Config` class to adjust:
- Environment size
- Number of drones and targets
- Detection radius
- Battery capacity
- Exploration parameters

## How It Works

1. Drones start at random positions and explore systematically
2. When a target is detected, the drone halts and notifies the operator
3. Operator manually controls the drone to verify the target
4. Operator accepts or discards the target
5. Drone resumes autonomous exploration

## Controls

- **Arrow Buttons**: Manually control selected drone
- **Accept Target**: Confirm the detected target
- **Discard Target**: Reject false positive
- **Queue List**: Switch between drones awaiting investigation

