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
- SciPy (for Voronoi tessellation)
- tkinter (usually comes with Python)

## Installation

```bash
pip install numpy pillow scipy
```

## Usage

**Method 1: Direct Python command**
```bash
python "# DRONE SWARM EXPLORATION SYSTEM.py"
```

**Method 2: Using the run script**
```bash
./run.sh
```

**Note:** If you encounter "No module named 'scipy'" errors, make sure scipy is installed in your active Python environment:
```bash
pip install scipy
```

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
- **Algorithm selection**: Toggle between Voronoi and Greedy (`USE_VORONOI = True/False`)
- **Voronoi update interval**: How often to recompute assignments

### Exploration Algorithms

**Voronoi-based (Default):**
- Divides environment based on drone positions
- Each drone explores its "territory"
- Better load balancing and coordination
- Scales well to 100+ drones

**Greedy (Fallback):**
- Each drone picks nearest unexplored region
- Simple and fast
- Set `Config.USE_VORONOI = False` to use

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

## Troubleshooting

### "No module named 'scipy'" Error

If you see this error, install scipy in your current Python environment:

```bash
# Check which Python you're using
which python

# Install scipy
pip install scipy pillow numpy

# Or if using conda
conda install scipy pillow numpy
```

### Multiple Python Installations

If you have multiple Python versions installed, make sure to install packages in the correct environment:

```bash
# Use python3 explicitly
python3 "# DRONE SWARM EXPLORATION SYSTEM.py"

# Or check your Python path
python --version
pip --version
```

## License

MIT License
