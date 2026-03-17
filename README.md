# 🚁 Drone Swarm Simulation

A multi-drone search and rescue simulation built in Python. Drones autonomously explore a disaster zone using a **Boustrophedon (lawnmower) sweep algorithm**, detect targets, and report to a command center UI where an operator can manually investigate and confirm findings.

---

## 📸 Overview

```
┌─────────────────────────────┐        ┌──────────────────────────────┐
│  Simulation Thread          │        │  Command Center (Tkinter UI) │
│  • 20 drones exploring      │◄──────►│  • Live map view             │
│  • Boustrophedon algorithm  │ Queues │  • Investigation queue       │
│  • Target detection         │        │  • Manual drone control      │
│  • Collision avoidance      │        │  • Accept / Discard targets  │
└─────────────────────────────┘        └──────────────────────────────┘
```

---

## ✨ Features

- **Boustrophedon sweep algorithm** — drones follow a systematic lawnmower path for guaranteed, non-redundant area coverage
- **Zone partitioning** — the grid is divided into equal slices, one per drone, so no two drones overlap
- **Collision avoidance** — repulsive potential field keeps drones from crashing into each other
- **Orphan region reassignment** — if a drone dies mid-sweep, its uncovered regions are redistributed to active drones
- **IPC via thread-safe queues** — simulation and UI run independently and communicate without shared state
- **Manual drone control** — operator can take over any halted drone using arrow keys
- **Target confirmation flow** — operator accepts or discards each detected target (handles false positives)
- **Power management** — drones consume power each iteration, with low-power warnings and dead-drone handling
- **Dark-themed live map** — color-coded drone states rendered as a PNG each tick

---

## 🖥️ Requirements

- Python 3.8+
- Libraries:

```bash
pip install pillow numpy scipy
```

> `tkinter` is included with Python by default. On Linux if missing: `sudo apt-get install python3-tk`

---

## 🚀 How to Run

```bash
python drone_swarm.py
```

On Mac, use `python3` instead of `python`.

---

## 🎮 Controls

| Action | How |
|---|---|
| Move manual drone | Arrow keys **↑ ↓ ← →** or the buttons in the UI |
| Accept a target report | Click **✔ ACCEPT** |
| Discard a false positive | Click **✘ DISCARD** |
| Select a drone from queue | Click it in the Investigation Queue list |

---

## 🗺️ Map Legend

| Color | Meaning |
|---|---|
| 🔵 Blue | Actively exploring |
| 🟠 Orange | Halted — found a target, waiting for operator |
| 🟣 Purple | Under manual operator control |
| 🔴 Red | Low power warning |
| ⚫ Dark gray | Dead — out of power |
| 🔴 Red circle (map) | Target location |
| 🟢 Green ring (map) | Confirmed target |

---

## ⚙️ Configuration

All settings are in the `Config` class at the top of `drone_swarm.py`:

| Parameter | Default | Description |
|---|---|---|
| `ENV_SIZE` | 60 | Grid size (60×60 cells) |
| `NUM_TARGETS` | 4 | Number of targets placed randomly |
| `NUM_DRONES` | 20 | Number of drones in the swarm |
| `INITIAL_POWER` | 1500 | Starting power per drone |
| `DETECTION_RADIUS` | 2.5 | Target detection radius (in cells) |
| `REGION_SIZE` | 5 | Sub-region size for sweep assignment |
| `DRONE_SPEED` | 0.6 | Autonomous movement speed |
| `MANUAL_SPEED` | 1.2 | Manual control movement speed |

---

## 🧠 How the Algorithm Works

### 1. Zone Partitioning
The full grid is divided into 5×5 sub-regions. These regions are sorted and split into equal vertical slices — one slice per active drone. No two drones are assigned the same region.

### 2. Boustrophedon Path
Within each slice, the drone visits regions in a lawnmower pattern:
- Column 1: top → bottom
- Column 2: bottom → top
- Column 3: top → bottom … and so on

This guarantees complete coverage with zero backtracking or re-visiting.

### 3. Target Detection Flow
```
Drone detects target within radius
  → Drone halts, sends report to UI queue
    → Operator selects drone in Investigation Queue
      → Operator flies drone manually to inspect
        → Operator clicks ACCEPT or DISCARD
          → Drone is released back to its sweep path
```

---

## 📁 Project Structure

```
drone_swarm.py     # Main file — simulation, algorithm, and UI all in one
drone_simulation.log  # Auto-generated log file (created on first run)
```

---

## 📋 System Design

The project is split into two logical processes running in parallel:

**Simulation thread** (`DroneSimulation.run`)
- Runs the main loop every 80ms
- Detects targets using `Environment.getinfo(x, y, radius)`
- Reads commands from the `to_simulation` queue
- Updates all drone positions using the boustrophedon algorithm
- Renders a PNG of the current state and sends it to `to_ui`

**Command center (main thread)** (`CommandCenter`)
- Tkinter UI polling every 50ms
- Displays the live map, investigation queue, and target reports
- Sends manual control commands and accept/discard decisions back to the simulation via `to_simulation`

---

## 📄 License

This project was built as part of a robotics software course assignment.
