# 🚀 Interactive Multi-Agent 2D RRT Path Planner

An interactive 2D simulation application built in Python using Pygame and NumPy. The system simulates multiple autonomous agents navigating a 2D environment by constructing and updating **Rapidly-exploring Random Trees (RRT)** in real-time, smoothing trajectories with splines, and dynamically avoiding obstacles.

---

## 🌟 Key Features

*   **Multi-Agent Navigation:** Supports up to 3 agents running concurrently, each with independent start/goal coordinates, search trees, and computed paths.
*   **Dynamic Obstacles:** Click and drag to draw custom polygon obstacles directly onto the canvas in real time.
*   **Adaptive Local Repair:** When a new obstacle intersects a planned path, the affected agent dynamically triggers a local RRT repair between the boundary waypoints of the collision zone. If the local repair fails, the agent falls back to a global replan.
*   **Path Smoothing:** Trajectories are post-processed using **Catmull-Rom spline interpolation**, converting jagged RRT paths into continuous, $C^1$-continuous smooth curves.
*   **Inter-Agent Avoidance:** Integrates real-time soft collision avoidance by applying a local repulsive velocity field when agents come within proximity of each other.
*   **Real-Time Animation:** Shows animated robots moving along their paths with dynamically updated heading indicators.

---

## 📂 Project Structure

```bash
├── Path-Planner_Code.py  # Interactive Pygame multi-agent path planner
├── README.md             # Project documentation (this file)
```

---

## 🛠️ Installation & Setup

Ensure you have Python 3.8+ installed. Install the required dependencies using `pip`:

```bash
pip install pygame numpy
```

---

## 🚀 How to Run

Launch the interactive RRT simulation:
```bash
python Path-Planner_Code.py
```

---

## 🎮 Interactive Simulation Controls

Use the following controls to interact with the simulation window:

| Action | Control / Keyboard Shortcut |
| :--- | :--- |
| **Draw Obstacle** | Left Click + Drag mouse (release to commit) |
| **Finalize Polygon** | `ENTER` (while drawing) |
| **Toggle Draw Mode** | `o` (Toggles between painting obstacles and placing start/goals) |
| **Select Agent** | `1`, `2`, or `3` keys (or `TAB` to cycle) |
| **Place Start Node** | Press `s`, then click anywhere on screen |
| **Place Goal Node** | Press `g`, then click anywhere on screen |
| **Start / Pause** | `SPACEBAR` (Runs all planners and begins path animation) |
| **Force Replan** | `r` (Clears trees and recalculates paths for active agents) |
| **Smooth Paths** | `p` (Triggers Catmull-Rom spline smoothing) |
| **Clear Obstacles** | `c` (Wipes all custom obstacles and resets paths) |
| **Quit Simulation** | `q` or `ESC` |

---

## 🧠 Underlying Algorithms

### Rapidly-exploring Random Tree (RRT)
RRT is a randomized search algorithm designed to efficiently search high-dimensional spaces by randomly building a space-filling tree. In this simulator:
1. The tree is biased towards the goal point using a customizable sample rate (default 5%).
2. Points are generated randomly, steer steps are calculated, and collision checks are performed against all polygon obstacles before node insertion.

### Catmull-Rom Spline Interpolation
Linear paths constructed by RRT have sharp corners that are unrealistic for real physical systems. We use a Catmull-Rom spline chain to interpolate the path. The spline guarantees first-derivative ($C^1$) continuity, yielding smooth transitions at waypoints while ensuring the trajectory passes exactly through all waypoints.
