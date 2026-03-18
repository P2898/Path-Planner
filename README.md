# Path-Planner
**Multi-Agent RRT Path Planning Simulator**

An interactive 2D simulation of multi-agent path planning using the Rapidly-exploring Random Tree (RRT) algorithm. The project demonstrates real-time path generation, dynamic obstacle handling, and smooth trajectory execution with animation.

**Features**
Multi-agent path planning (up to 3 agents)
RRT-based path generation
Real-time obstacle addition
Adaptive replanning when paths are blocked
Smooth path generation using spline interpolation
Animated agents with basic collision avoidance
Interactive UI built with Pygame

**Tech Stack**
Python
Pygame
NumPy

**How to Run**
Install dependencies:
pip install pygame numpy

Run the script:
python Path-Planner_Code.py

**Controls**
Key / Action	Function
Left Mouse Drag	Draw obstacles
o	Toggle obstacle drawing mode
1 / 2 / 3	Select agent
s + click	Set start point
g + click	Set goal point
SPACE	Start / pause simulation
r	Replan paths
p	Smooth paths
c	Clear obstacles
ENTER	Finalize polygon
ESC / q	Quit

**How It Works**
Uses RRT (Rapidly-exploring Random Trees) to generate paths from start to goal
Detects collisions with obstacles using geometric checks
Applies local repair or full replanning when paths become invalid
Smooths paths using spline interpolation for better motion
Agents follow paths with simple collision avoidance

**Use Cases**
Robotics path planning simulation
Understanding sampling-based algorithms (RRT)
Multi-agent coordination experiments
Visualization of motion planning techniques

**Limitations**
Agents are treated as point robots (no physical size)
No advanced dynamics or constraints
Collision avoidance is basic and heuristic-based


**Author**
GAYATRI PRANEETA SAMAYAMANTRI
