# ROS Turtlesim Sequential Exploration & TSP Navigation

A multi-robot exploration and path-planning system built in ROS (Robot Operating System) using Turtlesim. In this project, three robots sequentially explore an environment using a Lawnmower search algorithm to find randomly generated targets. Once the search is complete, the system elects a "Leader" based on the shortest travel distance, which then navigates to all discovered targets using a Traveling Salesperson Problem (TSP) algorithm.

## 🚀 Key Features

* **Automated Safe-Distance Target Generation:** A custom generator node spawns 6 random targets while calculating the Euclidean distance (`math.sqrt`) to ensure targets never overlap (minimum safe distance constraint).
* **Sequential Lawnmower Search:** Robots sweep the map vertically and horizontally. Edge detection is handled dynamically using absolute coordinate differences (`abs()`) rather than hardcoded map boundaries.
* **Inter-Node Communication:** Robots publish their status, found targets, and coordinates to a `/robot_info` topic using `latch=True`. This ensures sleeping nodes don't miss messages and wake up exactly when it's their turn.
* **Leader Election:** The system automatically compares total distances traveled by each robot. The most efficient robot is elected as the Leader for the final task.
* **Nearest Neighbor TSP Algorithm:** The Leader calculates the shortest path to visit all discovered targets using the Nearest Neighbor pathfinding logic.
* **Smooth Kinematic Control:** Implemented Proportional Control (PID) for linear and angular velocities. Includes a custom Pi (π) normalization block to prevent the 360-degree continuous spinning bug ("pirouette effect") during coordinate tracking.

## 📁 System Architecture

* `midterm.launch`: The main launch file. It starts the `turtlesim_node`, runs the `target_generator.py`, and automatically spawns Turtle 2 and Turtle 3 at their precise starting locations using `rosservice call /spawn` arguments.
* `target_generator.py`: Generates non-overlapping targets and draws them on the simulator.
* `explorer.py`: The core brain of the robots. Handles searching, topic communication, distance calculation, leader election, and TSP navigation.

## ⚙️ Prerequisites and Installation

This project requires Ubuntu and ROS (Melodic/Noetic).

1. Clone the repository into your catkin workspace's `src` folder:
```bash
cd ~/catkin_ws/src
git clone https://github.com/mozankaya/ROS-Turtlesim-Exploration.git beginner_tutorials
```

2. Make sure the Python scripts are executable:
```bash
cd ~/catkin_ws/src/beginner_tutorials/src
chmod +x explorer.py target_generator.py
```

3. Build your workspace:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 🏃‍♂️ How to Run

1. **Start the simulation and target generator:**
```bash
roslaunch beginner_tutorials midterm.launch
```
Wait a few seconds for the targets to be generated.

2. **Start the Explorer Robots:**
Open 3 new terminal windows (don't forget to `source devel/setup.bash` in each) and start the robots in reverse order so they wait for their turn:

Terminal 1:
```bash
rosrun beginner_tutorials explorer.py 3
```
Terminal 2:
```bash
rosrun beginner_tutorials explorer.py 2
```
Terminal 3:
```bash
rosrun beginner_tutorials explorer.py 1
```

Once Turtle 1 starts, the automated sequential exploration will begin!
