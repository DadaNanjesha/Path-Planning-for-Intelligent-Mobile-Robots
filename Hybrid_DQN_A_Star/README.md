# Hybrid DQN-A Path Planner

Hybrid DQN-A Path Planner is a hybrid path planning algorithm designed for grid-based navigation in autonomous systems. It combines a Deep Q-Network (DQN) for learning an optimal navigation policy with a fallback A* search to guarantee that the computed path reaches the goal. This approach leverages the strengths of reinforcement learning while ensuring robustness through traditional search methods.

## Overview

This project implements a hybrid approach to path planning that:
- **Builds a grid-based graph** of the environment where nodes represent discrete positions and obstacles are removed based on a specified clearance.
- **Uses a Deep Q-Network (DQN)** to learn an optimal policy for navigating the grid.
- **Falls back to A* search** if the learned policy does not reach the goal, ensuring that a valid path is always computed.
- **Provides dynamic visualization** of the planning process with a pause on each plotted node and overlays the final path as a continuous red line.
- **Computes key metrics** such as execution time, total path length, steps taken, and direction changes.

## Features

- **Grid Graph Construction:**  
  Constructs a complete grid from specified boundaries and removes nodes that are too close to obstacles using an efficient bounding box approach.

- **8-Connected Motion Model:**  
  Connects nodes based on 8 possible movements (up, down, left, right, and the four diagonals) with appropriate movement costs.

- **Deep Q-Network (DQN):**  
  Trains a feedforward neural network to approximate the Q-values for each action given the state, enabling the agent to learn an optimal navigation policy.

- **Fallback A* Search:**  
  In cases where the DQN-based policy does not reach the goal, the algorithm uses A* search on the same grid to connect the final state to the goal.

- **Dynamic Visualization:**  
  Displays the grid, obstacles, start and goal positions, and dynamically plots each node in the computed path with a pause (0.5 seconds per point) followed by a continuous red line showing the final path.

- **Performance Metrics:**  
  Reports:
  - **Execution Time:** Total time taken for evaluation.
  - **Path Length:** Total Euclidean distance of the final path.
  - **Steps Taken:** Number of nodes (steps) in the final path.
  - **Direction Changes:** Number of times the path changes direction.
