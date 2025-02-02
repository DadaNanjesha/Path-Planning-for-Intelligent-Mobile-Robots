# A* and A* Bidirectional  Path Planning Algorithms

This repository contains two implementations of the A* path planning algorithm for grid-based navigation with obstacles. The repository includes:

- **A\* Algorithm:** A standard A* implementation that computes the optimal path on a grid using an 8-connected motion model.
- **A\* Bidirectional Algorithm:** An enhanced version that runs A* search simultaneously from both the start and goal nodes to potentially reduce search time.

Both implementations share a similar graph structure and visualization framework.

## Overview

The algorithms are designed for a 2D grid environment with obstacles. A graph is constructed by discretizing the space into nodes over specified boundaries. Obstacles are incorporated by removing nodes that lie within a given clearance radius. The motion model supports eight directional movements (up, down, left, right, and diagonals), with diagonal moves incurring a cost of √2.

## Features

- **Graph Construction:**  
  Generates a grid graph based on defined boundaries and resolution. Nodes that fall within the robot's clearance of an obstacle are removed.

- **8-Connected Motion Model:**  
  Both algorithms use eight-directional movement for effective path exploration.

- **A\* Search:**  
  Uses a heuristic (Euclidean distance) to compute the optimal path from the start to the goal.

- **Bidirectional A\* Search:**  
  Expands the search simultaneously from the start and goal nodes, which can improve efficiency in certain scenarios.

- **Dynamic Visualization:**  
  Uses `matplotlib` to visualize the grid, obstacles, start and goal positions, and the computed path. Both implementations offer dynamic updates during the search process.

- **Performance Metrics:**  
  After executing, the algorithms report metrics such as execution time, path length, number of steps, and direction changes.

