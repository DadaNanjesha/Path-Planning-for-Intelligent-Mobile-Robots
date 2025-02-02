# RRT* Path Planning Algorithm

Implementation of the RRT* (Rapidly-exploring Random Tree Star) algorithm for path planning in a 2D space with obstacles. The algorithm plans a path from a start point to a goal point while avoiding obstacles and optimizing the path using rewiring of nodes.

## Overview

RRT* is an extension of the standard RRT algorithm that not only finds a feasible path but also continuously optimizes the path cost by rewiring the tree. This implementation includes:

- **Random Sampling with Goal Bias:** The algorithm samples random points in the configuration space with a probability of selecting the goal.
- **Collision Checking:** It uses both endpoint checks and line-segment distance calculations to detect potential collisions with obstacles.
- **Rewiring:** As new nodes are added, nearby nodes are rewired to ensure a lower cost path.
- **Visualization:** Dynamic visualization is provided using `matplotlib` to display obstacles, the growing tree, and the final path.
- **Metrics:** Execution time, path length, number of steps, and direction changes are calculated and displayed.

## Features

- **Obstacle Representation:**  
  Obstacles are defined as a set of coordinate points with a clearance defined by the robot's radius.
  
- **Dynamic Visualization:**  
  The algorithm visually displays the search process in real time. Obstacles are shown in black, the start and goal points in blue and green respectively, and the final path in red.

- **Performance Metrics:**  
  Upon completion, the algorithm prints and displays metrics including:
  - Execution time
  - Total path length
  - Number of steps
  - Number of direction changes

