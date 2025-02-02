# D* Path Planning Algorithm

A grid-based D* (D-Star) path planning algorithm in Python. The algorithm constructs a graph representation of the configuration space, incorporates obstacles by removing grid nodes within a specified clearance radius, and uses an A* search heuristic (via NetworkX) to compute an optimal path from a start to a goal node. Dynamic visualization with matplotlib illustrates both the exploration process and the final computed path.

## Overview

The algorithm works as follows:
- **Graph Construction:**  
  A grid graph is built with nodes corresponding to discrete positions in the environment. Obstacles are integrated by removing nodes that lie within a given radius of any obstacle coordinate.
  
- **Motion Model:**  
  An 8-connected motion model is used, allowing movement in the four cardinal directions as well as the four diagonal directions, each with appropriate cost (diagonals incur a cost of √2).

- **D* Search:**  
  A variant of the D* algorithm is implemented using NetworkX’s A* search. The heuristic function is based on Euclidean distance.

- **Dynamic Visualization:**  
  The algorithm visualizes the exploration process by displaying explored nodes in real time and finally plotting the optimal path along with performance metrics.

## Features

- **Grid-Based Graph Representation:**  
  Constructs a grid graph based on given x and y coordinates, with a configurable resolution.
  
- **Obstacle Integration:**  
  Obstacles are defined by lists of x and y coordinates. Nodes within the robot’s clearance (defined by a radius) are removed from the graph.
  
- **8-Connected Motion:**  
  The motion model supports 8 directions (up, down, left, right, and diagonals), with appropriate cost calculation.

- **D* Path Planning:**  
  Uses an A* search approach with a Euclidean distance heuristic to compute the optimal path in the modified grid.

- **Dynamic Visualization & Metrics:**  
  Real-time visualization shows the exploration and final path. Metrics such as execution time, path length, and number of steps are printed and displayed on the plot.


