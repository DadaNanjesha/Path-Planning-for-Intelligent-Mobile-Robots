# Advanced Path Planning Algorithms for Intelligent Mobile Robots

This project is a collection of advanced path planning algorithms for grid-based navigation and autonomous systems. Each algorithm is implemented in its own folder with a dedicated README file explaining its details, usage, and performance metrics. This top-level README provides an overview of the entire project, its structure, installation instructions, usage guidelines, and licensing information.

## Project Overview

The project contains several advanced path planning techniques, including:

- **A\* Algorithm:**  
  A classic grid-based path planning method that computes the optimal path using an 8-connected graph and a Euclidean distance heuristic.

- **Bidirectional A\* Algorithm:**  
  An enhanced version of A\* that simultaneously searches from the start and goal, potentially reducing the search time.

- **D\* Algorithm:**  
  A dynamic path planning algorithm that efficiently updates the optimal path in response to changes in the environment.

- **Theta\* Algorithm:**  
  An any-angle variant of A\* that uses line-of-sight checks to "shortcut" unnecessary nodes, producing smoother and more direct paths with fewer turns. This implementation includes dynamic visualization with step-by-step plotting and draws a continuous red line for the final path.

- **Advanced AI/ML (DQN-Based) Path Planning:**  
  An implementation that leverages deep reinforcement learning (using a Deep Q-Network) on the same grid structure to learn an optimal navigation policy through trial and error.

## Project Structure






## Requirements

- Python 3.x
- [networkx](https://networkx.org/)
- [matplotlib](https://matplotlib.org/)

Install the required libraries via pip:

```bash
pip install networkx matplotlib
