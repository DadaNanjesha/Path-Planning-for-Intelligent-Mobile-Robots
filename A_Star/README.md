# 🚀 A* and A* Bidirectional Path Planning Algorithms

Welcome to the repository featuring two powerful implementations of the A* path planning algorithm designed for grid-based navigation with obstacles. Whether you need a classic single-direction A* or an enhanced bidirectional approach, these algorithms offer robust solutions for optimal path planning in 2D environments.

---

## 📑 Table of Contents

- [Overview](#overview)
- [Algorithm Implementations](#algorithm-implementations)
  - [A* Algorithm](#a-algorithm)
  - [A* Bidirectional Algorithm](#a-bidirectional-algorithm)
- [Features](#features)
- [Algorithm Structure and Formulas](#algorithm-structure-and-formulas)
  - [A* Algorithm Details](#a-algorithm-details)
  - [Bidirectional A* Details](#bidirectional-a-details)
- [Visualization](#visualization)
- [GIF Demonstrations](#gif-demonstrations)
- [Conclusion](#conclusion)

---

## 🔍 Overview

Both algorithms are designed for a **2D grid environment** with obstacles. The space is discretized into nodes over defined boundaries, and obstacles are incorporated by removing nodes within a given clearance radius. The environment leverages an **8-connected motion model** supporting eight possible movements (up, down, left, right, and diagonals). Diagonal moves incur a cost of √2, while horizontal and vertical moves cost 1.

---

## ⚙️ Algorithm Implementations

### A* Algorithm

- **Description:**  
  The standard A* algorithm computes the optimal path between a start node and a goal node by evaluating a cost function that combines the actual path cost and a heuristic estimate.

- **Heuristic:**  
  Uses the **Euclidean distance**:
  
  $$h(n) = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}$$
  
  where \( (x_1, y_1) \) is the current node’s position and \( (x_2, y_2) \) is the goal position.

- **Cost Model:**  
  - **Horizontal/Vertical:** Cost = 1  
  - **Diagonal:**  $Cost =\sqrt{2}$

### A* Bidirectional Algorithm

- **Description:**  
  An enhanced version that performs simultaneous searches from the start and goal nodes. The searches progress until they meet in the middle, potentially reducing the search time.

- **Key Steps:**
  1. Begin search from both the start and goal nodes.
  2. Expand nodes in parallel from each side.
  3. Once a common node is found, reconstruct the path from both directions.

- **Heuristic and Cost:**  
  Uses the same Euclidean distance heuristic and 8-connected motion model as the standard A*.

---

## ✨ Features

- **Graph Construction:**  
  - Generates a grid graph from defined boundaries and resolution.
  - Removes nodes within the robot’s clearance distance from obstacles.

- **8-Connected Motion Model:**  
  - Eight-directional movement for comprehensive exploration:
    - **Right:** `(1, 0)`
    - **Up:** `(0, 1)`
    - **Left:** `(-1, 0)`
    - **Down:** `(0, -1)`
    - **Diagonals:** `(±1, ±1)` with a cost of $\sqrt{2}$.

- **A* Search:**  
  - Computes the optimal path using a heuristic to balance the actual path cost and the estimated cost to the goal.

- **Bidirectional A* Search:**  
  - Executes two simultaneous A* searches from both endpoints to reduce the number of nodes expanded.

- **Dynamic Visualization:**  
  - Uses `matplotlib` to display:
    - The grid with obstacles.
    - Start and goal positions.
    - The computed path with real-time search progress updates.

- **Performance Metrics:**  
  Provides detailed reports including:
  - **Execution Time:** Total search time.
  - **Path Length:** Total Euclidean distance.
  - **Steps Taken:** Number of nodes (steps) in the final path.
  - **Direction Changes:** Count of changes in movement direction.

---

## 📐 Algorithm Structure and Formulas

### A* Algorithm Details

- **Lists Maintained:**
  - **Open List:** Nodes pending evaluation.
  - **Closed List:** Nodes already evaluated.

- **Cost Function:**

  $$f(n) = g(n) + h(n)$$

  where:
  - \( g(n) \) is the cost from the start node to the current node.
  - \( h(n) \) is the heuristic estimate from the current node to the goal.

### Bidirectional A* Details

- **Dual Open Lists:**
  - **Start Open List:** Nodes expanded from the start.
  - **Goal Open List:** Nodes expanded from the goal.

- **Meeting Criterion:**  
  When both open lists share a common node, the path is reconstructed by tracing back to both endpoints.

- **Heuristic Functions:**

  $$f_{\text{start}}(n) = g_{\text{start}}(n) + h_{\text{start}}(n)$$
  
  $$f_{\text{goal}}(n) = g_{\text{goal}}(n) + h_{\text{goal}}(n)$$

- **Path Reconstruction:**

   $$final path=retrace(start,meeting node)+retrace(goal,meeting node)[1:]$$


---

## 🎨 Visualization

Both implementations feature dynamic visualization using `matplotlib`:
- The grid and obstacles are clearly depicted.
- Start and goal nodes are highlighted.
- Real-time updates show explored nodes during the search.
- The final path is emphasized in red.

---

## 🎞️ GIF Demonstrations

### A* Algorithm

Watch the A* algorithm in action:

<img src="https://github.com/DadaNanjesha/Path-Planning-for-Intelligent-Mobile-Robots/blob/main/media/astar.gif" alt="A* Algorithm" width="500" height="500">

### A* Bidirectional Algorithm

See the enhanced bidirectional search:

<img src="https://github.com/DadaNanjesha/Path-Planning-for-Intelligent-Mobile-Robots/blob/main/media/astar_bidirectional.gif" alt="A* Bidirectional Algorithm" width="500" height="500">
---

## 🏁 Conclusion

This repository offers robust and efficient path planning solutions through:
- **Standard A*** for straightforward optimal path planning.
- **Bidirectional A*** for potentially faster search times by reducing the search space.

Both implementations feature an 8-connected motion model, dynamic visualizations, and detailed performance metrics, making them ideal for educational and practical applications in robotics and path planning.

---

