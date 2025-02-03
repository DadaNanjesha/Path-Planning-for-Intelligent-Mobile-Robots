# 🚀 RRT* Path Planning Algorithm

## 📌 Overview
The **RRT* (Rapidly-exploring Random Tree Star)** algorithm is an optimized version of **RRT (Rapidly-exploring Random Tree)** for **path planning** in 2D environments with obstacles. It not only finds a feasible path but also **optimizes** it by continuously **rewiring** nodes.

### ✨ Features
- **Random Sampling with Goal Bias:** Ensures efficient exploration while increasing the likelihood of reaching the goal.
- **Collision Checking:** Uses both endpoint checks and segment distance calculations to avoid obstacles.
- **Rewiring:** Improves path quality by reconnecting nearby nodes to minimize costs.
- **Dynamic Visualization:** Displays tree growth, obstacles, and final path using `matplotlib`.
- **Performance Metrics:** Measures execution time, total path length, number of steps, and direction changes.
- **NetworkX Graph Representation:** The tree structure is stored using **NetworkX**, allowing efficient graph operations.

---

## 📂 Algorithm Workflow
1️⃣ **Random Sampling with Goal Bias** - Samples nodes, with increased probability of choosing the goal.  
2️⃣ **Nearest Node Selection** - Finds the closest existing node to the sampled node.  
3️⃣ **Steering Towards Sampled Node** - Moves in the direction of the sample with a limited step size.  
4️⃣ **Collision Checking** - Ensures the path does not pass through obstacles.  
5️⃣ **Rewiring** - Attempts to reconnect nodes to optimize path cost.  
6️⃣ **Goal Connection** - Connects the goal to the tree when within range.  
7️⃣ **Path Extraction** - Backtracks from the goal to extract the optimal path.  

---

## 📊 Performance Metrics
✔ **Execution Time** - Measures total algorithm runtime.  
✔ **Path Length** - Calculates total Euclidean distance.  
✔ **Steps Taken** - Counts total number of path nodes.  
✔ **Direction Changes** - Tracks how often the path changes between cardinal and diagonal directions.  
✔ **Graph Metrics** - Tracks the **number of nodes and edges** in the **NetworkX graph**.

---

## 📌 Mathematical Formulas
### 🔹 **Euclidean Distance (Cost Calculation)**
```math
\text{distance}(n_1, n_2) = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}
```

### 🔹 **Collision Checking (Point-to-Segment Distance)**
```math
\text{distance}(p, \text{segment}) = \min_{t \in [0,1]} \| p - (a + t \cdot (b - a)) \|
```

### 🔹 **Path Cost Update (Rewiring Nodes)**
```math
\text{new\_cost}(n_1, n_2) = \text{cost}(n_1) + \text{distance}(n_1, n_2)
```

---

## 🖥️ NetworkX Graph Representation
The **NetworkX** library is used in the RRT* algorithm to efficiently store and manage the **tree structure** during path planning. Each node in the graph represents a **point in the search space**, while edges define **valid connections between nodes**.

### 🔹 **How the Graph is Created**
- **Nodes**: Every newly generated valid point in the space is added to the **NetworkX graph** as a **node**.
- **Edges**: When a new node is connected to the nearest or rewired node, an **edge** is created between them, storing the movement cost.
- **Graph Growth**: The graph starts with only the **start node** and expands iteratively by adding new nodes and edges.

### 🔹 **Number of Nodes & Edges**
- The number of **nodes** in the graph depends on `max_iter` and successful additions to the tree.
- The number of **edges** is slightly less than the number of nodes since each node connects to at least one other node but may be rewired to minimize cost.
- If **goal connection** is successful, the graph contains the **shortest optimized path** connecting the **start** to the **goal**.

This **graph-based representation** enables efficient **pathfinding, rewiring, and visualization**, making it a crucial part of the **RRT*** algorithm.

---

## 🛠️ Code Structure
### 🏗 **Class: RRTStar**
- **Initialization** - Defines environment boundaries, obstacles, and visualization. Initializes the **NetworkX graph**.
- **`planning()`** - Runs the main algorithm loop, adding nodes and checking feasibility.
- **`connect_nodes()`** - Connects and rewires new nodes efficiently.
- **Graph Representation** - Uses **NetworkX** to track nodes and edges dynamically.
- **Visualization** - Provides step-by-step animation of tree growth.
- **Performance Tracking** - Calculates execution time, path length, graph statistics, and direction changes.

---

🎯 **Efficient and optimized pathfinding with real-time visualization and NetworkX graph analysis!** 🚀

