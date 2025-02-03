# 🚀 Advanced Theta* Path Planning with Dynamic Visualization

## 🌟 Overview
The **Theta*** (Theta Star) path planning algorithm is an advanced variant of A* that integrates **line-of-sight checks** to generate **smoother, more direct** paths in a grid-based environment. Unlike A*, Theta* allows diagonal and non-grid-constrained movement, reducing unnecessary turns and improving efficiency.

🔹 **Smooth and direct paths** with minimal nodes and direction changes.  
🔹 **Optimized grid representation** for efficient obstacle handling.  
🔹 **Dynamic visualization** to track the step-by-step search process.  
🔹 **Efficient priority queue (heapq) for fast node expansion.**  

---

## 🚀 Features
### 📌 **Grid Graph Construction**
✔ **8-connected motion model** (Cardinal + Diagonal movements).  
✔ **Obstacle clearance optimization** to avoid unnecessary calculations.  
✔ **Fast graph generation** with bounding box obstacle processing.  

### 🔍 **Theta* Search Algorithm**
✔ **Priority queue (heapq) for fast node expansion.**  
✔ **Line-of-sight shortcutting** to reduce path length.  
✔ **Reduced direction changes** for more natural navigation.  

### 🎥 **Dynamic Visualization**
✔ **Step-by-step path planning animation.**  
✔ **Final path rendering with direct comparison.**  
✔ **Adjustable delay for better observation.**  

### 📊 **Performance Metrics**
✔ **Execution Time:** Total processing time for path planning.  
✔ **Path Length:** Total distance from start to goal.  
✔ **Steps Taken:** Number of nodes used in the final path.  
✔ **Direction Changes:** Measures smoothness of movement.  

---

## 📐 Mathematical Formulas
### 🔹 **Euclidean Distance (Heuristic Function)**
```math
\text{distance}(n_1, n_2) = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}
```

### 🔹 **Cost Calculation Based on Movement Type**
```math
\text{cost}(a, b) = 
\begin{cases} 
1 & \text{if moving in cardinal directions} \\
\sqrt{2} & \text{if moving diagonally}
\end{cases}
```

### 🔹 **Line-of-Sight Check**
Determines if two nodes can be directly connected without obstruction.
```math
\text{LOS}(a, b) = \begin{cases} 
\text{True} & \text{if no obstacles between } a \text{ and } b \\
\text{False} & \text{otherwise}
\end{cases}
```

### 🔹 **Path Reconstruction**
Backtracks from the goal to extract the optimal path.

---

## 🛠️ Code Structure
### 🏗 **Graph Construction and Obstacle Handling**
✔ **`build_graph()`** - Generates the grid and removes nodes near obstacles.  
✔ **`motion_mod()`** - Defines valid movement directions and their costs.  

### 🔄 **Theta* Search Algorithm**
✔ **`theta_star_search()`** - Expands nodes using a priority queue.  
✔ **`line_of_sight()`** - Checks if direct paths can be used.  
✔ **`reconstruct_path()`** - Backtracks through `came_from` to generate the optimal path.  

### 📊 **Performance Calculation**
✔ **`calculate_path_length()`** - Computes total travel distance.  
✔ **`calculate_direction_changes()`** - Measures path smoothness.  

### 🎥 **Visualization and Path Rendering**
✔ **`visualize_path()`** - Displays step-by-step execution.  
✔ **`draw_final_path()`** - Renders the computed path in red.  
✔ **`direct_connection_check()`** - Verifies if a direct route was possible.  

---

🎯 **Theta* offers smarter, optimized pathfinding with real-time visualization!** 🚀

