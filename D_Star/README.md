# 🚀 D* Path Planning Algorithm

A **grid-based D* (D-Star) path planning algorithm** implemented in Python. This algorithm builds a **graph representation** of the environment, removes obstacle-affected nodes, and finds an optimal path using **A* heuristics with NetworkX**. The visualization dynamically illustrates the search and final path.

---

## 📌 Overview
🔹 **Graph Construction:** Builds a grid graph and removes obstacle-influenced nodes.  
🔹 **Motion Model:** Uses an **8-connected** motion model (cardinal & diagonal moves).  
🔹 **D* Search:** Implements A* heuristics for optimized path computation.  
🔹 **Dynamic Visualization:** Real-time search visualization with **matplotlib**.  

---

## ✨ Features
✔ **Grid-Based Representation** – Constructs a graph based on x/y coordinates.  
✔ **Obstacle Handling** – Removes nodes within a clearance radius.  
✔ **8-Connected Motion Model** – Supports diagonal and straight-line movements.  
✔ **D* Search Algorithm** – Uses Euclidean heuristics for optimal path planning.  
✔ **Real-Time Visualization** – Displays search progress dynamically.  
✔ **Performance Metrics** – Reports execution time, path length, steps, and turns.  

---

## 📊 Algorithm Breakdown

### 🏗 **1. Graph Construction (`build_graph`)**
🔹 Creates a **grid graph** with obstacles removed.  
🔹 Ensures no paths cross obstacles by removing clearance-affected nodes.  

### 🎯 **2. Motion Model (`motion_mod`)**
✔ **Cardinal Directions:** Up, Down, Left, Right (cost = 1)  
✔ **Diagonal Movements:** Top-Left, Top-Right, Bottom-Left, Bottom-Right (cost = √2)  

### 🔍 **3. Path Planning (`d_star_planning`)**
✔ Performs **D* search** dynamically, showing the real-time search process.  
✔ Uses `d_star_search` function for A* heuristic calculations.  

### 🔄 **4. Heuristic Calculation (`calc_heuristic`)**
Computes **Euclidean distance**:
```math
h(n) = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}
```

### 📏 **5. Path Length Calculation (`calculate_path_length`)**
Total path distance:
```math
\sum_{i=1}^{n-1} \sqrt{(x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2}
```

---

## 📊 Performance Metrics
✔ **Execution Time:** Total time taken for evaluation.  
✔ **Path Length:** Total Euclidean distance of the final path.  
✔ **Steps Taken:** Number of nodes (steps) in the final path.  
✔ **Direction Changes:** Number of times the path changes direction.  

---

## 🎥 Dynamic Visualization
The **matplotlib-based visualization** dynamically updates as the search progresses:
✔ **Explored nodes** appear in real-time.  
✔ **Final path** is plotted in red.  
✔ **Obstacles & environment** remain static.  

### 📌 D* Path Planning GIF
![D* Algorithm](path_to_your_D_star_gif.gif)

---
