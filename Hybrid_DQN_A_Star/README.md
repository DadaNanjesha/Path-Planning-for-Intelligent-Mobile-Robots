# 🚀 Hybrid DQN-A* Path Planner

The **Hybrid DQN-A* Path Planner** is an innovative path planning algorithm that combines **Deep Q-Network (DQN)** for reinforcement learning-based navigation with **A* search** as a fallback to guarantee successful pathfinding. This ensures the system benefits from both adaptive learning and reliable deterministic search.

---

## 📌 Overview
🔹 **Grid-Based Graph Construction** – Environment is discretized into a grid, with obstacles removed based on clearance constraints.  
🔹 **Deep Q-Network (DQN) Learning** – Learns an optimal navigation policy using reinforcement learning.  
🔹 **A* Search Fallback** – Ensures a path is always computed if DQN fails.  
🔹 **Dynamic Visualization** – Real-time plotting of the search process using `matplotlib`.  
🔹 **Performance Metrics** – Reports execution time, path length, steps, and direction changes.  

---

## ✨ Features

### 🏗 **1. Grid Graph Construction**
✔ Builds a structured grid for motion planning.  
✔ **Obstacle Handling** – Nodes within a defined clearance radius are removed.  

---

### 🔄 **2. 8-Connected Motion Model**
✔ Supports **8 movement directions**: 
   - **Cardinal Directions** (Up, Down, Left, Right) → Cost = 1
   - **Diagonal Movements** (Top-Left, Top-Right, Bottom-Left, Bottom-Right) → Cost = √2

Cost function for movement:


$$
\text{Cost}(dx, dy) = 
\begin{cases} 
1 & \text{if } dx \text{ or } dy = 0 \text{ (cardinal)} \\
\sqrt{2} & \text{if } dx \neq 0 \text{ and } dy \neq 0 \text{ (diagonal)}
\end{cases}
$$



---

### 🤖 **3. Deep Q-Network (DQN) Training**
✔ Uses a **feedforward neural network** to approximate Q-values.  
✔ Learns the best action policy using the **epsilon-greedy strategy**.  
✔ Trained using the **Bellman equation**:
```math
Q(s_t, a_t) \leftarrow Q(s_t, a_t) + \alpha \left[ r_t + \gamma \max_a Q(s_{t+1}, a) - Q(s_t, a_t) \right]
```
where:

  - ${\alpha}$ is the learning rate,
  - ${\gamma}$ is the discount factor,
  - ${r_t}$  is the reward,
  - $s_{t+1}$   is the next state.

---

### ⭐ **4. A* Search Fallback**
✔ If DQN **fails to reach the goal**, A* is used to complete the path.  
✔ Uses the **Euclidean distance heuristic**:
```math
h(n_1, n_2) = \sqrt{(x_2 - x_1)^2 + (y_2 - y_1)^2}
```
where \( (x_1, y_1) \) and \( (x_2, y_2) \) are the node coordinates.

---

### 🎥 **5. Dynamic Visualization**
✔ Real-time plotting of search progress using `matplotlib`.  
✔ **0.5-second pause** between points to visualize movement.

---

### 📊 **6. Performance Metrics**
✔ **Execution Time** – Total time taken for path computation.  
✔ **Path Length** – Total Euclidean distance from start to goal.  
✔ **Steps Taken** – Number of nodes in the computed path.  
✔ **Direction Changes** – Number of times the path switches between movement types.

---

## 🏋️‍♂️ Training & Evaluation

### 🏁 **1. Training the DQN Agent**
✔ **State Representation**:
```math
\text{state} = [\text{current}_x, \text{current}_y, \text{goal}_x, \text{goal}_y]
```
✔ **Action Space**: 8 possible movement actions.  
✔ **Reward Structure**:
   - Positive reward for reaching goal.
   - Negative penalty for collisions.

---

### 📊 **2. Evaluation**
✔ **Policy Execution** – Checks if the trained DQN agent reaches the goal.  
✔ **Fallback Mechanism** – A* search is triggered when necessary.  

---

🎉 **Happy Path Planning! 🚀**

