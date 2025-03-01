# 🚀 Advanced Path Planning Algorithms for Intelligent Mobile Robots

## 📌 Overview
This repository contains implementations of various **advanced path-planning algorithms** used in autonomous mobile robots. These algorithms are widely applied in **robotics, AI, and autonomous navigation systems**. 

The repository includes implementations of:
- **A\***, **D\***, **Hybrid DQN-A\***, **RRT\***, and **Theta\*** algorithms.
- Efficient solutions for **navigation in dynamic and static environments**.

---

## ✨ Features
✔ **A\* Algorithm** - Implements the classical A\* search and bidirectional variant for optimal path planning.  
✔ **D\* Algorithm** - A dynamic path planning algorithm that adapts to changing environments.  
✔ **Hybrid DQN-A\*** - A hybrid approach combining Deep Q-Network (DQN) with A\* for intelligent decision-making.  
✔ **RRT\* Algorithm** - Rapidly-exploring Random Tree (RRT\*) for sampling-based path planning.  
✔ **Theta\* Algorithm** - An optimized variant of A\* allowing any-angle path planning.  

---

## 📂 Directory Structure
```plaintext
📦 Advanced_Path_Planning
├── 📂 A_Star
│   ├── 📄 README.md
│   ├── 📄 a_star_algorithm.py
│   ├── 📄 a_star_bidirectional_algorithm.py
│
├── 📂 D_Star
│   ├── 📄 README.md
│   ├── 📄 d_star.py
│
├── 📂 Hybrid_DQN_A_Star
│   ├── 📄 README.md
│   ├── 📄 hybrid_dqn_a_star_algorithm.py
│
├── 📂 RRT_Star
│   ├── 📄 README.md
│   ├── 📄 rrt_star_algorithm.py
│
├── 📂 Theta_Star
│   ├── 📄 README.md
│   ├── 📄 theta_star_algorithm.py
│
├── 📂 media
│   ├── 📹 astar.gif
│   ├── 📹 astar_bidirectional.gif
│   ├── 📹 dstar.gif
│   ├── 📹 dqn_astar.gif
│   ├── 📹 rrtstar.gif
│   ├── 📹 thetastar.gif
│
├── 📂 test
│   ├── 📄 test_astar.py
│   ├── 📄 test_dstar.py
│   ├── 📄 test_hybrid_dqn_a_star.py
│   ├── 📄 test_rrt_star.py
│   ├── 📄 test_theta_star.py
│
├── 📄 .gitignore
├── 📄 LICENSE
├── 📄 README.md
├── 📄 requirements.txt
├── 📄 setup.py
├── 📂 .github

```

---

## ⚙️ Installation
To run the algorithms, install the necessary dependencies:
```sh
pip install -r requirements.txt
pip install -e .

```

---

## 🚀 Usage
Each algorithm can be executed independently. Example usage:
```sh
python A_Star/a_star_algorithm.py
            or
pytest -v --verbose
```
Modify the scripts as needed to test different environments or configurations.

---

## 📊 Project Overview
The project contains several advanced path planning techniques, including:
### 🔹 **A\* Algorithm**
A classic grid-based path planning method that computes the optimal path using an 8-connected graph and a Euclidean distance heuristic.  
<img src="media/astar.gif" width="300" height="300" alt="A* Algorithm">

### 🔹 **Bidirectional A\***
An enhanced version of A\* that simultaneously searches from the start and goal, potentially reducing the search time.  
<img src="media/astar_bidirectional.gif" width="300" height="300" alt="Bidirectional A* Algorithm">

### 🔹 **D\* Algorithm**
A dynamic path planning algorithm that efficiently updates the optimal path in response to changes in the environment.  
<img src="media/dstar.gif" width="300" height="300" alt="D* Algorithm">

### 🔹 **RRT\* Algorithm**
A rapidly exploring random tree (RRT) is an algorithm designed to efficiently search nonconvex, high-dimensional spaces by randomly building a space-filling tree.  
<img src="media/rrtstar.gif" width="300" height="300" alt="D* Algorithm">

### 🔹 **Theta\* Algorithm**
An any-angle variant of A\* that uses line-of-sight checks to "shortcut" unnecessary nodes, producing smoother and more direct paths.  
<img src="media/thetastar.gif" width="300" height="300" alt="Theta* Algorithm">

### 🔹 **Advanced AI/ML (DQN-Based) Path Planning**
An implementation leveraging deep reinforcement learning (DQN) to learn an optimal navigation policy.  
<img src="media/dqn_astar.gif" width="300" height="300" alt="DQN-A* Algorithm">

---

## 🎥 Visualization
Pathfinding results can be visualized using the generated `.gif` files in the `media` directory.

---

# **📊 Performance Metrics Overview**

This table provides a performance comparison of various path-planning algorithms implemented in this project.


| **Algorithm**         | **Execution Time (s)** | **Path Length** | **Steps Taken** | **Direction Changes** | **Optimality**       | **Computational Efficiency** | **Scalability** | **Adaptability to Dynamic Environments** | **Best Use Cases**                                                                 |
|------------------------|------------------------|-----------------|-----------------|-----------------------|----------------------|------------------------------|-----------------|------------------------------------------|-----------------------------------------------------------------------------------|
| **A***                | 20.45 - 30.48 🟡       | 85.05 🟢        | 67 🔴            | 13 🟡                 | High 🟢              | Low 🔴                        | Low 🔴          | Low 🔴                                   | Structured static environments (e.g., warehouse automation)                       |
| **Bidirectional A***  | 4.20 - 5.46 🟢         | 96.77 🟡        | 87 🔴            | 7 🟢                  | Moderate 🟡          | High 🟢                       | Moderate 🟡     | High 🟢                                  | Dynamic environments with known obstacles                                         |
| **D***                | 20.50 - 22.11 🟡       | 85.05 🟢        | 67 🔴            | 13 🟡                 | High 🟢              | Low 🔴                        | Low 🔴          | Moderate 🟡                              | Real-time replanning in semi-dynamic environments                                 |
| **RRT***              | 8.03 - 39.19 🟡        | 104.69 - 113.35 🔴 | 22 - 24 🟢      | 21 - 22 🔴            | Low - Moderate 🟡    | Moderate 🟡                   | High 🟢         | High 🟢                                  | Exploration & high-dimensional navigation                                         |
| **Theta***            | 0.08 - 0.12 🟢         | 82.27 🟢        | 6 🟢             | 4 🟢                  | Very High 🟢         | Very High 🟢                  | High 🟢         | Low 🔴                                   | Precise trajectory planning & smooth motion                                       |
| **DQN-A***            | 0.01 🟢                | 85.05 🟢        | 67 🔴            | 13 🟡                 | High 🟢              | Very High 🟢                  | Very High 🟢    | Very High 🟢                             | AI-enhanced real-time obstacle avoidance                                          |

---

This comparison helps in selecting the appropriate algorithm based on the application requirements, balancing optimality, efficiency, and adaptability.

## 🎨 **Legend**

- 🟢 **Green**: Best performance in a category.
- 🟡 **Yellow**: Moderate performance.
- 🔴 **Red**: Lower performance or limitations.


## 🎯 **Key Insights**

- ⚡ **Fastest Algorithms**: 
  - **DQN-A*** (0.01s) and **Theta*** (0.08 - 0.12s) are the fastest.
- 🛤️ **Shortest Path**: 
  - **Theta*** achieves the shortest path length (82.27) with minimal direction changes (4).
- 🔄 **Best for Dynamic Environments**: 
  - **Bidirectional A*** and **RRT*** excel in adaptability to dynamic environments.
- 🤖 **AI-Enhanced Performance**: 
  - **DQN-A*** combines high optimality with very high computational efficiency and scalability.



## 🤝 Contributing
Contributions are welcome! To contribute:
1. Fork the repository
2. Create a new branch (`git checkout -b feature-branch`)
3. Commit changes (`git commit -m "Add new feature"`)
4. Push to the branch (`git push origin feature-branch`)
5. Open a Pull Request

---

## 📜 License
This project is licensed under the **MIT License** - see the LICENSE file for details.

---

## 📚 **Citation**

If you find this repository useful for your work, please consider citing it in your research or projects. Below are the citation details in **APA 7th edition** and **BibTeX** formats.

---

### **APA 7th Edition**

```plaintext
Gouda Shanbog, D. N. (2025). Path-Planning-for-Intelligent-Mobile-Robots (1.0.1). Zenodo. https://doi.org/10.5281/zenodo.14954689
```
- BibTeX
```
@software{gouda_shanbog_2025_14954689,
  author       = {Gouda Shanbog, Dada Nanjesha},
  title        = {Path-Planning-for-Intelligent-Mobile-Robots},
  month        = feb,
  year         = 2025,
  publisher    = {Zenodo},
  version      = {1.0.1},
  doi          = {10.5281/zenodo.14954689},
  url          = {https://doi.org/10.5281/zenodo.14954689},
}
```
---
## 📬 Contact
For inquiries or collaborations, reach out via **GitHub Issues**.

---

🎉 **Happy Coding!** 🚀

