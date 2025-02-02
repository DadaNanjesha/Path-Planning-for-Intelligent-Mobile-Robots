# Point Navigation with Deep Reinforcement Learning

This project demonstrates how to train a Deep Reinforcement Learning (DRL) agent to navigate a 2D space from a starting position to a goal. The agent learns to move within the space by interacting with the environment using a Deep Q-Network (DQN).

---

## Features

- **Dynamic Visualization**: Real-time visualization of the agent's movement in a 2D space.
- **Deep Reinforcement Learning**: Utilizes a DQN for learning optimal policies.
- **Point-Based Representation**: Clearly marks the agent and goal for easy understanding.

---

## Requirements

Ensure you have the following Python libraries installed:

```bash
pip install numpy tensorflow matplotlib
```

---

## Usage

1. Clone this repository or download the code.
2. Run the script using:

   ```bash
   python path_to_script.py
   ```

3. Observe the dynamic visualization in the plotted figure as the agent learns to navigate toward the goal.

---

## Explanation of Components

### Environment
The environment is a 2D space where the agent starts at position `(0, 0)` and must reach the goal at `(area_size - 1, area_size - 1)`. The agent can move:
- **Up**
- **Down**
- **Left**
- **Right**

The environment provides rewards for reaching the goal and penalizes every other movement slightly.

### Agent
The agent uses a Deep Q-Network (DQN) to learn the optimal policy for navigating the environment. The DQN is built using:
- **Fully Connected Layers**: To process the state input and produce Q-values for actions.
- **Replay Memory**: To store past experiences and train the network efficiently.

### Visualization
The agent's position and goal are plotted in real-time. The **agent** is marked with a blue dot, and the **goal** is marked with a red dot. The figure updates dynamically during training.

---

## Example Output

### Dynamic Visualization

![Dynamic Visualization](dynamic_visualization.gif)

> *Note: Replace the above placeholder with an actual GIF or video link showing the agent navigating in real-time.*

---

## Customization

### Modify the Environment
You can adjust the size of the 2D space by changing the `area_size` parameter in the `PointNavigation` class.

### Change Hyperparameters
Experiment with the following hyperparameters in the `DQNAgent` class to see their impact on learning:
- **Learning Rate**
- **Discount Factor**
- **Epsilon Decay**

---

## License

This project is licensed under the MIT License. Feel free to use and modify the code as needed.

---

## Author

Dada Nanjesha

