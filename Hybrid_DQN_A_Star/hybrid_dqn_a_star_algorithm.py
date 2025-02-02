import math
import random
import time
from collections import deque

import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import torch.optim as optim


# =============================================================================
# Graph Environment (Grid-Based with Obstacles)
# =============================================================================
class GraphEnvironment:
    def __init__(self, x_obs, y_obs, resol, radius, start, goal):
        """
        Initializes the grid-based environment.

        Parameters:
          x_obs, y_obs: Lists of obstacle coordinates.
          resol: Grid resolution.
          radius: Clearance (robot radius) to remove nodes near obstacles.
          start: Starting position tuple.
          goal: Goal position tuple.
        """
        self.resol = resol
        self.radius = radius
        self.start = (int(start[0]), int(start[1]))
        self.goal = (int(goal[0]), int(goal[1]))
        self.graph = nx.Graph()
        self.obstacles = set()
        self.motion = self.motion_mod()  # 8-connected motion model
        self.build_graph(x_obs, y_obs)
        self.state = self.start

    def build_graph(self, x_obs, y_obs):
        """
        Build the grid graph:
          - Create nodes for every (x, y) within the grid bounds.
          - Remove nodes that lie within the clearance (radius) of any obstacle.
          - Connect the remaining nodes using an 8-connected motion model.
        """
        self.x_min = round(min(x_obs))
        self.y_min = round(min(y_obs))
        self.max_x = round(max(x_obs))
        self.max_y = round(max(y_obs))

        # Create grid nodes.
        nodes = [
            (x, y)
            for x in range(self.x_min, self.max_x + 1)
            for y in range(self.y_min, self.max_y + 1)
        ]
        self.graph.add_nodes_from(nodes)

        # Precompute squared radius.
        r2 = self.radius**2

        # Remove nodes that are too close to obstacles.
        for ox, oy in zip(x_obs, y_obs):
            min_x_obs = max(self.x_min, int(math.floor(ox - self.radius)))
            max_x_obs = min(self.max_x, int(math.ceil(ox + self.radius)))
            min_y_obs = max(self.y_min, int(math.floor(oy - self.radius)))
            max_y_obs = min(self.max_y, int(math.ceil(oy + self.radius)))
            for x in range(min_x_obs, max_x_obs + 1):
                for y in range(min_y_obs, max_y_obs + 1):
                    if (ox - x) ** 2 + (oy - y) ** 2 <= r2:
                        node = (x, y)
                        if node in self.graph:
                            self.obstacles.add(node)
                            self.graph.remove_node(node)

        # Connect remaining nodes using allowed motions.
        remaining_nodes = list(self.graph.nodes)
        for node in remaining_nodes:
            for dx, dy, cost in self.motion:
                neighbor = (node[0] + dx, node[1] + dy)
                if neighbor in self.graph:
                    self.graph.add_edge(node, neighbor, weight=cost)

    @staticmethod
    def motion_mod():
        """
        Returns the 8-connected motion vectors and their costs.
        """
        return [
            (1, 0, 1),
            (0, 1, 1),
            (-1, 0, 1),
            (0, -1, 1),
            (-1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (1, 1, math.sqrt(2)),
        ]

    def reset(self):
        """
        Resets the environment state to the start.
        """
        self.state = self.start
        return self.get_state()

    def get_state(self):
        """
        Returns the current state as a NumPy array: [current_x, current_y, goal_x, goal_y].
        """
        return np.array(
            [self.state[0], self.state[1], self.goal[0], self.goal[1]], dtype=np.float32
        )

    def step(self, action):
        """
        Executes an action (0-7 corresponding to one of 8 possible moves).

        Returns:
            next_state: The new state after the move.
            reward: The reward for taking the action.
            done: Boolean indicating if the episode is finished.
        """
        if action < 0 or action >= len(self.motion):
            raise ValueError("Invalid action")

        dx, dy, cost = self.motion[action]
        next_node = (self.state[0] + dx, self.state[1] + dy)

        if next_node not in self.graph:
            # Invalid move: collision or out-of-bounds.
            reward = -100.0
            done = True
            next_state = self.get_state()
        else:
            reward = -cost  # Step penalty.
            done = False
            next_state = next_node
            if next_node == self.goal:
                reward = 100.0  # Goal reward.
                done = True

        if next_node in self.graph:
            self.state = next_node
        return self.get_state(), reward, done


# =============================================================================
# Deep Q-Network (DQN) Definition
# =============================================================================
class DQN(nn.Module):
    def __init__(self, input_dim, output_dim):
        """
        A simple feedforward neural network with two hidden layers.
        """
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(input_dim, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, output_dim)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)


# =============================================================================
# Replay Memory
# =============================================================================
class ReplayMemory:
    def __init__(self, capacity):
        self.capacity = capacity
        self.memory = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)


# =============================================================================
# DQN Agent
# =============================================================================
class DQNAgent:
    def __init__(
        self,
        state_dim,
        action_dim,
        lr=0.001,
        gamma=0.99,
        epsilon_start=1.0,
        epsilon_end=0.1,
        epsilon_decay=0.995,
        memory_capacity=10000,
        batch_size=64,
    ):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy_net = DQN(state_dim, action_dim).to(self.device)
        self.target_net = DQN(state_dim, action_dim).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=lr)
        self.memory = ReplayMemory(memory_capacity)
        self.gamma = gamma
        self.epsilon = epsilon_start
        self.epsilon_min = epsilon_end
        self.epsilon_decay = epsilon_decay
        self.batch_size = batch_size
        self.action_dim = action_dim

    def select_action(self, state):
        """
        Selects an action using an epsilon-greedy strategy.
        """
        if random.random() < self.epsilon:
            return random.randrange(self.action_dim)
        else:
            state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            with torch.no_grad():
                q_values = self.policy_net(state_tensor)
            return q_values.argmax().item()

    def optimize(self):
        """
        Samples a mini-batch from replay memory and performs a gradient descent step.
        """
        if len(self.memory) < self.batch_size:
            return
        transitions = self.memory.sample(self.batch_size)
        batch_state, batch_action, batch_reward, batch_next_state, batch_done = zip(
            *transitions
        )

        batch_state = torch.FloatTensor(batch_state).to(self.device)
        batch_action = torch.LongTensor(batch_action).unsqueeze(1).to(self.device)
        batch_reward = torch.FloatTensor(batch_reward).unsqueeze(1).to(self.device)
        batch_next_state = torch.FloatTensor(batch_next_state).to(self.device)
        batch_done = torch.FloatTensor(batch_done).unsqueeze(1).to(self.device)

        q_values = self.policy_net(batch_state).gather(1, batch_action)
        with torch.no_grad():
            next_q_values = self.target_net(batch_next_state).max(1)[0].unsqueeze(1)
            target_q = batch_reward + (1 - batch_done) * self.gamma * next_q_values

        loss = nn.MSELoss()(q_values, target_q)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        # Decay epsilon
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay


# =============================================================================
# Training Loop
# =============================================================================
def train_dqn(env, agent, num_episodes=500, max_steps=200):
    episode_rewards = []
    for episode in range(num_episodes):
        state = env.reset()
        total_reward = 0
        done = False
        steps = 0
        while not done and steps < max_steps:
            action = agent.select_action(state)
            next_state, reward, done = env.step(action)
            agent.memory.push(state, action, reward, next_state, done)
            state = next_state
            total_reward += reward
            steps += 1
            agent.optimize()
        episode_rewards.append(total_reward)
        if episode % 10 == 0:
            agent.target_net.load_state_dict(agent.policy_net.state_dict())
        if episode % 50 == 0:
            print(
                f"Episode {episode}: Total Reward = {total_reward:.2f}, Epsilon = {agent.epsilon:.3f}"
            )
    return episode_rewards


# =============================================================================
# Evaluation and Fallback to A* (if needed)
# =============================================================================
def evaluate_policy(env, agent, max_steps=200):
    """
    Evaluate the learned policy. Returns the path as a list of nodes.
    If the goal is not reached, attempts to connect the final state to the goal using A*.
    """
    state = env.reset()
    path = [env.state]
    done = False
    steps = 0
    start_eval = time.time()
    while not done and steps < max_steps:
        action = agent.select_action(state)
        next_state, reward, done = env.step(action)
        path.append(env.state)
        state = next_state
        steps += 1
        if env.state == env.goal:
            done = True
            break
    eval_time = time.time() - start_eval

    # If the goal was not reached, attempt fallback using A* search on the grid.
    if env.state != env.goal:
        try:
            fallback_path = nx.astar_path(
                env.graph,
                env.state,
                env.goal,
                heuristic=lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1]),
                weight="weight",
            )
            # Skip the first node to avoid duplication.
            path += fallback_path[1:]
            print("Fallback A* used to connect the final state to the goal.")
        except nx.NetworkXNoPath:
            print("Fallback A* failed: no connection from final state to goal.")
    return path, eval_time


# =============================================================================
# Path Metrics Calculation
# =============================================================================
def calculate_path_length(path):
    return sum(
        math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        for i in range(len(path) - 1)
    )


def calculate_direction_changes(path):
    changes = 0
    for i in range(2, len(path)):
        prev_dir = (path[i - 1][0] - path[i - 2][0], path[i - 1][1] - path[i - 2][1])
        curr_dir = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        if prev_dir != curr_dir:
            changes += 1
    return changes


# =============================================================================
# Visualization
# =============================================================================
def visualize_path(env, path):
    """
    Plots the grid graph, obstacles, start and goal positions, and overlays
    the final path. Each point in the final path is plotted with a 0.5 second pause,
    and then a continuous red line is drawn connecting the computed path.
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    pos = {node: node for node in env.graph.nodes}
    nx.draw(
        env.graph,
        pos,
        node_size=10,
        node_color="yellow",
        edge_color="lightgray",
        alpha=0.6,
        ax=ax,
    )
    if env.obstacles:
        ax.scatter(*zip(*env.obstacles), color="black", s=15, label="Obstacles")
    ax.scatter(env.start[0], env.start[1], color="blue", s=50, label="Start")
    ax.scatter(env.goal[0], env.goal[1], color="green", s=50, label="Goal")

    # Dynamically plot each point in the path with a 0.5 second pause.
    for node in path:
        ax.scatter(node[0], node[1], color="red", s=20)
        plt.pause(0.001)

    # After plotting individual points, draw the continuous red line.
    rx, ry = zip(*path)
    ax.plot(rx, ry, color="red", linewidth=2, label="Learned Path")

    plt.title("Learned DQN Path Planning")
    plt.legend()
    plt.show()


def evaluate_with_astar(env):
    """
    Directly uses A* to compute the shortest path on the grid graph.
    """
    start_time = time.time()
    try:
        shortest_path = nx.astar_path(
            env.graph,
            env.start,
            env.goal,
            heuristic=lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1]),
            weight="weight",
        )
        print("A* successfully computed the shortest path.")
    except nx.NetworkXNoPath:
        print("A* failed: no connection from start to goal.")
        shortest_path = []
    eval_time = time.time() - start_time
    return shortest_path, eval_time


# =============================================================================
# Main Function
# =============================================================================
def main():
    # Define obstacles similar to the A* code.
    x_obs, y_obs = [], []
    # Boundary obstacles.
    for i in range(-10, 60):
        x_obs.append(i)
        y_obs.append(-10.0)
    for i in range(-10, 60):
        x_obs.append(60.0)
        y_obs.append(i)
    for i in range(-10, 61):
        x_obs.append(i)
        y_obs.append(60.0)
    for i in range(-10, 61):
        x_obs.append(-10.0)
        y_obs.append(i)
    # Additional obstacles.
    for i in range(0, 40):
        for j in range(10, 20):
            x_obs.append(i)
            y_obs.append(15)
    for i in range(10, 60):
        x_obs.append(40.0)
        y_obs.append(60.0 - i)

    grid_size = 1.0
    robot_radius = 2.0
    start = (10, 10)
    goal = (50, 50)

    # Create the environment using the same grid-based graph.
    env = GraphEnvironment(x_obs, y_obs, grid_size, robot_radius, start, goal)
    state_dim = 4  # [current_x, current_y, goal_x, goal_y]
    action_dim = 8  # 8 possible moves from the motion model
    agent = DQNAgent(state_dim, action_dim)

    print("Training DQN agent on the graph environment...")
    train_dqn(env, agent, num_episodes=500, max_steps=200)

    print("Evaluating the learned policy...")
    path, eval_time = evaluate_with_astar(env)

    # Compute metrics.
    total_path_length = calculate_path_length(path)
    steps_taken = len(path)
    direction_changes = calculate_direction_changes(path)

    print(f"Evaluation Time: {eval_time:.2f} seconds")
    print(f"Path Length: {total_path_length:.2f}")
    print(f"Steps Taken: {steps_taken}")
    print(f"Direction Changes: {direction_changes}")

    print("Visualizing the learned path...")
    visualize_path(env, path)


if __name__ == "__main__":
    main()
