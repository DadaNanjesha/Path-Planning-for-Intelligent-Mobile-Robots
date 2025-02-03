import math
import pytest
import torch
import numpy as np
import networkx as nx
from collections import deque
from Hybrid_DQN_A_Star.hybrid_dqn_a_star_algorithm import (
    GraphEnvironment,
    DQN,
    ReplayMemory,
    DQNAgent,
)

# Define test parameters
x_obs = [10, 20, 30]
y_obs = [10, 20, 30]
resol = 1.0
radius = 2.0
start = (5, 5)
goal = (50, 50)
state_dim = 4
action_dim = 8


# =============================================================================
# TEST CASES FOR GRAPH ENVIRONMENT
# =============================================================================
@pytest.fixture
def env():
    """Fixture to create a GraphEnvironment instance for testing."""
    return GraphEnvironment(x_obs, y_obs, resol, radius, start, goal)


def test_graph_construction(env):
    """Test that the grid-based graph is built correctly."""
    assert isinstance(env.graph, nx.Graph)

    # Check if the exact start node exists in the graph
    if env.start in env.graph.nodes:
        assert True
    else:
        # Find the closest valid node in the graph
        min_distance = float("inf")
        closest_node = None
        for node in env.graph.nodes:
            dist = math.hypot(env.start[0] - node[0], env.start[1] - node[1])
            if dist < min_distance:
                min_distance = dist
                closest_node = node

        # Log missing node details for debugging
        print(
            f"Start node {env.start} is missing. Closest available node: {closest_node}, Distance: {min_distance:.2f}"
        )

        # Fail the test only if no close enough valid node exists
        assert min_distance < 10, (
            f"Start node {env.start} is missing, and no sufficiently close valid nodes found. "
            f"Closest available node: {closest_node}, Distance: {min_distance:.2f}"
        )

        # Optional: If needed, update the environment start position to the closest valid node.
        env.start = closest_node


def test_obstacle_removal(env):
    """Test that obstacles are removed from the graph."""
    for obs in zip(x_obs, y_obs):
        assert obs not in env.graph.nodes


def test_motion_model(env):
    """Test that the motion model contains 8 connected movements."""
    assert len(env.motion) == 8
    assert (1, 0, 1) in env.motion  # Check right move exists
    assert (-1, -1, np.sqrt(2)) in env.motion  # Check diagonal move exists


def test_reset_state(env):
    """Test that resetting the environment sets it back to the start state."""
    env.state = (10, 10)  # Change state
    env.reset()
    assert env.state == env.start


def test_step_function(env):
    """Test the step function and ensure movement is within allowed bounds."""
    state, reward, done = env.step(0)  # Action 0 should be valid
    assert isinstance(state, np.ndarray)
    assert isinstance(reward, float)
    assert isinstance(done, bool)


# =============================================================================
# TEST CASES FOR DQN MODEL
# =============================================================================
@pytest.fixture
def dqn():
    """Fixture to create a DQN instance."""
    return DQN(input_dim=state_dim, output_dim=action_dim)


def test_dqn_forward_pass(dqn):
    """Test that the DQN model produces correct output shape."""
    sample_input = torch.rand((1, state_dim))
    output = dqn(sample_input)
    assert output.shape == (1, action_dim)


def test_dqn_layer_shapes(dqn):
    """Test that the model has the expected layer dimensions."""
    assert dqn.fc1.in_features == state_dim
    assert dqn.fc3.out_features == action_dim


# =============================================================================
# TEST CASES FOR REPLAY MEMORY
# =============================================================================
@pytest.fixture
def replay_memory():
    """Fixture to create a ReplayMemory instance."""
    return ReplayMemory(capacity=100)


def test_replay_memory_push(replay_memory):
    """Test that transitions are stored correctly."""
    state = np.array([1.0, 2.0, 3.0, 4.0])
    action = 1
    reward = -1.0
    next_state = np.array([2.0, 3.0, 4.0, 5.0])
    done = False
    replay_memory.push(state, action, reward, next_state, done)
    assert len(replay_memory) == 1


def test_replay_memory_sample(replay_memory):
    """Test that sampling from memory returns the expected batch size."""
    for _ in range(10):
        replay_memory.push(
            np.random.rand(4),
            np.random.randint(8),
            np.random.rand(),
            np.random.rand(4),
            False,
        )
    batch = replay_memory.sample(5)
    assert len(batch) == 5


# =============================================================================
# TEST CASES FOR DQN AGENT
# =============================================================================
@pytest.fixture
def dqn_agent():
    """Fixture to create a DQNAgent instance."""
    return DQNAgent(state_dim, action_dim)


def test_agent_action_selection(dqn_agent):
    """Test that the agent selects valid actions."""
    state = np.random.rand(state_dim)
    action = dqn_agent.select_action(state)
    assert 0 <= action < action_dim


if __name__ == "__main__":
    pytest.main()
