import pytest
import networkx as nx
import math
from Theta_Star.theta_star_algorithm import ThetaStarAlgorithm

# Test environment parameters
GRID_SIZE = 1.0
ROBOT_RADIUS = 2.0
START = (10, 10)
GOAL = (50, 50)
X_OBS, Y_OBS = [], []

# Generate obstacles in the environment
for i in range(-10, 60):
    X_OBS.append(i)
    Y_OBS.append(-10.0)
for i in range(-10, 60):
    X_OBS.append(60.0)
    Y_OBS.append(i)
for i in range(-10, 61):
    X_OBS.append(i)
    Y_OBS.append(60.0)
for i in range(-10, 61):
    X_OBS.append(-10.0)
    Y_OBS.append(i)
for i in range(0, 40):
    for j in range(10, 20):
        X_OBS.append(i)
        Y_OBS.append(15)
for i in range(10, 60):
    X_OBS.append(40.0)
    Y_OBS.append(60.0 - i)


@pytest.fixture
def theta_star():
    """Fixture to initialize Theta* instance before tests."""
    return ThetaStarAlgorithm(X_OBS, Y_OBS, GRID_SIZE, ROBOT_RADIUS)


# -------------------------------
# ✅ UNIT TEST CASES FOR THETA*
# -------------------------------


def test_graph_initialization(theta_star):
    """Test if the graph is correctly initialized."""
    assert isinstance(theta_star.graph, nx.Graph), "Graph should be a networkx Graph"
    assert (
        len(theta_star.graph.nodes) > 0
    ), "Graph should not be empty after initialization"


def test_node_removal_near_obstacles(theta_star):
    """Test if nodes near obstacles are removed correctly."""
    for obs in zip(X_OBS, Y_OBS):
        assert (
            obs not in theta_star.graph.nodes
        ), f"Obstacle node {obs} should not be in the graph"


def test_planning_execution(theta_star):
    """Test if the Theta* planning function runs without errors and returns a valid path."""
    path = theta_star.theta_star_search(START, GOAL)
    assert path, "Path should not be empty"
    assert len(path) > 1, "Path should have multiple nodes"
    assert path[0] == START, "Path should start at the given start position"
    assert path[-1] == GOAL, "Path should end at the given goal position"


def test_random_graph_edges(theta_star):
    """Test if edges are correctly added between connected nodes."""
    for node in list(theta_star.graph.nodes)[:10]:  # Check only a few nodes
        for dx, dy, _ in theta_star.motion_mod():
            neighbor = (node[0] + dx, node[1] + dy)
            if neighbor in theta_star.graph.nodes:
                assert theta_star.graph.has_edge(
                    node, neighbor
                ), f"Edge between {node} and {neighbor} should exist"


def test_heuristic_function(theta_star):
    """Test if heuristic function returns correct Euclidean distance."""
    n1, n2 = (10, 10), (13, 14)
    expected_distance = math.hypot(13 - 10, 14 - 10)
    assert math.isclose(
        theta_star.calc_heuristic(n1, n2), expected_distance, rel_tol=1e-3
    ), "Heuristic should match Euclidean distance"


def test_cost_function(theta_star):
    """Test if cost function calculates correct values."""
    n1, n2 = (10, 10), (10, 11)
    if theta_star.graph.has_edge(n1, n2):
        assert math.isclose(
            theta_star.cost(n1, n2), 1, rel_tol=1e-3
        ), "Cost for adjacent nodes should be 1"


def test_path_length_calculation(theta_star):
    """Test if path length calculation is correct."""
    path = [(10, 10), (15, 15), (20, 20), (50, 50)]
    expected_length = sum(
        math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        for i in range(len(path) - 1)
    )
    assert math.isclose(
        theta_star.calculate_path_length(path), expected_length, rel_tol=1e-3
    ), "Path length calculation should be correct"


def test_direction_changes(theta_star):
    """Test if direction change calculation works correctly."""
    path = [(10, 10), (15, 15), (15, 20), (20, 25)]
    assert (
        theta_star.calculate_direction_changes(path) == 2
    ), "Direction change count should be correct"
