import pytest
import networkx as nx
import math
from RRT_Star.rrt_star_algorithm import RRTStar


# Test environment parameters
X_MIN, Y_MIN = -10, -10
X_MAX, Y_MAX = 60, 60
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
def rrt_star():
    """Fixture to initialize RRT* instance before tests."""
    return RRTStar(X_OBS, Y_OBS, X_MIN, Y_MIN, X_MAX, Y_MAX, ROBOT_RADIUS)


# -------------------------------
# ✅ UNIT TEST CASES FOR RRT*
# -------------------------------


def test_graph_initialization(rrt_star):
    """Test if the graph is correctly initialized."""
    assert isinstance(rrt_star.graph, nx.Graph), "Graph should be a networkx Graph"
    assert len(rrt_star.graph.nodes) == 0, "Graph should be empty before planning"


def test_random_node_generation(rrt_star):
    """Test if random node generation returns a valid node within bounds."""
    rnd_node = rrt_star.get_random_node()
    assert isinstance(rnd_node, tuple), "Random node should be a tuple"
    assert (
        X_MIN <= rnd_node[0] <= X_MAX
    ), "Random node x-coordinate should be within bounds"
    assert (
        Y_MIN <= rnd_node[1] <= Y_MAX
    ), "Random node y-coordinate should be within bounds"


def test_nearest_node_selection(rrt_star):
    """Test if nearest node function correctly finds the closest node."""
    rrt_star.graph.add_node((10, 10))
    rrt_star.graph.add_node((20, 20))
    nearest = rrt_star.get_nearest_node((15, 15))
    assert nearest in [
        (10, 10),
        (20, 20),
    ], "Nearest node should be one of the existing nodes"


def test_steer_function(rrt_star):
    """Test if the steer function returns a valid node within step size."""
    from_node = (10, 10)
    to_node = (20, 20)
    new_node = rrt_star.steer(from_node, to_node)
    assert isinstance(new_node, tuple), "Steered node should be a tuple"
    assert (
        rrt_star.calc_distance(from_node, new_node) <= rrt_star.step_size
    ), "New node should be within step size"


def test_find_near_nodes(rrt_star):
    """Test if nearby nodes are correctly identified."""
    rrt_star.graph.add_nodes_from([(10, 10), (12, 12), (15, 15), (25, 25)])
    near_nodes = rrt_star.find_near_nodes((14, 14))
    assert len(near_nodes) > 0, "There should be at least one nearby node"


def test_connect_nodes(rrt_star):
    """Test if nodes are correctly connected and costs updated."""
    rrt_star.graph.add_node(START)
    rrt_star.graph.add_node((20, 20))
    rrt_star.came_from[START] = None
    rrt_star.cost[START] = 0

    rrt_star.connect_nodes(START, (20, 20), [START])
    assert (START, (20, 20)) in rrt_star.graph.edges, "Nodes should be connected"
    assert (
        rrt_star.came_from[(20, 20)] == START
    ), "New node should have the correct parent"


def test_path_finalization(rrt_star):
    """Test if the path finalization function correctly retrieves the path."""
    rrt_star.came_from = {
        GOAL: (40, 40),
        (40, 40): (30, 30),
        (30, 30): START,
    }
    rrt_star.goal = GOAL
    path = rrt_star.finalize_path()
    assert path[0] == START, "Path should start from the start node"
    assert path[-1] == GOAL, "Path should end at the goal node"
    assert len(path) > 2, "Path should contain multiple nodes"


def test_path_metrics(rrt_star):
    """Test if the path metrics calculation functions work correctly."""
    path = [(10, 10), (20, 20), (30, 30), (40, 40), (50, 50)]
    rrt_star.calculate_metrics(path)

    assert rrt_star.metrics["path_length"] > 0, "Path length should be greater than 0"
    assert (
        rrt_star.metrics["steps_taken"] == len(path) - 1
    ), "Steps taken should match path length - 1"
    assert (
        rrt_star.metrics["direction_changes"] >= 0
    ), "Direction changes should be non-negative"
