import math
import random
import time
import networkx as nx
import matplotlib.pyplot as plt


class RRTStar:
    def __init__(
        self,
        x_obs,
        y_obs,
        x_min,
        y_min,
        x_max,
        y_max,
        robot_radius,
        max_iter=1000,
        step_size=5.0,
        goal_sample_rate=5,
    ):
        """
        Initializes the RRT* algorithm with given parameters.

        Parameters:
            x_obs, y_obs: Lists of obstacle coordinates.
            x_min, y_min, x_max, y_max: Boundaries of the environment grid.
            robot_radius: The clearance around the robot to avoid obstacles.
            max_iter: Maximum number of iterations for the RRT* search.
            step_size: Step size for extending the tree towards a random sample.
            goal_sample_rate: Probability of randomly selecting the goal node during the search.
        """
        # Boundaries and robot parameters
        self.x_min = x_min
        self.y_min = y_min
        self.x_max = x_max
        self.y_max = y_max
        self.robot_radius = robot_radius
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate

        # Obstacles and graph initialization
        self.obstacles = set(zip(x_obs, y_obs))
        self.graph = nx.Graph()

        # Start and goal positions (to be set during planning)
        self.start = None
        self.goal = None

        # Visualization setup
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.setup_visualization()

        # Metrics for tracking path performance
        self.metrics = {
            "execution_time": 0,
            "path_length": 0,
            "steps_taken": 0,
            "direction_changes": 0,
        }

        # Pathfinding auxiliary data structures
        self.came_from = {}
        self.cost = {}

    def setup_visualization(self):
        """Initialize visualization with obstacles and axis settings."""
        self.ax.set_xlim(self.x_min, self.x_max)
        self.ax.set_ylim(self.y_min, self.y_max)

        # Plot obstacles
        if self.obstacles:
            self.ax.scatter(
                *zip(*self.obstacles), color="black", s=15, label="Obstacles"
            )

        # Ensure aspect ratio is equal for proper scaling of the plot
        self.ax.set_aspect("equal")

    def planning(self, start, goal, dynamic_visualization=False):
        """
        Perform RRT* path planning to find a path from the start to the goal.

        Parameters:
            start: Starting position (tuple).
            goal: Goal position (tuple).
            dynamic_visualization: Boolean to enable dynamic visualization of the search process.

        Returns:
            path: A list of nodes representing the computed path from start to goal.
        """
        start_time = time.time()

        # Initialize start and goal
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.graph.add_node(self.start)
        self.came_from[self.start] = None
        self.cost[self.start] = 0

        # Plot start and goal positions
        self.ax.scatter(*self.start, color="blue", s=50, label="Start")
        self.ax.scatter(*self.goal, color="green", s=50, label="Goal")
        plt.legend()

        # RRT* planning loop
        for _ in range(self.max_iter):
            # Generate random node (biased towards the goal sometimes)
            rnd_node = self.get_random_node()
            nearest_node = self.get_nearest_node(rnd_node)
            new_node = self.steer(nearest_node, rnd_node)

            # Check for collision and process the new node
            if self.check_collision(nearest_node, new_node):
                near_nodes = self.find_near_nodes(new_node)
                self.connect_nodes(nearest_node, new_node, near_nodes)

                # Visualize the current node and edge if required
                if dynamic_visualization:
                    self.update_visualization(new_node)

                # Try to connect the new node to the goal
                if self.try_connect_to_goal(new_node):
                    break

        # Record execution time and finalize the path
        self.metrics["execution_time"] = time.time() - start_time
        path = self.finalize_path()
        self.show_metrics()

        # Display the path planning visualization
        plt.title("RRT* Path Planning")
        plt.show()

        return path

    def connect_nodes(self, nearest_node, new_node, near_nodes):
        """
        Connect the new node to the tree, considering nearby nodes for optimization.

        Parameters:
            nearest_node: The closest node to the new node.
            new_node: The newly generated node to add to the tree.
            near_nodes: A list of nearby nodes to check for possible better connections.
        """
        min_cost_node = nearest_node
        min_cost = self.cost[nearest_node] + self.calc_distance(nearest_node, new_node)

        # Find the best connecting node based on cost and collision checks
        for near_node in near_nodes:
            if self.check_collision(near_node, new_node):
                tentative_cost = self.cost[near_node] + self.calc_distance(
                    near_node, new_node
                )
                if tentative_cost < min_cost:
                    min_cost = tentative_cost
                    min_cost_node = near_node

        # Add the best node connection to the graph
        self.graph.add_edge(min_cost_node, new_node, weight=min_cost)
        self.came_from[new_node] = min_cost_node
        self.cost[new_node] = min_cost

        # Check if the new node can optimize connections to near nodes
        for near_node in near_nodes:
            if self.check_collision(new_node, near_node):
                tentative_cost = self.cost[new_node] + self.calc_distance(
                    new_node, near_node
                )
                if tentative_cost < self.cost.get(near_node, float("inf")):
                    self.came_from[near_node] = new_node
                    self.cost[near_node] = tentative_cost

    def finalize_path(self):
        """Finalize the path from start to goal by backtracking through the came_from dictionary."""
        path = self.get_path(self.goal)
        if path:
            self.draw_final_path(path)
            self.calculate_metrics(path)
        return path

    def draw_final_path(self, path):
        """Draw the final path in red over the current graph."""
        rx, ry = zip(*path)
        self.ax.plot(rx, ry, "r-", linewidth=2, label="Final Path")
        plt.legend()

    def calculate_metrics(self, path):
        """Calculate path metrics: total length, number of steps and direction changes."""
        self.metrics["path_length"] = sum(
            self.calc_distance(path[i], path[i + 1]) for i in range(len(path) - 1)
        )
        self.metrics["steps_taken"] = len(path) - 1
        self.metrics["direction_changes"] = self.count_direction_changes(path)

    def count_direction_changes(self, path):
        """Count the number of direction changes along the path."""
        changes = 0
        prev_vec = None
        for i in range(1, len(path)):
            current_vec = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            if prev_vec and current_vec != prev_vec:
                changes += 1
            prev_vec = current_vec
        return changes

    def show_metrics(self):
        """Display metrics both in the console and on the plot."""
        metrics_text = (
            f"Execution Time: {self.metrics['execution_time']:.2f} s\n"
            f"Path Length: {self.metrics['path_length']:.2f}\n"
            f"Steps Taken: {self.metrics['steps_taken']}\n"
            f"Direction Changes: {self.metrics['direction_changes']}"
        )
        self.ax.text(
            0.05,
            0.95,
            metrics_text,
            transform=self.ax.transAxes,
            va="top",
            bbox=dict(facecolor="white", alpha=0.8),
        )
        print(metrics_text)

    def try_connect_to_goal(self, new_node):
        """Attempt to connect a newly added node to the goal."""
        if self.calc_distance(new_node, self.goal) <= self.step_size:
            if self.check_collision(new_node, self.goal):
                self.graph.add_node(self.goal)
                self.graph.add_edge(
                    new_node, self.goal, weight=self.calc_distance(new_node, self.goal)
                )
                self.came_from[self.goal] = new_node
                self.cost[self.goal] = self.cost[new_node] + self.calc_distance(
                    new_node, self.goal
                )
                return True
        return False

    def update_visualization(self, new_node):
        """Dynamically update the visualization as new nodes are added."""
        if self.came_from[new_node]:
            parent = self.came_from[new_node]
            self.ax.plot(
                [parent[0], new_node[0]],
                [parent[1], new_node[1]],
                "lightgray",
                alpha=0.6,
            )
        self.ax.plot(new_node[0], new_node[1], "yo", markersize=3)
        plt.pause(0.001)

    def point_to_segment_distance(self, p, a, b):
        """
        Calculate the minimum distance from point p to the line segment ab.
        This is used to detect if an obstacle (at p) is too close to the path from a to b.
        """
        ap = (p[0] - a[0], p[1] - a[1])
        ab = (b[0] - a[0], b[1] - a[1])
        ab_sq = ab[0] ** 2 + ab[1] ** 2
        if ab_sq == 0:
            return self.calc_distance(p, a)
        t = max(0, min(1, (ap[0] * ab[0] + ap[1] * ab[1]) / ab_sq))
        projection = (a[0] + t * ab[0], a[1] + t * ab[1])
        return self.calc_distance(p, projection)

    def get_random_node(self):
        """Return a random node within the search space with a chance to bias toward the goal."""
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = (
                random.uniform(self.x_min, self.x_max),
                random.uniform(self.y_min, self.y_max),
            )
        else:
            rnd = self.goal
        return rnd

    def get_nearest_node(self, rnd_node):
        """Find the nearest node in the tree to a given random node."""
        nearest_node = None
        min_dist = float("inf")
        for node in self.graph.nodes:
            dist = self.calc_distance(node, rnd_node)
            if dist < min_dist:
                nearest_node = node
                min_dist = dist
        return nearest_node

    def steer(self, from_node, to_node):
        """Steer from the current node toward the target node by a fixed step size."""
        dist = self.calc_distance(from_node, to_node)
        if dist < self.step_size:
            return to_node
        theta = math.atan2(to_node[1] - from_node[1], to_node[0] - from_node[0])
        return (
            from_node[0] + self.step_size * math.cos(theta),
            from_node[1] + self.step_size * math.sin(theta),
        )

    def find_near_nodes(self, new_node):
        """Return a list of nodes that lie within a neighborhood of the new node."""
        n = len(self.graph.nodes) + 1
        r = min(self.step_size * math.sqrt(math.log(n) / n), self.step_size)
        near_nodes = []
        for node in self.graph.nodes:
            if self.calc_distance(node, new_node) <= r:
                near_nodes.append(node)
        return near_nodes

    def check_collision(self, from_node, to_node):
        """
        Check if the path between from_node and to_node is collision-free.
        This method tests both the endpoints and the line segment between them.
        """
        for ox, oy in self.obstacles:
            # Check if either endpoint is too close to an obstacle.
            if self.calc_distance(from_node, (ox, oy)) <= self.robot_radius:
                return False
            if self.calc_distance(to_node, (ox, oy)) <= self.robot_radius:
                return False
            # Check the minimum distance from the obstacle to the line segment.
            if (
                self.point_to_segment_distance((ox, oy), from_node, to_node)
                <= self.robot_radius
            ):
                return False
        return True

    def get_path(self, goal):
        """Retrieve the final path from start to goal by backtracking through the came_from dictionary."""
        path = []
        node = goal
        while node is not None:
            path.append(node)
            node = self.came_from.get(node)
        path.reverse()
        return path

    @staticmethod
    def calc_distance(node1, node2):
        """Calculate the Euclidean distance between two nodes."""
        return math.hypot(node1[0] - node2[0], node1[1] - node2[1])


def main():
    # Define boundaries and robot parameters (same as in your original A* example).
    x_min, y_min = -10, -10
    x_max, y_max = 60, 60
    robot_radius = 2.0

    # Define obstacles along boundaries and additional obstacles.
    x_obs, y_obs = [], []
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
    for i in range(0, 40):
        for j in range(10, 20):
            x_obs.append(i)
            y_obs.append(15)
    for i in range(10, 60):
        x_obs.append(40.0)
        y_obs.append(60.0 - i)

    start = (10.0, 10.0)
    goal = (50.0, 50.0)

    rrt_star = RRTStar(x_obs, y_obs, x_min, y_min, x_max, y_max, robot_radius)
    path = rrt_star.planning(start, goal, dynamic_visualization=True)


if __name__ == "__main__":
    main()
