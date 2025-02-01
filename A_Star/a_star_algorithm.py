import math
import time
import networkx as nx
import matplotlib.pyplot as plt


class AStarAlgorithm:
    def __init__(self, x_obs, y_obs, resol, radius):
        """
        Initialize the A* algorithm with obstacles and parameters.
        """
        self.resol = resol
        self.radius = radius
        self.graph = nx.Graph()
        self.obstacles = set()
        self.motion = self.motion_mod()
        self.build_graph(x_obs, y_obs)

    def build_graph(self, x_obs, y_obs):
        """
        Build the graph with nodes, edges, and obstacles.
        """
        self.x_min = round(min(x_obs))
        self.y_min = round(min(y_obs))
        self.max_x = round(max(x_obs))
        self.max_y = round(max(y_obs))

        # Create grid nodes in one go.
        nodes = [
            (x, y)
            for x in range(self.x_min, self.max_x + 1)
            for y in range(self.y_min, self.max_y + 1)
        ]
        self.graph.add_nodes_from(nodes)

        # Precompute squared radius to avoid repeated sqrt calculations.
        r2 = self.radius**2

        # Efficiently remove obstacle nodes by iterating only over a bounding box for each obstacle.
        for ox, oy in zip(x_obs, y_obs):
            min_x_obs = max(self.x_min, int(math.floor(ox - self.radius)))
            max_x_obs = min(self.max_x, int(math.ceil(ox + self.radius)))
            min_y_obs = max(self.y_min, int(math.floor(oy - self.radius)))
            max_y_obs = min(self.max_y, int(math.ceil(oy + self.radius)))
            for x in range(min_x_obs, max_x_obs + 1):
                for y in range(min_y_obs, max_y_obs + 1):
                    if (ox - x) ** 2 + (oy - y) ** 2 <= r2:
                        node = (x, y)
                        if self.graph.has_node(node):
                            self.obstacles.add(node)
                            self.graph.remove_node(node)

        # Connect remaining nodes based on allowed motions.
        remaining_nodes = list(
            self.graph.nodes
        )  # Avoid issues with concurrent modification.
        for node in remaining_nodes:
            for dx, dy, cost in self.motion:
                neighbor = (node[0] + dx, node[1] + dy)
                if neighbor in self.graph:
                    self.graph.add_edge(node, neighbor, weight=cost)

    @staticmethod
    def motion_mod():
        """
        Define motion vectors for 8-connected grid.
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

    def planning(self, sx, sy, gx, gy, dynamic_visualization=False):
        """
        Perform A* pathfinding from start (sx, sy) to goal (gx, gy).
        """
        start_time = time.time()

        start = (int(sx), int(sy))
        goal = (int(gx), int(gy))

        if start not in self.graph or goal not in self.graph:
            print("Start or goal node is invalid!")
            return [], []

        # Set up visualization.
        fig, ax = plt.subplots(figsize=(10, 10))
        pos = {node: node for node in self.graph.nodes}
        nx.draw(
            self.graph,
            pos,
            node_size=10,
            node_color="yellow",
            edge_color="lightgray",
            alpha=0.6,
            ax=ax,
        )
        if self.obstacles:
            ax.scatter(*zip(*self.obstacles), color="black", label="Obstacles", s=15)
        ax.scatter(*start, color="blue", label="Start", s=50)
        ax.scatter(*goal, color="green", label="Goal", s=50)
        plt.legend()

        try:
            path = nx.astar_path(
                self.graph, start, goal, heuristic=self.calc_heuristic, weight="weight"
            )
        except nx.NetworkXNoPath:
            print("No path found!")
            plt.title("No Path Found")
            plt.show()
            return [], []

        # Optional dynamic visualization of the explored nodes.
        if dynamic_visualization:
            explored = set()
            for node in path:
                explored.add(node)
                ax.scatter(
                    node[0],
                    node[1],
                    color="red",
                    s=20,
                    label="Explored" if len(explored) == 1 else "",
                )
                plt.pause(0.1)

        rx, ry = zip(*path)

        # Calculate metrics.
        path_length = self.calculate_path_length(path)
        execution_time = time.time() - start_time
        steps = len(path)
        direction_changes = self.calculate_direction_changes(path)

        print(f"Execution Time: {execution_time:.2f} seconds")
        print(f"Path Length: {path_length:.2f}")
        print(f"Steps Taken: {steps}")
        print(f"Direction Changes: {direction_changes}")

        # Plot final path
        if dynamic_visualization:
            ax.plot(rx, ry, color="red", label="Path", linewidth=2)
            plt.pause(0.1)

        plt.title("A* Path Planning")
        plt.show()
        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        """
        Calculate Euclidean distance as a heuristic.
        """
        return math.hypot(n1[0] - n2[0], n1[1] - n2[1])

    @staticmethod
    def calculate_path_length(path):
        """
        Calculate the total length of the path.
        """
        return sum(
            math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
            for i in range(len(path) - 1)
        )

    @staticmethod
    def calculate_direction_changes(path):
        """
        Calculate the number of direction changes in the path.
        """
        changes = 0
        for i in range(2, len(path)):
            prev_dir = (
                path[i - 1][0] - path[i - 2][0],
                path[i - 1][1] - path[i - 2][1],
            )
            curr_dir = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            if prev_dir != curr_dir:
                changes += 1
        return changes


def main():
    """
    Main function to execute A* algorithm.
    """
    sx, sy = 10.0, 10.0
    gx, gy = 50.0, 50.0
    grid_size = 1.0
    robot_radius = 2.0

    x_obs, y_obs = [], []
    # Define boundary obstacles.
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

    a_star = AStarAlgorithm(x_obs, y_obs, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy, dynamic_visualization=True)


if __name__ == "__main__":
    main()
