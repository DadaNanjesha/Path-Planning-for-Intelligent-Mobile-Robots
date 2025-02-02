import math
import time
import networkx as nx
import matplotlib.pyplot as plt


class DStarAlgorithm:
    def __init__(self, x_obs, y_obs, resol, radius):
        """
        Initialize the D* algorithm with obstacles and parameters.
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

        # Add nodes to the graph
        for x in range(self.x_min, self.max_x + 1):
            for y in range(self.y_min, self.max_y + 1):
                self.graph.add_node((x, y))

        # Add obstacles
        for ox, oy in zip(x_obs, y_obs):
            for x in range(
                max(self.x_min, int(ox - self.radius)),
                min(self.max_x, int(ox + self.radius)) + 1,
            ):
                for y in range(
                    max(self.y_min, int(oy - self.radius)),
                    min(self.max_y, int(oy + self.radius)) + 1,
                ):
                    if math.hypot(ox - x, oy - y) <= self.radius:
                        obstacle_node = (x, y)
                        if obstacle_node in self.graph:
                            self.obstacles.add(obstacle_node)
                            self.graph.remove_node(obstacle_node)

        # Connect nodes based on motion
        for node in list(self.graph.nodes):
            for dx, dy, cost in self.motion:
                neighbor = (node[0] + dx, node[1] + dy)
                if neighbor in self.graph:
                    self.graph.add_edge(node, neighbor, weight=cost)

    @staticmethod
    def motion_mod():
        """
        Define motion vectors for an 8-connected grid.
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

    def d_star_planning(
        self, sx, sy, gx, gy, dynamic_visualization=False, pause_time=0.0001
    ):
        """
        Perform D* path planning with dynamic visualization.
        """
        start_time = time.time()

        start = (int(sx), int(sy))
        goal = (int(gx), int(gy))

        if start not in self.graph or goal not in self.graph:
            print("Start or goal node is invalid!")
            return [], []

        # For visualization
        fig, ax = plt.subplots(figsize=(10, 10))
        pos = {node: (node[0], node[1]) for node in self.graph.nodes}
        nx.draw(
            self.graph,
            pos,
            node_size=10,
            node_color="yellow",
            edge_color="lightgray",
            alpha=0.6,
            ax=ax,
        )
        ax.scatter(*zip(*self.obstacles), color="black", label="Obstacles", s=15)
        ax.scatter(*start, color="blue", label="Start", s=50)
        ax.scatter(*goal, color="green", label="Goal", s=50)
        plt.legend()

        # Save the original axis limits
        original_xlim = ax.get_xlim()
        original_ylim = ax.get_ylim()

        # Perform D* Search
        try:
            path, explored_nodes = self.d_star_search(start, goal)
            print("Final Path Nodes:")
        except nx.NetworkXNoPath:
            print("No path found!")
            plt.title("No Path Found")
            plt.show()
            return [], []

        # Dynamic visualization of explored nodes
        if dynamic_visualization:
            for node in explored_nodes:
                ax.scatter(node[0], node[1], color="red", s=20)
                plt.pause(pause_time)

        # Ensure all dynamic elements remain
        plt.draw()

        # Plot the final path
        if path and len(path) > 1:
            rx, ry = zip(*path)
            ax.plot(rx, ry, color="red", label="Final Path", linewidth=2, zorder=10)
            print("Final Path Plotted Successfully.")
        else:
            print("No valid path found to plot.")

        # Restore original zoom level
        ax.set_xlim(original_xlim)
        ax.set_ylim(original_ylim)

        # Metrics
        path_length = self.calculate_path_length(path)
        execution_time = time.time() - start_time

        print(f"Execution Time: {execution_time:.2f} seconds")
        print(f"Path Length: {path_length:.2f}")
        print(f"Steps Taken: {len(path)}")

        plt.title("D* Path Planning")
        plt.legend()
        plt.show()

        return rx, ry

    def d_star_search(self, start, goal):
        """
        Perform the D* search algorithm.
        """
        explored_nodes = set()

        try:
            path = nx.astar_path(
                self.graph, start, goal, heuristic=self.calc_heuristic, weight="weight"
            )
            explored_nodes.update(path)  # Mark explored nodes
            return path, explored_nodes
        except nx.NetworkXNoPath:
            raise nx.NetworkXNoPath("No path found!")

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


def main():
    """
    Main function to execute D* algorithm.
    """
    sx, sy = 10.0, 10.0  # Start position
    gx, gy = 50.0, 50.0  # Goal position
    grid_size = 1.0  # Grid resolution
    robot_radius = 2.0  # Robot size (used for obstacle clearance)

    # Define obstacle positions
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

    # Initialize D* algorithm
    d_star = DStarAlgorithm(x_obs, y_obs, grid_size, robot_radius)

    # Run pathfinding algorithm with visualization
    rx, ry = d_star.d_star_planning(
        sx, sy, gx, gy, dynamic_visualization=True, pause_time=0.001
    )


if __name__ == "__main__":
    main()
