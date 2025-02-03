import math
import time
import networkx as nx
import matplotlib.pyplot as plt


class AStarAlgorithm:
    def __init__(self, x_obs, y_obs, resol, radius):
        """
        Initialize the A* algorithm with obstacles and parameters.
        Arguments:
        - x_obs, y_obs: Lists of x and y coordinates of the obstacles.
        - resol: The grid resolution (i.e., the spacing between nodes in the grid).
        - radius: The radius of the robot (used to avoid obstacles).
        """
        self.resol = resol  # Grid resolution (spacing between nodes).
        self.radius = radius  # Radius of the robot.
        self.graph = nx.Graph()  # Graph to represent the grid and connections.
        self.obstacles = set()  # Set to store obstacle positions.
        self.motion = (
            self.motion_mod()
        )  # Predefined motion patterns (8-connected grid).
        self.build_graph(x_obs, y_obs)  # Build the graph with obstacles.

    def build_graph(self, x_obs, y_obs):
        """
        Build the graph with nodes, edges, and obstacles.
        The graph consists of a grid with nodes that are connected if they are not blocked by obstacles.
        Arguments:
        - x_obs, y_obs: Lists of x and y coordinates of obstacles.
        """
        # Determine the bounding box for the grid based on the obstacles' coordinates.
        self.x_min = round(min(x_obs))  # Minimum x-coordinate.
        self.y_min = round(min(y_obs))  # Minimum y-coordinate.
        self.max_x = round(max(x_obs))  # Maximum x-coordinate.
        self.max_y = round(max(y_obs))  # Maximum y-coordinate.

        # Create grid nodes (points) from x_min to max_x and y_min to max_y.
        nodes = [
            (x, y)
            for x in range(self.x_min, self.max_x + 1)
            for y in range(self.y_min, self.max_y + 1)
        ]
        self.graph.add_nodes_from(nodes)  # Add these nodes to the graph.

        # Precompute the square of the robot's radius to avoid repeatedly calculating the square root.
        r2 = self.radius**2

        # Iterate over each obstacle and remove nodes within the obstacle's radius from the graph.
        for ox, oy in zip(x_obs, y_obs):
            min_x_obs = max(self.x_min, int(math.floor(ox - self.radius)))
            max_x_obs = min(self.max_x, int(math.ceil(ox + self.radius)))
            min_y_obs = max(self.y_min, int(math.floor(oy - self.radius)))
            max_y_obs = min(self.max_y, int(math.ceil(oy + self.radius)))

            # Iterate over the bounding box of each obstacle.
            for x in range(min_x_obs, max_x_obs + 1):
                for y in range(min_y_obs, max_y_obs + 1):
                    # Check if the node is within the obstacle's radius.
                    if (ox - x) ** 2 + (oy - y) ** 2 <= r2:
                        node = (x, y)
                        if self.graph.has_node(
                            node
                        ):  # If the node exists in the graph.
                            self.obstacles.add(node)  # Add it to the obstacles set.
                            self.graph.remove_node(node)  # Remove it from the graph.

        # Connect remaining nodes based on the allowed motion.
        remaining_nodes = list(
            self.graph.nodes
        )  # Avoid issues with concurrent modification.
        for node in remaining_nodes:
            for dx, dy, cost in self.motion:
                # Create neighbors by applying the motion vectors.
                neighbor = (node[0] + dx, node[1] + dy)
                if neighbor in self.graph:
                    self.graph.add_edge(
                        node, neighbor, weight=cost
                    )  # Add edges between nodes.

    @staticmethod
    def motion_mod():
        """
        Define motion vectors for 8-connected grid.
        This function returns the possible directions in which movement can occur:
        - Horizontal: (1, 0), (0, 1), (-1, 0), (0, -1)
        - Diagonal: (-1, -1), (-1, 1), (1, -1), (1, 1)
        """
        return [
            (1, 0, 1),  # Move right (cost 1)
            (0, 1, 1),  # Move up (cost 1)
            (-1, 0, 1),  # Move left (cost 1)
            (0, -1, 1),  # Move down (cost 1)
            (-1, -1, math.sqrt(2)),  # Move diagonally left-down (cost sqrt(2))
            (-1, 1, math.sqrt(2)),  # Move diagonally left-up (cost sqrt(2))
            (1, -1, math.sqrt(2)),  # Move diagonally right-down (cost sqrt(2))
            (1, 1, math.sqrt(2)),  # Move diagonally right-up (cost sqrt(2))
        ]

    def planning(self, sx, sy, gx, gy, dynamic_visualization=False):
        """
        Perform A* pathfinding from start (sx, sy) to goal (gx, gy).
        Arguments:
        - sx, sy: Starting coordinates.
        - gx, gy: Goal coordinates.
        - dynamic_visualization: Boolean flag to enable step-by-step visualization.
        """
        start_time = time.time()  # Start the timer for performance tracking.

        start = (int(sx), int(sy))  # Start position.
        goal = (int(gx), int(gy))  # Goal position.

        # Check if the start or goal is invalid (not in the graph).
        if start not in self.graph or goal not in self.graph:
            print("Start or goal node is invalid!")
            return [], []

        # Set up visualization (using matplotlib).
        fig, ax = plt.subplots(figsize=(10, 10))
        pos = {node: node for node in self.graph.nodes}  # Positions for nodes.
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
            # Perform A* algorithm to find the shortest path.
            path = nx.astar_path(
                self.graph, start, goal, heuristic=self.calc_heuristic, weight="weight"
            )
        except nx.NetworkXNoPath:
            print("No path found!")
            plt.title("No Path Found")
            plt.show()
            return [], []

        # Optional: visualize the pathfinding process dynamically.
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

        rx, ry = zip(*path)  # Extract path coordinates for x and y.

        # Calculate performance metrics.
        path_length = self.calculate_path_length(path)
        execution_time = time.time() - start_time
        steps = len(path)
        direction_changes = self.calculate_direction_changes(path)

        # Print the results.
        print(f"Execution Time: {execution_time:.2f} seconds")
        print(f"Path Length: {path_length:.2f}")
        print(f"Steps Taken: {steps}")
        print(f"Direction Changes: {direction_changes}")

        # Plot the final path.
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
        This is used to estimate the remaining distance to the goal.
        Arguments:
        - n1, n2: Coordinates of the two nodes.
        """
        return math.hypot(n1[0] - n2[0], n1[1] - n2[1])

    @staticmethod
    def calculate_path_length(path):
        """
        Calculate the total length of the path.
        Arguments:
        - path: List of nodes in the path.
        """
        return sum(
            math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
            for i in range(len(path) - 1)
        )

    @staticmethod
    def calculate_direction_changes(path):
        """
        Calculate the number of direction changes in the path.
        Arguments:
        - path: List of nodes in the path.
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
    Defines the start and goal positions, obstacles, and runs the A* algorithm.
    """
    sx, sy = 10.0, 10.0  # Starting position (x, y).
    gx, gy = 50.0, 50.0  # Goal position (x, y).
    grid_size = 1.0  # Grid resolution.
    robot_radius = 2.0  # Robot radius.

    x_obs, y_obs = [], []  # Lists to store obstacle coordinates.

    # Define boundary obstacles (a square boundary).
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

    # Additional obstacles inside the grid.
    for i in range(0, 40):
        for j in range(10, 20):
            x_obs.append(i)
            y_obs.append(15)
    for i in range(10, 60):
        x_obs.append(40.0)
        y_obs.append(60.0 - i)

    # Initialize the A* algorithm with the defined obstacles and robot radius.
    a_star = AStarAlgorithm(x_obs, y_obs, grid_size, robot_radius)

    # Run the A* algorithm to plan the path.
    rx, ry = a_star.planning(sx, sy, gx, gy, dynamic_visualization=True)


if __name__ == "__main__":
    main()
