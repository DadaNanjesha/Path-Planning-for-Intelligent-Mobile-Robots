import math
import time
import networkx as nx
import matplotlib.pyplot as plt


class DStarAlgorithm:
    def __init__(self, x_obs, y_obs, resol, radius):
        """
        Initialize the D* algorithm with obstacles and parameters.

        Arguments:
        - x_obs, y_obs: Lists of x and y coordinates of the obstacles.
        - resol: The grid resolution (spacing between nodes).
        - radius: The radius of the robot (used for obstacle clearance).
        """
        self.resol = resol  # Grid resolution (spacing between nodes).
        self.radius = radius  # Radius of the robot.
        self.graph = nx.Graph()  # Graph to represent the grid and connections.
        self.obstacles = set()  # Set to store obstacle positions.
        self.motion = (
            self.motion_mod()
        )  # Predefined motion patterns for 8-connected grid.
        self.build_graph(x_obs, y_obs)  # Build the graph with obstacles.

    def build_graph(self, x_obs, y_obs):
        """
        Build the grid graph with nodes, edges, and obstacles.

        Arguments:
        - x_obs, y_obs: Lists of x and y coordinates of obstacles.
        """
        # Determine the bounding box for the grid based on the obstacle coordinates.
        self.x_min = round(min(x_obs))  # Minimum x-coordinate.
        self.y_min = round(min(y_obs))  # Minimum y-coordinate.
        self.max_x = round(max(x_obs))  # Maximum x-coordinate.
        self.max_y = round(max(y_obs))  # Maximum y-coordinate.

        # Create nodes in the graph for each point within the grid bounds.
        for x in range(self.x_min, self.max_x + 1):
            for y in range(self.y_min, self.max_y + 1):
                self.graph.add_node((x, y))

        # Add obstacles: Remove nodes within the obstacle's radius from the graph.
        for ox, oy in zip(x_obs, y_obs):
            for x in range(
                max(self.x_min, int(ox - self.radius)),
                min(self.max_x, int(ox + self.radius)) + 1,
            ):
                for y in range(
                    max(self.y_min, int(oy - self.radius)),
                    min(self.max_y, int(oy + self.radius)) + 1,
                ):
                    # Check if a node lies within the obstacle's radius.
                    if math.hypot(ox - x, oy - y) <= self.radius:
                        obstacle_node = (x, y)
                        if obstacle_node in self.graph:
                            self.obstacles.add(obstacle_node)  # Add to obstacle set.
                            self.graph.remove_node(
                                obstacle_node
                            )  # Remove from the graph.

        # Connect the remaining nodes based on allowed motion vectors.
        for node in list(self.graph.nodes):
            for dx, dy, cost in self.motion:
                neighbor = (node[0] + dx, node[1] + dy)
                if neighbor in self.graph:
                    self.graph.add_edge(
                        node, neighbor, weight=cost
                    )  # Add edges between nodes.

    @staticmethod
    def motion_mod():
        """
        Define motion vectors for an 8-connected grid.

        Returns a list of directions and their associated costs:
        - Horizontal and vertical moves have a cost of 1.
        - Diagonal moves have a cost of sqrt(2).
        """
        return [
            (1, 0, 1),  # Right (cost 1)
            (0, 1, 1),  # Up (cost 1)
            (-1, 0, 1),  # Left (cost 1)
            (0, -1, 1),  # Down (cost 1)
            (-1, -1, math.sqrt(2)),  # Diagonal left-down (cost sqrt(2))
            (-1, 1, math.sqrt(2)),  # Diagonal left-up (cost sqrt(2))
            (1, -1, math.sqrt(2)),  # Diagonal right-down (cost sqrt(2))
            (1, 1, math.sqrt(2)),  # Diagonal right-up (cost sqrt(2))
        ]

    def d_star_planning(
        self, sx, sy, gx, gy, dynamic_visualization=False, pause_time=0.0001
    ):
        """
        Perform D* path planning with dynamic visualization.

        Arguments:
        - sx, sy: Start position.
        - gx, gy: Goal position.
        - dynamic_visualization: Boolean flag to enable step-by-step visualization.
        - pause_time: Time interval between visualization steps.
        """
        start_time = time.time()  # Start the timer for performance tracking.

        start = (int(sx), int(sy))  # Start node.
        goal = (int(gx), int(gy))  # Goal node.

        # Check if start or goal is invalid (not present in the graph).
        if start not in self.graph or goal not in self.graph:
            print("Start or goal node is invalid!")
            return [], []

        # Set up visualization using matplotlib.
        fig, ax = plt.subplots(figsize=(10, 10))
        pos = {node: (node[0], node[1]) for node in self.graph.nodes}  # Node positions.
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

        # Save the original axis limits for later restoration.
        original_xlim = ax.get_xlim()
        original_ylim = ax.get_ylim()

        # Perform D* Search to find the path.
        try:
            path, explored_nodes = self.d_star_search(start, goal)
            print("Final Path Nodes:")
        except nx.NetworkXNoPath:
            print("No path found!")
            plt.title("No Path Found")
            plt.show()
            return [], []

        # Optionally visualize the explored nodes dynamically.
        if dynamic_visualization:
            for node in explored_nodes:
                ax.scatter(node[0], node[1], color="red", s=20)
                plt.pause(pause_time)

        # Ensure that all dynamic elements remain on the plot.
        plt.draw()

        # Plot the final path.
        if path and len(path) > 1:
            rx, ry = zip(*path)
            ax.plot(rx, ry, color="red", label="Final Path", linewidth=2, zorder=10)
            print("Final Path Plotted Successfully.")
        else:
            print("No valid path found to plot.")

        # Restore the original zoom level after plotting.
        ax.set_xlim(original_xlim)
        ax.set_ylim(original_ylim)

        # Metrics
        path_length = self.calculate_path_length(path)
        execution_time = time.time() - start_time
        direction_changes = self.calculate_direction_changes(path)

        # Print out relevant metrics.
        print(f"Execution Time: {execution_time:.2f} seconds")
        print(f"Path Length: {path_length:.2f}")
        print(f"Steps Taken: {len(path)}")
        print(f"Direction Changes: {direction_changes}")

        plt.title("D* Path Planning")
        plt.legend()
        plt.show()

        return rx, ry

    def d_star_search(self, start, goal):
        """
        Perform the D* search algorithm to find the path from start to goal.

        Arguments:
        - start: Start node.
        - goal: Goal node.

        Returns:
        - path: The optimal path from start to goal.
        - explored_nodes: Set of nodes that were explored during the search.
        """
        explored_nodes = set()

        try:
            # Use A* to find the path from start to goal.
            path = nx.astar_path(
                self.graph, start, goal, heuristic=self.calc_heuristic, weight="weight"
            )
            explored_nodes.update(path)  # Mark the nodes that were explored.
            return path, explored_nodes
        except nx.NetworkXNoPath:
            raise nx.NetworkXNoPath("No path found!")

    @staticmethod
    def calc_heuristic(n1, n2):
        """
        Calculate the Euclidean distance as a heuristic between two nodes.

        Arguments:
        - n1, n2: The two nodes (x, y).

        Returns:
        - The Euclidean distance between the nodes.
        """
        return math.hypot(n1[0] - n2[0], n1[1] - n2[1])

    @staticmethod
    def calculate_path_length(path):
        """
        Calculate the total length of the path.

        Arguments:
        - path: List of nodes in the path.

        Returns:
        - The total length of the path.
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
    Main function to execute the D* algorithm for pathfinding.
    Defines the start and goal positions, obstacles, and runs the D* algorithm.
    """
    sx, sy = 10.0, 10.0  # Start position (x, y).
    gx, gy = 50.0, 50.0  # Goal position (x, y).
    grid_size = 1.0  # Grid resolution (spacing between nodes).
    robot_radius = 2.0  # Robot size (radius used for obstacle clearance).

    # Define obstacle positions (creating a boundary and some interior obstacles).
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

    # Initialize D* algorithm with the defined obstacles and robot radius.
    d_star = DStarAlgorithm(x_obs, y_obs, grid_size, robot_radius)

    # Run the D* algorithm to plan the path with visualization.
    rx, ry = d_star.d_star_planning(
        sx, sy, gx, gy, dynamic_visualization=True, pause_time=0.001
    )


if __name__ == "__main__":
    main()
