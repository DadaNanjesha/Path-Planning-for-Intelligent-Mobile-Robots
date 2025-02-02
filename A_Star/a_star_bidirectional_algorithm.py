import math
import time
import networkx as nx
import matplotlib.pyplot as plt


class BidirectionalAStarAlgorithm:

    def __init__(self, x_obs, y_obs, resol, radius):
        """
        Initialize the Bidirectional A* algorithm with obstacles and parameters.
        Arguments:
        - x_obs, y_obs: Lists of x and y coordinates of the obstacles.
        - resol: The grid resolution (i.e., the spacing between nodes in the grid).
        - radius: The radius of the robot (used for obstacle clearance).
        """
        self.resol = resol  # Grid resolution.
        self.radius = radius  # Radius of the robot.
        self.graph = nx.Graph()  # Graph representation of the grid.
        self.obstacles = set()  # Set of obstacle nodes.
        self.motion = (
            self.motion_mod()
        )  # Predefined motion vectors for 8-connected grid.
        self.build_graph(x_obs, y_obs)  # Build the graph with obstacles.

    def build_graph(self, x_obs, y_obs):
        """
        Build the graph with nodes, edges, and obstacles.
        The graph consists of nodes and edges representing free space,
        with obstacles removed and no connections between blocked nodes.
        Arguments:
        - x_obs, y_obs: Lists of x and y coordinates of obstacles.
        """
        # Determine the bounding box for the grid based on the obstacles' coordinates.
        self.x_min = round(min(x_obs))  # Minimum x-coordinate.
        self.y_min = round(min(y_obs))  # Minimum y-coordinate.
        self.max_x = round(max(x_obs))  # Maximum x-coordinate.
        self.max_y = round(max(y_obs))  # Maximum y-coordinate.

        # Add nodes to the graph for each point within the bounding box.
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
                    # If a node is within the obstacle's radius, remove it from the graph.
                    if math.hypot(ox - x, oy - y) <= self.radius:
                        obstacle_node = (x, y)
                        if obstacle_node in self.graph:
                            self.obstacles.add(obstacle_node)
                            self.graph.remove_node(obstacle_node)

        # Connect nodes based on valid motions (8-connected grid).
        for node in list(self.graph.nodes):
            for dx, dy, cost in self.motion:
                neighbor = (node[0] + dx, node[1] + dy)
                if neighbor in self.graph:
                    self.graph.add_edge(node, neighbor, weight=cost)

    @staticmethod
    def motion_mod():
        """
        Define motion vectors for 8-connected grid.
        Returns a list of possible directions and their associated costs.
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

    def bidirectional_astar(
        self, sx, sy, gx, gy, dynamic_visualization=False, pause_time=0.2
    ):
        """
        Perform Bidirectional A* pathfinding with dynamic visualization.
        Arguments:
        - sx, sy: Start position.
        - gx, gy: Goal position.
        - dynamic_visualization: Boolean flag to enable step-by-step visualization.
        - pause_time: Time interval for each visualization step.
        """
        start_time = time.time()  # Start timer for performance tracking.

        start = (int(sx), int(sy))  # Start coordinates.
        goal = (int(gx), int(gy))  # Goal coordinates.

        # Check if the start or goal is invalid (not in the graph).
        if start not in self.graph or goal not in self.graph:
            print("Start or goal node is invalid!")
            return [], []

        # Set up the visualization using matplotlib.
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

        # Save the Original Axis Limits to prevent zooming during dynamic updates.
        original_xlim = ax.get_xlim()
        original_ylim = ax.get_ylim()

        # Perform bidirectional search to find the path.
        try:
            path, explored_start, explored_goal = self.bidirectional_search(start, goal)
            print("Final Path Nodes:")
        except nx.NetworkXNoPath:
            print("No path found!")
            plt.title("No Path Found")
            plt.show()
            return [], []

        # Visualize the exploration process dynamically (optional).
        if dynamic_visualization:
            batch_size = 10  # Number of nodes to visualize at once.
            for i in range(0, len(explored_start), batch_size):
                batch = list(explored_start)[i : i + batch_size]
                ax.scatter(
                    *zip(*batch), color="cyan", s=20
                )  # Visualize start-side explored nodes.
                plt.pause(pause_time)

            for i in range(0, len(explored_goal), batch_size):
                batch = list(explored_goal)[i : i + batch_size]
                ax.scatter(
                    *zip(*batch), color="magenta", s=20
                )  # Visualize goal-side explored nodes.
                plt.pause(pause_time)

        plt.draw()  # Ensure previous elements remain on the plot.

        # Plot the final path, if a valid path is found.
        if path and len(path) > 1:
            rx, ry = zip(*path)
            ax.plot(rx, ry, color="red", label="Final Path", linewidth=2, zorder=10)
            print("Final Path Plotted Successfully.")
        else:
            print("No valid path found to plot.")

        # Restore the original zoom to prevent unwanted zooming.
        ax.set_xlim(original_xlim)
        ax.set_ylim(original_ylim)

        # Metrics: Calculate path length and execution time.
        path_length = self.calculate_path_length(path)
        execution_time = time.time() - start_time

        print(f"Execution Time: {execution_time:.2f} seconds")
        print(f"Path Length: {path_length:.2f}")
        print(f"Steps Taken: {len(path)}")

        plt.title("Bidirectional A* Path Planning")
        plt.legend()
        plt.show()

        return rx, ry

    def bidirectional_search(self, start, goal):
        """
        Perform bidirectional search from start to goal.
        Arguments:
        - start: Start node.
        - goal: Goal node.
        Returns:
        - path: The reconstructed path from start to goal.
        - explored_start: Explored nodes from the start.
        - explored_goal: Explored nodes from the goal.
        """
        open_set_start = {start}  # Start open set.
        open_set_goal = {goal}  # Goal open set.
        closed_set_start = set()  # Closed set for start.
        closed_set_goal = set()  # Closed set for goal.

        parents_start = {start: None}  # Parent pointers for start.
        parents_goal = {goal: None}  # Parent pointers for goal.

        while open_set_start and open_set_goal:
            # Expand the start side of the search.
            current_start = min(
                open_set_start, key=lambda node: self.calc_heuristic(node, goal)
            )
            open_set_start.remove(current_start)
            closed_set_start.add(current_start)

            # Expand the goal side of the search.
            current_goal = min(
                open_set_goal, key=lambda node: self.calc_heuristic(node, start)
            )
            open_set_goal.remove(current_goal)
            closed_set_goal.add(current_goal)

            # Check for path intersection.
            if current_start in closed_set_goal:
                return (
                    self.reconstruct_path(current_start, parents_start, parents_goal),
                    closed_set_start,
                    closed_set_goal,
                )
            if current_goal in closed_set_start:
                return (
                    self.reconstruct_path(current_goal, parents_start, parents_goal),
                    closed_set_start,
                    closed_set_goal,
                )

            # Expand neighbors from the start side.
            for neighbor in self.graph.neighbors(current_start):
                if neighbor not in closed_set_start:
                    open_set_start.add(neighbor)
                    parents_start.setdefault(neighbor, current_start)

            # Expand neighbors from the goal side.
            for neighbor in self.graph.neighbors(current_goal):
                if neighbor not in closed_set_goal:
                    open_set_goal.add(neighbor)
                    parents_goal.setdefault(neighbor, current_goal)

        raise nx.NetworkXNoPath

    def reconstruct_path(self, meeting_node, parents_start, parents_goal):
        """
        Reconstruct the path by tracing back from the meeting point.
        Arguments:
        - meeting_node: The node where the two search fronts met.
        - parents_start, parents_goal: Parent mappings from the start and goal searches.
        Returns:
        - The reconstructed path.
        """
        path_start = []
        current = meeting_node
        while current is not None:
            path_start.append(current)
            current = parents_start[current]

        path_goal = []
        current = meeting_node
        while current is not None:
            path_goal.append(current)
            current = parents_goal[current]

        path_start.reverse()  # Reverse start path to get the correct order.
        return path_start + path_goal[1:]

    @staticmethod
    def calc_heuristic(n1, n2):
        """
        Calculate the Euclidean distance between two nodes as the heuristic.
        Arguments:
        - n1, n2: Two nodes (x, y) coordinates.
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


def main():
    """
    Main function to execute Bidirectional A* algorithm.
    Defines the start and goal positions, obstacles, and runs the Bidirectional A* algorithm.
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

    # Initialize the Bidirectional A* algorithm with the defined obstacles and robot radius.
    bidir_astar = BidirectionalAStarAlgorithm(x_obs, y_obs, grid_size, robot_radius)

    # Run the Bidirectional A* algorithm to find the path with visualization.
    rx, ry = bidir_astar.bidirectional_astar(
        sx, sy, gx, gy, dynamic_visualization=True, pause_time=0.2
    )


if __name__ == "__main__":
    main()
