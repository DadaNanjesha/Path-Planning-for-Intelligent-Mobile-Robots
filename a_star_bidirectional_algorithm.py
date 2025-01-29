import math
import time
import networkx as nx
import matplotlib.pyplot as plt


class BidirectionalAStarAlgorithm:

    def __init__(self, x_obs, y_obs, resol, radius):
        """
        Initialize the Bidirectional A* algorithm with obstacles and parameters.
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

    def bidirectional_astar(
        self, sx, sy, gx, gy, dynamic_visualization=False, pause_time=0.2
    ):
        """
        Perform Bidirectional A* pathfinding with dynamic visualization.
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

        # Save the Original Axis Limits
        original_xlim = ax.get_xlim()
        original_ylim = ax.get_ylim()

        # Perform Bidirectional Search
        try:
            path, explored_start, explored_goal = self.bidirectional_search(start, goal)
            print("Final Path Nodes:", path)
        except nx.NetworkXNoPath:
            print("No path found!")
            plt.title("No Path Found")
            plt.show()
            return [], []

        # Dynamic visualization of explored nodes
        if dynamic_visualization:
            batch_size = 10
            for i in range(0, len(explored_start), batch_size):
                batch = list(explored_start)[i : i + batch_size]
                ax.scatter(*zip(*batch), color="cyan", s=20)
                plt.pause(pause_time)

            for i in range(0, len(explored_goal), batch_size):
                batch = list(explored_goal)[i : i + batch_size]
                ax.scatter(*zip(*batch), color="magenta", s=20)
                plt.pause(pause_time)

        plt.draw()  # Ensures previous elements remain

        # Plot the final path
        if path and len(path) > 1:
            rx, ry = zip(*path)
            ax.plot(rx, ry, color="red", label="Final Path", linewidth=2, zorder=10)
            print("Final Path Plotted Successfully.")
        else:
            print("No valid path found to plot.")

        # Restore Original Zoom to Prevent Unwanted Zooming
        ax.set_xlim(original_xlim)
        ax.set_ylim(original_ylim)

        # Metrics
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
        open_set_start = {start}
        open_set_goal = {goal}
        closed_set_start = set()
        closed_set_goal = set()

        parents_start = {start: None}
        parents_goal = {goal: None}

        while open_set_start and open_set_goal:
            current_start = min(
                open_set_start, key=lambda node: self.calc_heuristic(node, goal)
            )
            open_set_start.remove(current_start)
            closed_set_start.add(current_start)

            current_goal = min(
                open_set_goal, key=lambda node: self.calc_heuristic(node, start)
            )
            open_set_goal.remove(current_goal)
            closed_set_goal.add(current_goal)

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

            for neighbor in self.graph.neighbors(current_start):
                if neighbor not in closed_set_start:
                    open_set_start.add(neighbor)
                    parents_start.setdefault(neighbor, current_start)

            for neighbor in self.graph.neighbors(current_goal):
                if neighbor not in closed_set_goal:
                    open_set_goal.add(neighbor)
                    parents_goal.setdefault(neighbor, current_goal)

        raise nx.NetworkXNoPath

    def reconstruct_path(self, meeting_node, parents_start, parents_goal):
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

        path_start.reverse()
        return path_start + path_goal[1:]

    @staticmethod
    def calc_heuristic(n1, n2):
        return math.hypot(n1[0] - n2[0], n1[1] - n2[1])

    @staticmethod
    def calculate_path_length(path):
        return sum(
            math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
            for i in range(len(path) - 1)
        )


def main():
    """
    Main function to execute Bidirectional A* algorithm.
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

    # Initialize Bidirectional A* algorithm
    bidir_astar = BidirectionalAStarAlgorithm(x_obs, y_obs, grid_size, robot_radius)

    # Run the pathfinding algorithm with visualization
    rx, ry = bidir_astar.bidirectional_astar(
        sx, sy, gx, gy, dynamic_visualization=True, pause_time=0.2
    )


if __name__ == "__main__":
    main()
