import math
import time
import heapq
import networkx as nx
import matplotlib.pyplot as plt


class ThetaStarAlgorithm:
    def __init__(self, x_obs, y_obs, resol, radius):
        """
        Initialize the Theta* algorithm with obstacles and parameters.
        - x_obs, y_obs: lists of obstacle coordinates.
        - resol: grid resolution.
        - radius: clearance (robot radius).
        """
        self.resol = resol
        self.radius = radius
        self.graph = nx.Graph()  # Create a graph to represent the grid
        self.obstacles = set()  # Set to store obstacles
        self.motion = self.motion_mod()  # Define motion model
        self.build_graph(x_obs, y_obs)  # Build the grid graph

    def build_graph(self, x_obs, y_obs):
        """
        Build the grid graph:
          - Create nodes for every (x,y) in the grid.
          - Remove nodes within the clearance (radius) of any obstacle.
          - Connect remaining nodes using an 8-connected motion model.
        """
        self.x_min = round(min(x_obs))  # Get minimum x-coordinate of obstacles
        self.y_min = round(min(y_obs))  # Get minimum y-coordinate of obstacles
        self.max_x = round(max(x_obs))  # Get maximum x-coordinate of obstacles
        self.max_y = round(max(y_obs))  # Get maximum y-coordinate of obstacles

        # Create grid nodes for each point (x, y) within the boundaries of obstacles
        nodes = [
            (x, y)
            for x in range(self.x_min, self.max_x + 1)
            for y in range(self.y_min, self.max_y + 1)
        ]
        self.graph.add_nodes_from(nodes)

        # Precompute squared radius to avoid redundant square root calculations
        r2 = self.radius**2

        # Remove nodes that are too close to any obstacles (based on the robot's clearance)
        for ox, oy in zip(x_obs, y_obs):
            min_x_obs = max(self.x_min, int(math.floor(ox - self.radius)))
            max_x_obs = min(self.max_x, int(math.ceil(ox + self.radius)))
            min_y_obs = max(self.y_min, int(math.floor(oy - self.radius)))
            max_y_obs = min(self.max_y, int(math.ceil(oy + self.radius)))
            for x in range(min_x_obs, max_x_obs + 1):
                for y in range(min_y_obs, max_y_obs + 1):
                    if (ox - x) ** 2 + (oy - y) ** 2 <= r2:
                        node = (x, y)
                        if node in self.graph:
                            self.obstacles.add(node)  # Add to obstacle set
                            self.graph.remove_node(node)  # Remove the obstacle node

        # Connect remaining nodes using allowed motions
        remaining_nodes = list(self.graph.nodes)
        for node in remaining_nodes:
            for dx, dy, cost in self.motion:
                neighbor = (node[0] + dx, node[1] + dy)
                if neighbor in self.graph:
                    self.graph.add_edge(node, neighbor, weight=cost)

    @staticmethod
    def motion_mod():
        """
        Define 8-connected motion vectors (dx, dy) and their cost.
        The motion model allows for vertical, horizontal, and diagonal moves.
        """
        return [
            (1, 0, 1),  # Right
            (0, 1, 1),  # Up
            (-1, 0, 1),  # Left
            (0, -1, 1),  # Down
            (-1, -1, math.sqrt(2)),  # Bottom-left diagonal
            (-1, 1, math.sqrt(2)),  # Top-left diagonal
            (1, -1, math.sqrt(2)),  # Bottom-right diagonal
            (1, 1, math.sqrt(2)),  # Top-right diagonal
        ]

    def line_of_sight(self, a, b):
        """
        Returns True if there is a clear line-of-sight between nodes a and b.
        Interpolates along the line and ensures each intermediate grid point exists.
        """
        x0, y0 = a
        x1, y1 = b
        dx = x1 - x0
        dy = y1 - y0
        steps = int(max(abs(dx), abs(dy)))  # Calculate steps for line interpolation
        if steps == 0:
            return True
        for i in range(steps + 1):
            t = i / steps
            x = round(x0 + t * dx)  # Interpolate x-coordinate
            y = round(y0 + t * dy)  # Interpolate y-coordinate
            if (
                x,
                y,
            ) not in self.graph:  # Check if the interpolated point is a valid node
                return False
        return True

    def calc_heuristic(self, n1, n2):
        """
        Euclidean distance heuristic to estimate the cost from node n1 to node n2.
        """
        return math.hypot(n1[0] - n2[0], n1[1] - n2[1])

    def cost(self, a, b):
        """
        Return the cost between adjacent nodes a and b.
        If an edge exists between the nodes, return the weight, otherwise calculate Euclidean distance.
        """
        if self.graph.has_edge(a, b):
            return self.graph[a][b]["weight"]
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def theta_star_search(self, start, goal):
        """
        Perform Theta* search from start to goal.
        Uses A* search with line-of-sight optimization.
        Returns the final path as a list of nodes.
        """
        open_set = []
        heapq.heappush(
            open_set, (self.calc_heuristic(start, goal), start)
        )  # Use heap for efficient priority queue
        came_from = {}
        g = {start: 0}  # Cost to reach each node from start
        came_from[start] = start  # The start's parent is itself.
        closed_set = set()  # Set to track visited nodes

        while open_set:
            current_f, current = heapq.heappop(
                open_set
            )  # Pop the node with the smallest f-value
            if current == goal:
                break  # Exit if we reached the goal
            closed_set.add(current)  # Mark the current node as visited
            for neighbor in self.graph.neighbors(
                current
            ):  # Explore neighbors of current node
                if neighbor in closed_set:
                    continue
                # Use line-of-sight optimization: check if we can shortcut using line-of-sight from parent to neighbor
                if current != start and self.line_of_sight(
                    came_from[current], neighbor
                ):
                    tentative_g = g[came_from[current]] + self.cost(
                        came_from[current], neighbor
                    )
                    parent_candidate = came_from[current]
                else:
                    tentative_g = g[current] + self.cost(current, neighbor)
                    parent_candidate = current
                if neighbor not in g or tentative_g < g[neighbor]:
                    g[neighbor] = tentative_g
                    f = tentative_g + self.calc_heuristic(
                        neighbor, goal
                    )  # f = g + h (heuristic)
                    came_from[neighbor] = parent_candidate
                    heapq.heappush(
                        open_set, (f, neighbor)
                    )  # Push the neighbor into the priority queue

        # If goal was never reached, return empty path.
        if goal not in came_from:
            return []

        # Reconstruct path from goal to start by backtracking through the came_from dictionary.
        path = [goal]
        current = goal
        while current != start:
            current = came_from[current]
            path.append(current)
        path.reverse()  # Reverse the path to start from the beginning
        return path

    def calculate_path_length(self, path):
        """
        Calculate total Euclidean path length.
        """
        return sum(
            math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
            for i in range(len(path) - 1)
        )

    def calculate_direction_changes(self, path):
        """
        Count number of direction changes in the path.
        """
        changes = 0
        for i in range(2, len(path)):
            prev_dir = (
                path[i - 1][0] - path[i - 2][0],
                path[i - 1][1] - path[i - 2][1],
            )
            curr_dir = (path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
            if prev_dir != curr_dir:  # If the direction changes
                changes += 1
        return changes

    def planning(self, sx, sy, gx, gy, dynamic_visualization=False):
        """
        Perform Theta* path planning from start (sx, sy) to goal (gx, gy).
        Displays the graph and path, prints metrics (Execution Time, Path Length,
        Steps Taken, Direction Changes) and adds a red dashed line connecting the
        start and goal directly (if unobstructed). When dynamic_visualization is
        True, each point is plotted with a 0.5 second pause, and then a continuous
        red line is drawn connecting the computed path.
        """
        start_time = time.time()
        start = (int(sx), int(sy))
        goal = (int(gx), int(gy))
        if start not in self.graph or goal not in self.graph:
            print("Start or goal node is invalid!")
            return [], []

        # Perform Theta* search.
        path = self.theta_star_search(start, goal)
        if not path:
            print("No path found!")
            plt.title("No Path Found")
            plt.show()
            return [], []

        execution_time = time.time() - start_time
        path_length = self.calculate_path_length(path)
        steps = len(path)
        direction_changes = self.calculate_direction_changes(path)

        print(f"Execution Time: {execution_time:.2f} seconds")
        print(f"Path Length: {path_length:.2f}")
        print(f"Steps Taken: {steps}")
        print(f"Direction Changes: {direction_changes}")

        # Set up the plot.
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
            ax.scatter(*zip(*self.obstacles), color="black", s=15, label="Obstacles")
        ax.scatter(start[0], start[1], color="blue", s=50, label="Start")
        ax.scatter(goal[0], goal[1], color="green", s=50, label="Goal")

        # If dynamic visualization is enabled, plot each node with a pause.
        if dynamic_visualization:
            for node in path:
                ax.scatter(node[0], node[1], color="red", s=20)
                plt.pause(0.5)  # 0.5 second pause for each node

        # After plotting the nodes, draw the continuous red line for the computed path.
        rx, ry = zip(*path)
        ax.plot(rx, ry, color="red", linewidth=2, label="Computed Path")

        # Check and add a red dashed line connecting start and goal if the direct connection is clear.
        if self.line_of_sight(start, goal):
            ax.plot(
                [start[0], goal[0]],
                [start[1], goal[1]],
                color="red",
                linewidth=2,
                linestyle="--",
                label="Direct Connection",
            )
        else:
            print(
                "Direct connection from start to goal is blocked by obstacles; not drawn."
            )

        plt.title("Theta* Path Planning")
        plt.legend()
        plt.show()
        return rx, ry


# =============================================================================
# Main Function
# =============================================================================
def main():
    # Define start and goal positions.
    sx, sy = 10.0, 10.0
    gx, gy = 50.0, 50.0
    grid_size = 1.0
    robot_radius = 2.0

    # Define obstacles.
    x_obs, y_obs = [], []
    # Boundary obstacles.
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

    theta_star = ThetaStarAlgorithm(x_obs, y_obs, grid_size, robot_radius)
    rx, ry = theta_star.planning(sx, sy, gx, gy, dynamic_visualization=True)


if __name__ == "__main__":
    main()
