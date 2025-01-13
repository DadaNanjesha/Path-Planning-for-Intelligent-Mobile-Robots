import math
import matplotlib.pyplot as plt

show_animation = True


# This class likely implements the A* algorithm for path finding.
class AStarAlgorithm:

    def __init__(self, x_obs, y_obs, resol, radius):
        """
        The function initializes a grid map for A* path planning with given obstacle positions and
        parameters.

        :param x_obs: The `x_obs` parameter is a list containing the x positions of obstacles in meters.
        :param y_obs: The `y_obs` parameter is the list of y positions of obstacles in meters. It is
            used to initialize the grid map for a star planning algorithm.
        :param resolution: The `resolution` parameter in the `__init__` function represents the grid
            resolution, which is the distance between each grid cell in the map. It determines the level of
            detail in the map representation and affects the accuracy of path planning and obstacle
            avoidance algorithms. A smaller resolution value results in a higher.
        :param radius: The `radius` parameter in the `__init__` function represents the radius of the
            robot in meters. This value is used in path planning algorithms to ensure that the robot can
            safely navigate around obstacles without colliding with them
        """
        self.x_min, self.y_min = 0, 0
        self.max_x, self.max_y = 0, 0
        self.resol = resol
        self.radius = radius
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.motion_mod()
        self.obstacle_map_calculation(x_obs, y_obs)

    # This class is likely used to define nodes in a data structure like a linked list or tree.
    class Node:

        def __init__(self, x, y, cost, parent_index):
            """
            This function initializes an object with attributes for grid indices, cost, and parent
            index.

            :param x: The parameter `x` represents the index of the grid. It is used to store the
                x-coordinate or column index of a grid cell in a grid-based algorithm or data structure
            :param y: The `y` parameter represents the index of the grid in the `__init__` method you
                provided. It is used to store the y-coordinate of a grid location
            :param cost: The `cost` parameter in the `__init__` method represents the cost associated
                with reaching the current grid cell from the starting point. This cost could be based on
                various factors such as distance, terrain difficulty, or any other metric relevant to the
                problem you are solving
            :param parent_index: The `parent_index` parameter in the `__init__` method is used to store
                the index of the parent node in a graph or grid structure. This is commonly used in
                algorithms like A* search or Dijkstra's algorithm to keep track of the path from the start
                node to the current node
            """
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return (
                str(self.x)
                + ","
                + str(self.y)
                + ","
                + str(self.cost)
                + ","
                + str(self.parent_index)
            )

    def planning(self, sx, sy, gx, gy):
        """
        The function `planning` implements the A* algorithm to find a path from a start node to a goal
        node on a grid.

        :param sx: The parameter `sx` in the `planning` function represents the x-coordinate of the
            start position for path planning. It is used to create the start node for the pathfinding
            algorithm
        :param sy: The parameter `sy` in the `planning` function represents the y-coordinate of the
            starting point in a path planning algorithm. It is used to specify the initial y-coordinate of
            the robot or object for which the path needs to be planned
        :param gx: The parameter `gx` in the `planning` function represents the x-coordinate of the goal
            position in the grid. It is used to create a goal node for path planning from the start position
            (sx, sy) to the goal position (gx, gy) on the grid
        :param gy: The parameter `gy` in the `planning` function represents the y-coordinate of the goal
            position in a grid-based path planning algorithm. It is used to specify the y-coordinate of the
            goal node where the path planning algorithm will try to navigate the robot or agent to
        :return: The `planning` function returns the final path `rx` and `ry` after finding the goal
            node in the search grid.
        """

        start_node = self.Node(
            self.calc_xy_index(sx, self.x_min),
            self.calc_xy_index(sy, self.y_min),
            0.0,
            -1,
        )
        goal_node = self.Node(
            self.calc_xy_index(gx, self.x_min),
            self.calc_xy_index(gy, self.y_min),
            0.0,
            -1,
        )

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost
                + self.calc_heuristic(goal_node, open_set[o]),
            )
            current = open_set[c_id]

            if show_animation:
                plt.plot(
                    self.calc_grid_position(current.x, self.x_min),
                    self.calc_grid_position(current.y, self.y_min),
                    "xc",
                )
                plt.gcf().canvas.mpl_connect(
                    "key_release_event",
                    lambda event: [exit(0) if event.key == "escape" else None],
                )
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]

            closed_set[c_id] = current

            for i, _ in enumerate(self.motion):
                node = self.Node(
                    current.x + self.motion[i][0],
                    current.y + self.motion[i][1],
                    current.cost + self.motion[i][2],
                    c_id,
                )
                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        """
        The function calculates the final path from the start node to the goal node based on the closed
        set of nodes.

        :param goal_node: The `calc_final_path` method takes in three parameters: `self`, `goal_node`,
            and `closed_set`
        :param closed_set: The `closed_set` parameter in the `calc_final_path` method is a set of nodes
            that have already been evaluated during the path finding process. These nodes are considered
            closed and will not be revisited. The method uses the `closed_set` to trace back the path from
            the goal node to
        :return: The function `calc_final_path` returns two lists, `rx` and `ry`, which contain the x
            and y grid positions of the nodes in the path from the goal node to the start node.
        """

        rx, ry = [self.calc_grid_position(goal_node.x, self.x_min)], [
            self.calc_grid_position(goal_node.y, self.y_min)
        ]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.x_min))
            ry.append(self.calc_grid_position(n.y, self.y_min))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        """
        This Python function calculates the heuristic distance between two points using the Euclidean
        distance formula.

        :param n1: It seems like you were about to provide some information about the parameter `n1`,
            but the message got cut off. Could you please provide more details about what `n1` represents in
            this context?
        :param n2: It seems like you forgot to provide the details for the parameter `n2`. Could you
            please provide more information about `n2` so that I can assist you further with the
            `calc_heuristic` function?
        :return: The function `calc_heuristic` is returning the Euclidean distance between two points
            `n1` and `n2` multiplied by the weight `w`.
        """

        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        The function calculates the position of an element in a grid based on its index and minimum
        position.

        :param index: The `index` parameter represents the position index in the grid. It is used to
            calculate the final position based on the resolution and the minimum position
        :param min_position: The `min_position` parameter in the `calc_grid_position` function
            represents the minimum position value that you want to start calculating the grid position from.
            This value is used in the calculation to determine the final position based on the index and
            resolution
        :return: the calculated position based on the index, resolution, and minimum position provided
            as input arguments.
        """
        pos = index * self.resol + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        """
        The function calculates the index of a position based on a minimum position and resolution.

        :param position: The `position` parameter represents the current position value for which you
            want to calculate the index
        :param min_pos: The `min_pos` parameter represents the minimum position value in the range
        :return: the index of a position calculated based on the minimum position and resolution.
        """

        return round((position - min_pos) / self.resol)

    def calc_grid_index(self, node):
        """
        This function calculates the index of a node in a grid based on its coordinates and the grid
        dimensions.

        :param node: The `node` parameter represents a point in a grid. It likely has `x` and `y`
            coordinates that specify its position within the grid
        :return: The function `calc_grid_index` is returning the index of a node in a grid based on its
            x and y coordinates relative to the minimum x and y values of the grid.
        """
        return (node.y - self.y_min) * self.x_width + (node.x - self.x_min)

    def verify_node(self, node):
        """
        The function `verify_node` checks if a given node is within the grid boundaries and not
        colliding with obstacles.

        :param node: The `node` parameter in the `verify_node` method represents a node in a grid. The
            method calculates the grid position of the node based on its x and y coordinates and then
            performs various checks to verify if the node is within the grid boundaries and does not collide
            with any obstacles in the obstacle
        :return: a boolean value - either True or False.
        """
        px = self.calc_grid_position(node.x, self.x_min)
        py = self.calc_grid_position(node.y, self.y_min)

        if px < self.x_min:
            return False
        elif py < self.y_min:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def obstacle_map_calculation(self, x_obs, y_obs):
        """
        The function calculates an obstacle map based on given obstacle coordinates and radius.

        :param x_obs: It seems like the `x_obs` parameter represents a list of x-coordinates of
        `obstacles in your environment. You can pass a list of x-coordinates of obstacles to the
        ``obstacle_map_calculation` method to calculate the obstacle map based on these coordinates
        :param y_obs: The `y_obs` parameter in the `obstacle_map_calculation` function represents the
            y-coordinates of the obstacles in your environment. These y-coordinates are used along with the
            x-coordinates (`x_obs`) to calculate the obstacle map for path planning or obstacle avoidance
            algorithms. The function calculates
        """

        self.x_min = round(min(x_obs))
        self.y_min = round(min(y_obs))
        self.max_x = round(max(x_obs))
        self.max_y = round(max(y_obs))
        print("x_min:", self.x_min)
        print("y_min:", self.y_min)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.x_min) / self.resol)
        self.y_width = round((self.max_y - self.y_min) / self.resol)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [
            [False for _ in range(self.y_width)] for _ in range(self.x_width)
        ]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.x_min)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.y_min)
                for iox, ioy in zip(x_obs, y_obs):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def motion_mod():
        """
        The function `motion_mod` returns a list of motion vectors with corresponding magnitudes.
        :return: The function `motion_mod()` returns a list of motion vectors. Each motion vector is
            represented as a list containing three elements: the change in x-coordinate, the change in
            y-coordinate, and the distance covered by that motion vector.
        """
        motion = [
            [1, 0, 1],
            [0, 1, 1],
            [-1, 0, 1],
            [0, -1, 1],
            [-1, -1, math.sqrt(2)],
            [-1, 1, math.sqrt(2)],
            [1, -1, math.sqrt(2)],
            [1, 1, math.sqrt(2)],
        ]

        return motion


def main():
    """
    The main function sets up a grid-based path planning problem with obstacles and uses the A*
    algorithm to find a path from a start to a goal position.
    """
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 2.0  # [m]

    # set obstacle positions
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
    for i in range(-10, 30):
        for j in range(10, 20):
            x_obs.append(i)
            y_obs.append(13.0)
    for i in range(0, 40):
        x_obs.append(40.0)
        y_obs.append(60.0 - i)

    if show_animation:  # pragma: no cover
        plt.plot(x_obs, y_obs, ".r")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "ob")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarAlgorithm(x_obs, y_obs, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-b")
        plt.pause(0.001)
        plt.show()


if __name__ == "__main__":
    main()
