import math
import time
import matplotlib.pyplot as plt

show_animation = True


# This class likely implements a bidirectional A* path planning algorithm in Python.
class BidirectionalAStarPlanner:

    def __init__(self, x_obs, y_obs, resol, radius):
        """
        The function initializes variables related to obstacle map calculation and motion.

        :param x_obs: It looks like the code snippet you provided is a part of a class initialization
            method. The parameters `x_obs`, `y_obs`, `resol`, and `radius` are being passed to the
            `__init__` method of the class
        :param y_obs: It seems like you were about to provide information about the `y_obs` parameter
            but the text got cut off. Could you please provide more details or let me know how I can assist
            you further with this code snippet?
        :param resol: The `resol` parameter in the `__init__` method likely represents the resolution or
            granularity of the map or grid being used in the context of path planning or obstacle avoidance.
            It determines the size of each grid cell or step size used for planning paths or checking for
            obstacles. A smaller resolution
        :param radius: The `radius` parameter in the `__init__` method likely refers to the radius of
            the robot or the minimum distance that the robot needs to maintain from obstacles in the
            environment. This parameter is used in path planning algorithms to ensure that the robot can
            navigate through the environment without colliding with obstacles
        """
        self.min_x, self.min_y = None, None
        self.max_x, self.max_y = None, None
        self.x_width, self.y_width, self.obstacle_map = None, None, None
        self.resol = resol
        self.radius = radius
        self.obstacle_map_calculation(x_obs, y_obs)
        self.motion = self.motion_mod()

    # The above class defines a Node for a data structure.
    class Node:
        def __init__(self, x, y, cost, parent_index):
            """
            The function initializes an object with x and y coordinates, cost, and parent index.

            :param x: The parameter `x` represents the index of the grid. It is used to store the
                x-coordinate or column index of a grid cell in a grid-based algorithm or data structure
            :param y: The `y` parameter represents the index of the grid in the y-axis. It is used to
                specify the vertical position of a point or node in a grid or coordinate system
            :param cost: The `cost` parameter in the `__init__` method represents the cost associated
                with reaching the current grid cell from the starting point. This cost could be based on
                various factors such as distance, terrain difficulty, or any other metric relevant to the
                problem you are solving. It is used in path
            :param parent_index: The `parent_index` parameter in the `__init__` method is used to store
                the index of the parent node in a grid or graph structure. This is commonly used in
                pathfinding algorithms like A* to keep track of the path from the start node to the current
                node. By storing the
            """
            self.x = x  # index of grid
            self.y = y  # index of grid
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
        start_time = time.time()  # Track execution time
        path_length = 0
        direction_changes = 0
        steps = 0

        start_node = self.Node(
            self.calc_xy_index(sx, self.min_x),
            self.calc_xy_index(sy, self.min_y),
            0.0,
            -1,
        )
        goal_node = self.Node(
            self.calc_xy_index(gx, self.min_x),
            self.calc_xy_index(gy, self.min_y),
            0.0,
            -1,
        )

        open_set_A, closed_set_A = dict(), dict()
        open_set_B, closed_set_B = dict(), dict()
        open_set_A[self.calc_grid_index(start_node)] = start_node
        open_set_B[self.calc_grid_index(goal_node)] = goal_node

        current_A = start_node
        current_B = goal_node
        meet_point_A, meet_point_B = None, None

        while True:
            steps += 1
            if len(open_set_A) == 0 or len(open_set_B) == 0:
                print("No path found.")
                break

            c_id_A = min(
                open_set_A, key=lambda o: self.find_total_cost(open_set_A, o, current_B)
            )
            current_A = open_set_A[c_id_A]

            c_id_B = min(
                open_set_B, key=lambda o: self.find_total_cost(open_set_B, o, current_A)
            )
            current_B = open_set_B[c_id_B]

            if show_animation:
                plt.plot(
                    self.calc_grid_position(current_A.x, self.min_x),
                    self.calc_grid_position(current_A.y, self.min_y),
                    "xc",
                )
                plt.plot(
                    self.calc_grid_position(current_B.x, self.min_x),
                    self.calc_grid_position(current_B.y, self.min_y),
                    "xc",
                )
                plt.pause(0.001)

            if current_A.x == current_B.x and current_A.y == current_B.y:
                print("Path found.")
                meet_point_A = current_A
                meet_point_B = current_B
                break

            del open_set_A[c_id_A]
            del open_set_B[c_id_B]
            closed_set_A[c_id_A] = current_A
            closed_set_B[c_id_B] = current_B

            for i, _ in enumerate(self.motion):
                c_nodes = [
                    self.Node(
                        current_A.x + self.motion[i][0],
                        current_A.y + self.motion[i][1],
                        current_A.cost + self.motion[i][2],
                        c_id_A,
                    ),
                    self.Node(
                        current_B.x + self.motion[i][0],
                        current_B.y + self.motion[i][1],
                        current_B.cost + self.motion[i][2],
                        c_id_B,
                    ),
                ]

                n_ids = [
                    self.calc_grid_index(c_nodes[0]),
                    self.calc_grid_index(c_nodes[1]),
                ]

                continue_ = self.check_nodes_and_sets(
                    c_nodes, closed_set_A, closed_set_B, n_ids
                )

                if not continue_[0]:
                    if n_ids[0] not in open_set_A:
                        open_set_A[n_ids[0]] = c_nodes[0]
                    else:
                        if open_set_A[n_ids[0]].cost > c_nodes[0].cost:
                            open_set_A[n_ids[0]] = c_nodes[0]

                if not continue_[1]:
                    if n_ids[1] not in open_set_B:
                        open_set_B[n_ids[1]] = c_nodes[1]
                    else:
                        if open_set_B[n_ids[1]].cost > c_nodes[1].cost:
                            open_set_B[n_ids[1]] = c_nodes[1]

        rx, ry = self.calc_final_bidirectional_path(
            meet_point_A, meet_point_B, closed_set_A, closed_set_B
        )

        # Calculate path metrics
        if rx and ry:
            for i in range(1, len(rx)):
                path_length += math.hypot(rx[i] - rx[i - 1], ry[i] - ry[i - 1])
                if i > 1:
                    prev_dir = (rx[i - 1] - rx[i - 2], ry[i - 1] - ry[i - 2])
                    curr_dir = (rx[i] - rx[i - 1], ry[i] - ry[i - 1])
                    if prev_dir != curr_dir:
                        direction_changes += 1

        execution_time = time.time() - start_time

        print(f"Execution Time: {execution_time:.2f} seconds")
        print(f"Path Length: {path_length:.2f}")
        print(f"Steps Taken: {steps}")
        print(f"Direction Changes: {direction_changes}")

        return rx, ry

    # takes two sets and two meeting nodes and return the optimal path
    def calc_final_bidirectional_path(self, n1, n2, setA, setB):
        """
        The function `calc_final_bidirectional_path` calculates the final bidirectional path by
        combining the final paths from two sets of nodes.

        :param n1: The `calc_final_bidirectional_path` method seems to calculate a bidirectional path by
            combining the final paths from two sets of nodes `setA` and `setB`, starting from nodes `n1` and
            `n2` respectively
        :param n2: n2 is a parameter representing a node or point in a graph. It is used as one of the
            inputs for calculating the final bidirectional path in the given code snippet
        :param setA: It seems like your message got cut off. Could you please provide more information
            about the `setA` parameter so that I can assist you further with the
            `calc_final_bidirectional_path` function?
        :param setB: It seems like you were about to provide some information about the `setB` parameter
            but the message got cut off. Could you please provide more details or let me know if you need
            help with something specific related to the `setB` parameter?
        :return: The function `calc_final_bidirectional_path` returns two lists `rx` and `ry`, which are
            the concatenated final paths from node `n1` to the set `setA` and from node `n2` to the set
            `setB`, respectively.
        """

        rx_A, ry_A = self.calc_final_path(n1, setA)
        rx_B, ry_B = self.calc_final_path(n2, setB)

        rx_A.reverse()
        ry_A.reverse()

        rx = rx_A + rx_B
        ry = ry_A + ry_B

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        """
        The function calculates the final path from the start node to the goal node using the closed set
        of nodes.

        :param goal_node: The `calc_final_path` method you provided seems to be a part of a pathfinding
            algorithm implementation. It calculates the final path from the goal node back to the start node
            using the closed set of nodes
        :param closed_set: The `closed_set` parameter in the `calc_final_path` method is a set of nodes
            that have been evaluated during the search process and have been closed (i.e., their neighbors
            have been explored). This set typically contains nodes that are part of the final path from the
            start node to the goal
        :return: two lists, `rx` and `ry`, which contain the x and y grid positions of the nodes in the
            path from the goal node to the start node.
        """
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)
        ]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def check_nodes_and_sets(self, c_nodes, closedSet_A, closedSet_B, n_ids):
        """
        The function `check_nodes_and_sets` checks if nodes meet certain conditions and if their IDs are
        in specified sets before continuing.

        :param c_nodes: The `c_nodes` parameter likely represents a list of two nodes that are being
            checked in the function
        :param closedSet_A: closedSet_A is a set containing nodes that have already been evaluated in a
            pathfinding algorithm for group A
        :param closedSet_B: The `closedSet_B` parameter in the `check_nodes_and_sets` function likely
            represents a set of nodes that have already been visited or processed in some way. This set is
            used to keep track of nodes that should not be revisited or reprocessed during the execution of
            the algorithm
        :param n_ids: The `n_ids` parameter likely represents a list of node IDs. In the
            `check_nodes_and_sets` method, the node IDs are being checked against the closed sets
            `closedSet_A` and `closedSet_B`. If a node ID is found in the respective closed set or if the
            corresponding
        :return: The function `check_nodes_and_sets` returns a list `continue_` containing two boolean
            values.
        """

        continue_ = [False, False]
        if not self.verify_node(c_nodes[0]) or n_ids[0] in closedSet_A:
            continue_[0] = True

        if not self.verify_node(c_nodes[1]) or n_ids[1] in closedSet_B:
            continue_[1] = True

        return continue_

    @staticmethod
    def calc_heuristic(n1, n2):
        """
        The function calculates the heuristic distance between two points using the Euclidean distance
        formula with a specified weight.

        :param n1: It seems like you were about to provide some information about the parameter `n1`,
            but the message got cut off. Could you please provide more details about what `n1` represents in
            your code?
        :param n2: It seems like you have provided the code snippet for calculating the heuristic
            distance between two nodes `n1` and `n2`. However, you have not provided the definition or
            values for the `n2` node. In order to calculate the heuristic distance, we need the coordinates
            (x, y
        :return: The function `calc_heuristic` returns the Euclidean distance between two points `n1`
            and `n2` multiplied by the weight `w`.
        """
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def find_total_cost(self, open_set, lambda_, n1):
        """
        This function calculates the total cost for a given node in a search algorithm based on the
        g_cost, h_cost, and a specified lambda value.

        :param open_set: The `open_set` parameter is a data structure that typically stores nodes that
            are candidates for further exploration in a search algorithm like A* or Dijkstra's algorithm. It
            usually contains nodes that have been discovered but not yet visited or expanded
        :param lambda_: Lambda_ is the index of the node in the open set for which you want to calculate
            the total cost. It is used to access the specific node in the open set for which the cost needs
            to be calculated
        :param n1: It looks like you were about to provide a description of the `n1` parameter, but the
            description is missing. Could you please provide more information about what the `n1` parameter
            represents or is used for in the `find_total_cost` method?
        :return: the total cost (f_cost) calculated based on the given parameters: the cost of the
            current node in the open set (g_cost), the heuristic cost from the current node to the target
            node (h_cost), and the sum of the two (f_cost).
        """

        g_cost = open_set[lambda_].cost
        h_cost = self.calc_heuristic(n1, open_set[lambda_])
        f_cost = g_cost + h_cost
        return f_cost

    def calc_grid_position(self, index, min_position):
        """
        This Python function calculates the position in a grid based on the index and minimum position
        provided.

        :param index: The `index` parameter in the `calc_grid_position` function represents the position
            of the element in the grid. It is used to calculate the final position of the element based on
            the resolution and the minimum position provided
        :param min_position: The `min_position` parameter in the `calc_grid_position` function
            represents the minimum position value that you want to start calculating the grid position from.
            This value is added to the result of the calculation using the formula `pos = index * self.resol
            + min_position`. It helps in determining the
        :return: the calculated position based on the index, resolution, and minimum position provided
            as input arguments.
        """

        pos = index * self.resol + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resol)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        """
        The function `verify_node` checks if a given node is within the grid boundaries and not
        colliding with obstacles.

        :param node: The `verify_node` function takes a `node` as a parameter. The `node` likely
            represents a point in a grid or a position in a coordinate system. The function first calculates
            the grid position of the node based on the minimum x and y values. It then checks if the
            calculated grid
        :return: The function `verify_node` returns a boolean value - `True` if the node is valid
            (within the grid boundaries and not colliding with obstacles), and `False` if the node is
            invalid (outside grid boundaries or colliding with obstacles).
        """

        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
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
        The function calculates an obstacle map based on given obstacle coordinates and a specified
        resolution.

        :param x_obs: It seems like the `x_obs` parameter represents a list of x-coordinates of
            obstacles in your obstacle map calculation function. You can pass a list of x-coordinates of
            obstacles to this function for further processing
        :param y_obs: It seems like the parameter `y_obs` represents the y-coordinates of the obstacles
            in your obstacle map calculation function. These y-coordinates are used along with the
            x-coordinates (`x_obs`) to calculate the obstacle map. If you have any specific questions or
            need further assistance with this code snippet
        """

        self.min_x = round(min(x_obs))
        self.min_y = round(min(y_obs))
        self.max_x = round(max(x_obs))
        self.max_y = round(max(y_obs))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resol)
        self.y_width = round((self.max_y - self.min_y) / self.resol)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [
            [False for _ in range(self.y_width)] for _ in range(self.x_width)
        ]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(x_obs, y_obs):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def motion_mod():
        # dx, dy, cost
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
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

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
    for i in range(0, 40):
        for j in range(10, 20):
            x_obs.append(i)
            y_obs.append(15)
    for i in range(10, 60):
        x_obs.append(40.0)
        y_obs.append(60.0 - i)

    if show_animation:
        plt.title("Bidirectional A Star Planner Algorithm")
        plt.plot(x_obs, y_obs, ".r")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "ob")
        plt.grid(True)
        plt.axis("equal")

    bidirastar = BidirectionalAStarPlanner(x_obs, y_obs, grid_size, robot_radius)
    rx, ry = bidirastar.planning(sx, sy, gx, gy)

    if show_animation:
        plt.plot(rx, ry, "-b")

        plt.pause(0.0001)
        plt.show()


if __name__ == "__main__":
    main()
