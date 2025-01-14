import math
from sys import maxsize
import matplotlib.pyplot as plt

show_animation = True


class instant_state:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.instant_state = "."
        self.t = "new"
        self.h = 0
        self.k = 0

    def cost(self, instant_state):
        if self.instant_state == "#" or instant_state.instant_state == "#":
            return maxsize

        return math.sqrt(
            math.pow((self.x - instant_state.x), 2)
            + math.pow((self.y - instant_state.y), 2)
        )

    def set_state(self, instant_state):
        if instant_state not in ["s", ".", "#", "e", "*"]:
            return
        self.instant_state = instant_state


class Layout:

    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.graph = self.init_map()

    def init_map(self):
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(instant_state(i, j))
            map_list.append(tmp)
        return map_list

    def get_neighbors(self, instant_state):
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if instant_state.x + i < 0 or instant_state.x + i >= self.row:
                    continue
                if instant_state.y + j < 0 or instant_state.y + j >= self.col:
                    continue
                state_list.append(self.graph[instant_state.x + i][instant_state.y + j])
        return state_list

    def set_obstacle(self, point_list):
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue

            self.graph[x][y].set_state("#")


class Dstar:
    def __init__(self, maps):
        self.graph = maps
        self.open_list = set()

    def process_state(self):
        x = self.min_state()

        if x is None:
            return -1

        k_old = self.get_kmin()
        self.remove(x)

        if k_old < x.h:
            for y in self.graph.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)
        if k_old == x.h:
            for y in self.graph.get_neighbors(x):
                if (
                    y.t == "new"
                    or y.parent == x
                    and y.h != x.h + x.cost(y)
                    or y.parent != x
                    and y.h > x.h + x.cost(y)
                ):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
        else:
            for y in self.graph.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(x, x.h)
                    else:
                        if (
                            y.parent != x
                            and x.h > y.h + x.cost(y)
                            and y.t == "close"
                            and y.h > k_old
                        ):
                            self.insert(y, y.h)
        return self.get_kmin()

    def min_state(self):
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, instant_state, h_new):
        if instant_state.t == "new":
            instant_state.k = h_new
        elif instant_state.t == "open":
            instant_state.k = min(instant_state.k, h_new)
        elif instant_state.t == "close":
            instant_state.k = min(instant_state.h, h_new)
        instant_state.h = h_new
        instant_state.t = "open"
        self.open_list.add(instant_state)

    def remove(self, instant_state):
        if instant_state.t == "open":
            instant_state.t = "close"
        self.open_list.remove(instant_state)

    def modify_cost(self, x):
        if x.t == "close":
            self.insert(x, x.parent.h + x.cost(x.parent))

    def run(self, start, end):

        rx = []
        ry = []

        self.insert(end, 0.0)

        while True:
            self.process_state()
            if start.t == "close":
                break

        start.set_state("s")
        s = start
        s = s.parent
        s.set_state("e")
        tmp = start

        AddNewObstacle(self.graph)  # add new obstacle after the first search finished

        while tmp != end:
            tmp.set_state("*")
            rx.append(tmp.x)
            ry.append(tmp.y)
            if show_animation:
                plt.plot(rx, ry, "-g")
                plt.pause(0.01)
            if tmp.parent.instant_state == "#":
                self.modify(tmp)
                continue
            tmp = tmp.parent
        tmp.set_state("e")

        return rx, ry

    def modify(self, instant_state):
        self.modify_cost(instant_state)
        while True:
            k_min = self.process_state()
            if k_min >= instant_state.h:
                break


def AddNewObstacle(graph: Layout):
    o_x, o_y = [], []
    for i in range(5, 30):
        o_x.append(i)
        o_y.append(0)
    graph.set_obstacle([(i, j) for i, j in zip(o_x, o_y)])
    if show_animation:
        plt.pause(0.001)
        plt.plot(o_x, o_y, ".k")


def main():
    m = Layout(100, 100)
    o_x, o_y = [], []
    for i in range(-10, 60):
        o_x.append(i)
        o_y.append(-10)
    for i in range(-10, 60):
        o_x.append(60)
        o_y.append(i)
    for i in range(-10, 61):
        o_x.append(i)
        o_y.append(60)
    for i in range(-10, 61):
        o_x.append(-10)
        o_y.append(i)
    for i in range(0, 50):
        for j in range(0, 20):
            o_x.append(i)
            o_y.append(20)
    for i in range(0, 60):
        o_x.append(40)
        o_y.append(60 - i)
    m.set_obstacle([(i, j) for i, j in zip(o_x, o_y)])

    start = [10, 10]
    goal = [50, 50]
    if show_animation:
        plt.plot(o_x, o_y, ".r")
        plt.plot(start[0], start[1], "og")
        plt.plot(goal[0], goal[1], "xb")
        plt.axis("equal")

    start = m.graph[start[0]][start[1]]
    end = m.graph[goal[0]][goal[1]]
    dstar = Dstar(m)
    rx, ry = dstar.run(start, end)

    if show_animation:
        plt.plot(rx, ry, "-b")
        plt.show()


if __name__ == "__main__":
    main()
