import numpy as np
import math
from queue import PriorityQueue

# image contains 0 and 1 : 0 represents barriers/obstacles and 1 means walkable
WALKABLE = 0
OBSTACLE = 1
CLOSED = -1
START = -2
END = -3
PATH = -4


class Node:
    def __init__(self, row, col, total_rows, total_cols, value=None):
        self.row = row
        self.col = col
        self.value = value
        self.neighbors = []
        self.total_rows = total_rows
        self.total_cols = total_cols

    def get_pos(self):
        return self.row, self.col

    def is_closed(self):
        return self.value == CLOSED

    def is_open(self):
        return self.value == WALKABLE

    def is_barrier(self):
        return self.value == OBSTACLE

    def is_start(self):
        return self.value == START

    def is_end(self):
        return self.value == END

    def reset(self):
        self.value = None

    def make_closed(self):
        self.value = CLOSED

    def make_open(self):
        self.value = WALKABLE

    def make_barrier(self):
        self.value = OBSTACLE

    def make_start(self):
        self.value = START

    def make_end(self):
        self.value = END

    def make_path(self):
        self.value = PATH

    def update_neighbors(self, grid):
        self.neighbors = []
        # 4 neighbors (DOWN, UP, RIGHT and LEFT)
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():  # DOWN
            self.neighbors.append(grid[self.row + 1][self.col])

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():  # UP
            self.neighbors.append(grid[self.row - 1][self.col])

        if self.col < self.total_cols - 1 and not grid[self.row][self.col + 1].is_barrier():  # RIGHT
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():  # LEFT
            self.neighbors.append(grid[self.row][self.col - 1])

        # diagonal neighbors
        if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].is_barrier():  # top left neighbor
            self.neighbors.append(grid[self.row - 1][self.col - 1])

        if self.row > 0 and self.col < self.total_cols - 1 and not grid[self.row - 1][
            self.col + 1].is_barrier():  # top right neighbor
            self.neighbors.append(grid[self.row - 1][self.col + 1])

        if self.row < self.total_rows - 1 and self.col > 0 and not grid[self.row + 1][
            self.col - 1].is_barrier():  # bottom left neighbor
            self.neighbors.append(grid[self.row + 1][self.col - 1])

        if self.row < self.total_rows - 1 and self.col < self.total_cols - 1 and not grid[self.row + 1][
            self.col + 1].is_barrier():  # bottom right neighbor
            self.neighbors.append(grid[self.row + 1][self.col + 1])

    def __lt__(self, other):
        return False


# distance function
def h(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    # return abs(x1 - x2) + abs(y1 - y2)
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


# convert image to occupancy grid
def image_to_occupancy_grid(im):
    rows = im.shape[0]
    cols = im.shape[1]
    grid = []
    for i in range(rows):
        grid.append([])
        for j in range(cols):
            n = Node(i, j, rows, cols)
            n.value = im[i, j]
            grid[i].append(n)
    return grid


def reconstruct_path(came_from, current, scale):
    path = []
    path.append((current.get_pos()[0]*scale, current.get_pos()[1]*scale))
    while current in came_from:
        current = came_from[current]
        path.append((current.get_pos()[0]*scale, current.get_pos()[1]*scale))
    if len(path) <= 20:
        return path[::-1]
    elif len(path) < 100:
        rpath = (path[::-1])[80//scale::80//scale]
        if path[0] != rpath[-1]:
            rpath.append(path[0])
    else:
        rpath = (path[::-1])[80//scale::80//scale]
        if path[0] != rpath[-1]:
            rpath.append(path[0])
    # print("start Point: ", rpath[0])
    # print("the calculated path:",rpath)
    return rpath  # reverse the list


def pathfinding_astar(im, start, end, scale):
    # create nodes in the occupancy grid and define start and end nodes
    start = (start[1] // scale, start[0] // scale)  # must
    end = (end[1] // scale, end[0] // scale)

    grid = image_to_occupancy_grid(im)
    start_node = Node(start[0], start[1], im.shape[0], im.shape[1], START)
    grid[start[0]][start[1]] = start_node
    end_node = Node(end[0], end[1], im.shape[0], im.shape[1], END)
    grid[end[0]][end[1]] = end_node

    # update neighbors for all nodes
    for row in grid:
        for node in row:
            node.update_neighbors(grid)

    # A* algorithm
    count = 0
    open_set = PriorityQueue()
    open_set.put((0, count, start_node))
    came_from = {}
    g_score = {node: float("inf") for row in grid for node in row}
    g_score[start_node] = 0
    f_score = {node: float("inf") for row in grid for node in row}
    f_score[start_node] = h(start_node.get_pos(), end_node.get_pos())

    open_set_hash = {start_node}

    path = []
    # main loop
    while not open_set.empty():
        current = open_set.get()[2]
        open_set_hash.remove(current)

        if current == end_node:
            path = reconstruct_path(came_from, end_node, scale)
            end_node.make_end()
            break

        for neighbor in current.neighbors:
            temp_g_score = g_score[current] + 1
            if temp_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = temp_g_score
                f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end_node.get_pos())
                if neighbor not in open_set_hash:
                    count += 1
                    open_set.put((f_score[neighbor], count, neighbor))
                    open_set_hash.add(neighbor)
                    neighbor.make_open()

        if current != start_node:
            current.make_closed()

    if path == []:
        print('No solution-----------------------------------------------------------------------------------')
        return None
    return path


# # testing the function
# if __name__ == "__main__":
#     maze = [[0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
#             [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]
#
#     maze = np.array(maze)
#     maze = 1 - maze
#     start = (0, 0)
#     end = (7, 6)
#     path = pathfinding_astar(maze, start, end)
#     print(path)