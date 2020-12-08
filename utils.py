import numpy as np


def reconstruct_path(curr, come_from):
    path = []
    while curr is not None:
        path.insert(0, curr)
        curr = come_from[curr]
    return path


def dist(a, b):
    return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def path_length(path):
    length = 0
    for i in range(len(path) - 1):
        length += dist(path[i], path[i + 1])
    return length


def heuristic(grid, goal, metric='euclidean'):
    h = np.zeros(grid.shape)
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            if metric is 'euclidean':
                h[x][y] = np.sqrt((x - goal[0]) ** 2 + (y - goal[1]) ** 2)
            if metric is 'manhattan':
                h[x][y] = dist((x, y), goal)
    return h


def get_neighborhood(cell, grid, wall):
    moves = [[0, -1], [0, 1], [-1, 0], [1, 0], [-1, -1], [1, -1], [-1, 1], [1, 1]]
    coordinates = [tuple(cell + m) for m in np.array(moves)]
    infeasible = [elem for elem in coordinates if \
                  (elem[0] < 0) or (elem[1] < 0) or \
                  (elem[0] >= grid.shape[0]) or (elem[1] >= grid.shape[1]) \
                  or (elem in wall)]
    return [tuple(elem) for elem in coordinates if elem not in infeasible]


def line_of_sight(a, b, wall, accuracy=100):
    try:
        xs = np.rint(np.linspace(a[0], b[0], accuracy))
        ys = np.rint(np.linspace(a[1], b[1], accuracy))
        for x, y in zip(xs, ys):
            if (x, y) in wall:
                return False
        return True
    except:
        return False


def post_smoothing(come_from, wall):
    k = 0
    path = [come_from[0]]
    for i in range(len(come_from) - 1):
        if not line_of_sight(path[k], come_from[i + 1], wall):
            k += 1
            path.append(come_from[i])
    k += 1
    path.append(come_from[-1])
    return path
