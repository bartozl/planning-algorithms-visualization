import sys

import numpy as np
import pygame

from algorithms import A_star, Dijkstra, greed_best_first, Theta_star
from utils import heuristic, post_smoothing, path_length


class world:

    def __init__(self, height=20, width=20, margin=1, pixels=800, step_by_step=False, update_speed=45):
        self.print_this = ''
        self.screen = None
        self.update_speed = update_speed if step_by_step is False else -1
        self.clock = pygame.time.Clock()
        self.step_by_step = step_by_step
        self.H = height
        self.W = width
        self.MARGIN = margin
        self.PIXELS = pixels // height
        self.WINDOW_SIZE = [self.PIXELS * self.W + self.MARGIN * self.W + self.MARGIN,
                            self.PIXELS * self.H + self.MARGIN * self.H + self.MARGIN]

        self.caption = 'environment'

        self.control = {"LEFT_CLICK": 1, "MIDDLE_CLICK": 2, "RIGHT_CLICK": 3,
                        "ENTER": 13, "CTRL": 306, "SHIFT": 304, "SPACE": 32,
                        "A": 97, "D": 100, "G": 103, "P": 112,
                        "Q": 113, "R": 114, "S": 115, "T": 116}

        self.goal = None
        self.start = None
        self.paths = []  # list of paths (e.g. path[0] found by A*, path[1] found by GBF exc.)
        self.wall = []  # list of wall cells

        # color used for cells and paths
        self.color = {"SHADOW": (192, 192, 192), "WHITE": (255, 255, 255), "LIGHTGREEN": (0, 255, 0),
                      "GREEN": (35, 250, 44), "BLUE": (0, 0, 128), "LIGHTBLUE": (0, 0, 255),
                      "RED": (220, 0, 0), "LIGHTRED": (255, 100, 100), "PURPLE": (102, 0, 102),
                      "LIGHTPURPLE": (153, 0, 153), "BLACK": (0, 0, 0), "YELLOW": (245, 255, 137)}

    def update_grid(self, grid, cell_type, pos):
        """
        :param grid: the cell grid
        :param cell_type: "clean", "source", "wall", "goal", "frontier", "inner", "path",
        :param pos: cell that should be updated
        :return: nothing. Update the grid by changing the color of the cell "pos" w.r.t. the cell_type value
        """
        x, y = pos  # pos = (x,y) = (h, w)
        value = None

        if cell_type == 'clean':
            value = 0
            if grid[x][y] == 1:
                self.start = None  # deleting a source
            elif grid[x][y] == 2:
                self.wall = [cell for cell in self.wall if cell != (x, y)]
            elif grid[x][y] == 3:
                self.goal = None  # deleting a goal

        elif cell_type == 'source':
            if self.start is None and grid[x][y] not in [2, 3]:  # don't put a source on wall(2) or goal(3)
                self.start = (x, y)
                value = 1

        elif cell_type == 'wall':
            if grid[x][y] not in [1, 3]:  # don't put a wall on source(1) or goal(3)
                self.wall.append((x, y))
                value = 2

        elif cell_type == 'goal':
            if self.goal is None and grid[x][y] not in [1, 2]:  # don't put a goal on source(1) or wall(2)
                self.goal = (x, y)
                value = 3

        elif cell_type == 'frontier':
            value = 4
            if grid[x][y] == 1:
                value = 1
            if grid[x][y] == 3:
                value = 3

        elif cell_type == 'inner':
            value = 5

        elif cell_type == 'path':
            value = 6
            if grid[x][y] == 1:
                value = 1
            if grid[x][y] == 3:
                value = 3

        if cell_type is not None and value is not None:
            grid[x][y] = value

    def draw(self, grid):
        x, y = self.MARGIN, self.MARGIN
        for row in grid:
            for elem in row:
                color = self.color['WHITE']
                if elem == 1:
                    color = self.color['GREEN']
                elif elem == 2:
                    color = self.color['BLACK']
                elif elem == 3:
                    color = self.color['RED']
                elif elem == 4:
                    color = self.color['YELLOW']
                elif elem == 5:
                    color = self.color['SHADOW']
                elif elem == 6:
                    color = self.color['LIGHTBLUE']
                # draw the cell
                pygame.draw.rect(self.screen, color, [x, y, self.PIXELS, self.PIXELS])
                x += self.PIXELS + self.MARGIN
            y += self.PIXELS + self.MARGIN
            x = self.MARGIN

        pygame.display.set_caption(self.caption)

        self.draw_path()

    def draw_path(self):
        closed = False
        width = 10
        for path, color in self.paths:
            pointlist = np.asarray([pos[::-1] for pos in path]) * (self.MARGIN + self.PIXELS) + (self.PIXELS / 2)
            pygame.draw.lines(self.screen, color, closed, pointlist, width)

    def reset_world(self):
        if len(self.paths) != 0:  # separate prints of different runs
            print('* ' * (len(self.print_this) // 2))
        self.caption = 'environment'
        self.goal = None
        self.start = None
        self.paths = []
        self.wall = []
        clean_grid = np.zeros([self.H, self.W], dtype=np.int8)
        return clean_grid

    def set_the_env(self, clean_grid):
        drag = False  # flag used for wall-cells creation and cells cleaning
        clean = False  # flag used for cleaning the entire grid
        curr_grid = clean_grid.copy()  # the actual grid.
        # The clean_grid stores only source, goal and walls cells. Useful for cleaning
        while True:
            # capture the pygame events
            for event in pygame.event.get():
                # endgame event
                if event.type == pygame.QUIT:
                    return False

                # when a key in the keyboard is released
                if event.type == pygame.KEYUP:

                    # CTRL released: end of cleaning phase
                    if event.key == self.control["CTRL"]:
                        clean = False

                    # SHIFT released: end of walls drawing phase
                    if event.key == self.control["SHIFT"]:
                        drag = False

                    # A, D, P, G, T, S released: apply the selected algorithm
                    if event.key == self.control["A"]:
                        curr_grid = self.algorithm(self.screen, clean_grid, 'A_star', self.color["RED"])
                    if event.key == self.control["D"]:
                        curr_grid = self.algorithm(self.screen, clean_grid, 'Dijkstra', self.color["YELLOW"])
                    if event.key == self.control["P"]:
                        curr_grid = self.algorithm(self.screen, clean_grid, 'A_star_PS', self.color["PURPLE"])
                    if event.key == self.control["G"]:
                        curr_grid = self.algorithm(self.screen, clean_grid, 'greed_best_first', self.color["BLUE"])
                    if event.key == self.control["T"]:
                        curr_grid = self.algorithm(self.screen, clean_grid, 'Theta_star', self.color["GREEN"])
                    if event.key == self.control["S"]:
                        curr_grid.image.save(self.screen, '{}.png'.format(self.caption))

                # when a key in the keyboard is pressed
                if event.type == pygame.KEYDOWN:
                    # CTRL pressed: start of cleaning phase
                    if event.key == self.control["CTRL"]:
                        clean = True

                    # SHIFT pressed: allow wall creation phase
                    if event.key == self.control["SHIFT"]:
                        drag = True
                        if len(self.paths) != 0:  # separate prints of different runs
                            print('* ' * (len(self.print_this) // 2))
                        self.paths = []  # clean the already drawn paths
                        curr_grid = clean_grid  # clean the colored cells

                    # R released: restart the scenario keeping the source, goal and walls
                    if event.key == self.control["R"]:
                        if len(self.paths) != 0:  # separate prints of different runs
                            print('* ' * (len(self.print_this) // 2))
                        curr_grid = clean_grid
                        self.paths = []

                    # Q released: restart the scenario from scratch
                    if event.key == self.control["Q"]:
                        clean_grid = self.reset_world()
                        curr_grid = clean_grid

                # when a mouse button is pressed
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button in [self.control["LEFT_CLICK"], self.control["RIGHT_CLICK"]]:
                        curr_grid = clean_grid
                    # middle click pressed: allow walls creations by moving the mouse
                    if event.button == self.control["MIDDLE_CLICK"]:
                        drag = True

                # when the mouse is moving
                if event.type == pygame.MOUSEMOTION:
                    cell_type = None
                    if drag:
                        cell_type = 'wall'
                        self.paths = []  # clean the already drawn paths
                    if clean:
                        cell_type = 'clean'
                    if cell_type is not None:
                        # update both the current and the previous grid
                        pos = np.asarray(event.pos)[::-1] // (self.PIXELS + self.MARGIN)
                        self.update_grid(clean_grid, cell_type, pos)
                        self.update_grid(curr_grid, cell_type, pos)

                # when a mouse button is released
                if event.type == pygame.MOUSEBUTTONUP:
                    cell_type = None

                    # add a source cell
                    if event.button == self.control["LEFT_CLICK"]:
                        if self.start is None:
                            cell_type = 'source'

                    # add a wall cell
                    if event.button == self.control["MIDDLE_CLICK"]:
                        drag = False
                        cell_type = 'wall'

                    # add a goal cell
                    if event.button == self.control["RIGHT_CLICK"]:
                        if self.goal is None:
                            cell_type = 'goal'
                            # self.goal = np.asarray(event.pos)[::-1] // (self.PIXELS + self.MARGIN)

                    # update both the current and the previous grid
                    pos = np.asarray(event.pos)[::-1] // (self.PIXELS + self.MARGIN)
                    self.update_grid(curr_grid, cell_type, pos)
                    self.update_grid(clean_grid, cell_type, pos)

            self.screen.fill(self.color['BLACK'])
            self.draw(curr_grid)
            pygame.display.flip()

    def step(self, data_algo, algorithm, curr_grid, color):
        done = False
        if (algorithm is 'A_star') or (algorithm is 'A_star_PS'):
            data_algo['frontier'], data_algo['inner'], data_algo['g_score'], data_algo['come_from'] = A_star(curr_grid,
                                                                                                             self.wall,
                                                                                                             self.goal,
                                                                                                             data_algo[
                                                                                                                 'h'],
                                                                                                             data_algo[
                                                                                                                 'frontier'],
                                                                                                             data_algo[
                                                                                                                 'inner'],
                                                                                                             data_algo[
                                                                                                                 'g_score'],
                                                                                                             data_algo[
                                                                                                                 'f_score'],
                                                                                                             data_algo[
                                                                                                                 'come_from'])
        if algorithm is 'Dijkstra':
            data_algo['frontier'], data_algo['inner'], data_algo['g_score'], data_algo['come_from'] = Dijkstra(
                curr_grid,
                self.wall, self.goal,
                data_algo['frontier'],
                data_algo['inner'],
                data_algo['g_score'],
                data_algo['come_from'])
        if algorithm is 'greed_best_first':
            data_algo['frontier'], data_algo['inner'], data_algo['come_from'] = greed_best_first(curr_grid, self.wall,
                                                                                                 self.goal,
                                                                                                 data_algo['h'],
                                                                                                 data_algo['frontier'],
                                                                                                 data_algo['inner'],
                                                                                                 data_algo['come_from'])
        if algorithm is 'Theta_star':
            data_algo['frontier'], data_algo['inner'], data_algo['g_score'], data_algo['come_from'] = Theta_star(
                curr_grid,
                self.wall,
                self.start,
                self.goal,
                data_algo['h'],
                data_algo['frontier'],
                data_algo['inner'],
                data_algo['g_score'],
                data_algo['f_score'],
                data_algo['come_from'])

        # color the grid cells according to step updates
        if data_algo['frontier'] is None:  # path has been found
            if algorithm is 'A_star_PS':
                data_algo['come_from'] = post_smoothing(data_algo['come_from'], self.wall)
            for pos in data_algo['come_from']:
                self.update_grid(curr_grid, 'path', pos)

            self.paths.append([data_algo['come_from'], color])
            done = True
        else:
            for pos in data_algo['frontier']:
                if (pos is not self.start) and (pos is not self.goal):
                    self.update_grid(curr_grid, 'frontier', pos)
            for pos in data_algo['inner']:
                if (pos is not self.start) and (pos is not self.goal):
                    self.update_grid(curr_grid, 'inner', pos)

        return data_algo, curr_grid, done

    def algorithm(self, screen, clean_grid, algorithm, color):
        """
        start, goal, wall = unpack_grid(clean_grid)
        if start == -1:
            return clean_grid
        """
        if self.start is None or self.goal is None:
            return clean_grid
        start = self.start
        goal = self.goal
        h = heuristic(clean_grid, goal)
        data_algo = {'h': h,
                     'frontier': {start},
                     'inner': set(),
                     'g_score': {start: 0},
                     'f_score': {start: h[start]},
                     'come_from': {start: None}}
        steps = 0

        curr_grid = clean_grid.copy()
        length = None

        while len(data_algo['frontier']) > 0:
            if self.step_by_step:
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        if event.type == pygame.QUIT:
                            sys.exit(0)
                        if event.key == self.control["R"]:
                            self.paths = []
                            return clean_grid
                        # update only when SPACE is pressed
                        if event.type == pygame.KEYDOWN and event.key == self.control["SPACE"]:
                            # apply one step of the selected algorithm
                            data_algo, curr_grid, done = self.step(data_algo, algorithm, curr_grid, color)
                            steps += 1
                            if done:
                                length = path_length(self.paths[-1][0])
                                self.print_this = "[algo] {:19s} [steps] {:6s} [path length] {:5s}" \
                                    .format(algorithm, str(steps), str(length))
                                print(self.print_this)
                                return curr_grid
            else:
                data_algo, curr_grid, done = self.step(data_algo, algorithm, curr_grid, color)
                steps += 1
                if done:
                    length = path_length(self.paths[-1][0])
                    self.print_this = "[algo] {:19s} [steps] {:6s} [path length] {:5s}" \
                        .format(algorithm, str(steps), str(length))
                    print(self.print_this)
                    return curr_grid

            self.caption = "{} {} {}".format(algorithm, steps, length)
            pygame.display.set_caption(self.caption)
            self.draw(curr_grid)
            pygame.display.flip()
            self.clock.tick(self.update_speed)

    def run(self):
        pygame.init()
        self.screen = pygame.display.set_mode(self.WINDOW_SIZE)
        pygame.display.set_caption(self.caption)
        repeat = True

        while repeat:
            self.start = None
            self.goal = None
            grid = np.zeros([self.H, self.W], dtype=np.int8)
            repeat = self.set_the_env(grid)

        return 1
