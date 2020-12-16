import sys
from collections import defaultdict

import numpy as np
import pygame

from algorithms import A_star, Dijkstra, greed_best_first, Theta_star
from utils import heuristic, post_smoothing, build_print_line, create_gif


class world:

    def __init__(self, height=20, width=20, margin=1, pixels=800, step_by_step=False, update_speed=45, gif=False):

        self.world_is_changed = True  # keep track of updates in start, goal or wall cells
        self.run_num = 0  # identify the run number
        self.print_this = ''  # print infos about [run][algo][steps][distance]
        self.printed_infos = []  # for each run, save results of each algorithms.
        self.step_by_step = step_by_step  # allows step by step execution of the algorithm
        self.best_algo_dict = defaultdict(int)
        self.algorithm_runs = defaultdict(int)  # keep track of number of runs with a spec. algorithm
        self.applied_this_run = defaultdict(bool)

        # pygame
        self.screen = None
        self.update_speed = update_speed if step_by_step is False else -1
        self.clock = pygame.time.Clock()
        self.caption = 'environment'

        # initialize the world
        self.H, self.W, self.MARGIN, self.PIXELS = height, width, margin, pixels // height
        self.WINDOW_SIZE = [self.PIXELS * self.W + self.MARGIN * self.W + self.MARGIN,
                            self.PIXELS * self.H + self.MARGIN * self.H + self.MARGIN]
        self.clean_grid = np.zeros([self.H, self.W], dtype=np.int8)
        self.curr_grid = self.clean_grid.copy()
        self.goal = None
        self.start = None
        self.paths = []  # list of paths (e.g. path[0] found by A*, path[1] found by GBF exc.)
        self.wall = []  # list of wall cells

        # utilities
        self.control = {"LEFT_CLICK": 1, "MIDDLE_CLICK": 2, "RIGHT_CLICK": 3,
                        "ENTER": 13, "CTRL": 306, "SHIFT": 304, "SPACE": 32,
                        "A": 97, "D": 100, "G": 103, "P": 112,
                        "Q": 113, "R": 114, "S": 115, "T": 116}
        self.color = {"SHADOW": (192, 192, 192), "WHITE": (255, 255, 255), "LIGHTGREEN": (0, 255, 0),
                      "GREEN": (35, 250, 44), "BLUE": (0, 0, 128), "LIGHTBLUE": (0, 0, 255),
                      "RED": (220, 0, 0), "LIGHTRED": (255, 100, 100), "PURPLE": (102, 0, 102),
                      "LIGHTPURPLE": (153, 0, 153), "BLACK": (0, 0, 0), "YELLOW": (245, 255, 137)}
        self.gif = gif

    def update_grid(self, cell_type, pos, clean_grid=False):
        """
        :param clean_grid: if true update the clean grid else the current grid
        :param cell_type: "clean", "source", "wall", "goal", "frontier", "inner", "path",
        :param pos: cell that should be updated
        :return: nothing. Update the grid by changing the color of the cell "pos" w.r.t. the cell_type value
        """
        if clean_grid:
            grid = self.clean_grid
        else:
            grid = self.curr_grid
        x, y = pos  # pos = (x,y) = (h, w)
        value = None

        try:
            if cell_type == 'clean':
                value = 0
                if grid[x][y] == 1:
                    self.start = None  # deleting a source
                    self.world_is_changed = True
                elif grid[x][y] == 2:
                    self.wall = [cell for cell in self.wall if cell != (x, y)]
                    self.world_is_changed = True
                elif grid[x][y] == 3:
                    self.goal = None  # deleting a goal
                    self.world_is_changed = True

            elif cell_type == 'source':
                if self.start is None and grid[x][y] not in [2, 3]:  # don't put a source on wall(2) or goal(3)
                    self.start = (x, y)
                    self.world_is_changed = True
                    value = 1

            elif cell_type == 'wall':
                if grid[x][y] not in [1, 3]:  # don't put a wall on source(1) or goal(3)
                    self.wall.append((x, y))
                    self.world_is_changed = True
                    value = 2

            elif cell_type == 'goal':
                if self.goal is None and grid[x][y] not in [1, 2]:  # don't put a goal on source(1) or wall(2)
                    self.goal = (x, y)
                    self.world_is_changed = True
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

            if clean_grid:
                self.clean_grid = grid
            else:
                self.curr_grid = grid

        except IndexError:
            pass

    def draw(self):
        x, y = self.MARGIN, self.MARGIN
        for row in self.curr_grid:
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
        self.draw_path()

    def draw_path(self):
        closed = False
        width = 10
        for path, color in self.paths:
            pointlist = np.asarray([pos[::-1] for pos in path]) * (self.MARGIN + self.PIXELS) + (self.PIXELS / 2)
            pygame.draw.lines(self.screen, color, closed, pointlist, width)

    def best_run(self, ever=False):
        if not ever:
            min_length = np.inf
            min_steps = np.inf
            best_algo = [(None, np.inf)]
            for run in self.printed_infos:
                if run['run'] != self.run_num or run['length'] > min_length:
                    continue
                # a shorter path has been found
                if run['length'] < min_length:
                    best_algo = [(run['algo'], run['steps'])]
                    min_length = run['length']
                    min_steps = run['steps']
                # a path equal to the shorter one has been found
                elif run['length'] == min_length:
                    # in the same number of steps --> add the algo to the list of the bests
                    if run['steps'] == best_algo[0][1]:
                        best_algo.append((run['algo'], run['steps']))
                    # in a lower number of steps --> reinitialize the list of the best with the new one
                    if run['steps'] < best_algo[0][1]:
                        best_algo = [(run['algo'], run['steps'])]
                        min_steps = run['steps']
            best_algo_list = [x[0] for x in list(dict.fromkeys(best_algo))]
            for algo in best_algo_list:
                self.best_algo_dict[algo] += 1
            best_algo = ', '.join(best_algo_list)
            return " *Best* --> length {} found in {} steps by {}".format(min_length, min_steps, best_algo)
        else:
            string = ''
            for algo in self.algorithm_runs:
                string += " {:16s} |  {:5s}% | {:.1f}% \n" \
                          .format(algo,
                                  str(np.round(100 * self.algorithm_runs[algo] / self.run_num, 1)),
                                  100 * self.best_algo_dict[algo] / self.algorithm_runs[algo])
            return string

    def reset_world(self, mode, new_run=False, keep_paths=False):
        # soft reset: start, goal and walls and run_num and paths are kept (called when R is pressed)
        # middle: start, goal, walls and run_num are kept (when a new algo in computed)
        if mode == 'soft':
            self.curr_grid = self.clean_grid
            if not keep_paths:
                self.paths = []
            # if goal, start, wall have been changed, it counts as a new run
            if self.world_is_changed and new_run:
                if len(self.print_this) > 0:
                    print(self.best_run())
                print('* ' * (len(self.print_this) // 2))
                print(' {:4s} | {:16s} |  {:6s} | {:6s}'.format('RUN', 'ALGORITHM', 'STEPS', 'LENGTH'))
                self.paths = []
                self.run_num += 1
                self.applied_this_run = defaultdict(bool)
                #

            self.world_is_changed = False

        # hard reset: complete reset of the environment (called when Q is pressed)
        elif mode == 'hard':
            if len(self.printed_infos) > 0:
                print(self.best_run())
                print('* ' * (len(self.print_this) // 2))
                print(' {:16s} |   {:4s}  | {:5s}'.format('ALGORITHM', 'USE', 'BEST'))
                print(self.best_run(ever=True))
                print('* ' * (len(self.print_this) // 2))
                # print(' {:4s} | {:16s} |  {:6s} | {:6s}'.format('RUN', 'ALGORITHM', 'STEPS', 'LENGTH'))
            self.algorithm_runs = defaultdict(int)
            self.best_algo_dict = defaultdict(int)
            self.applied_this_run = defaultdict(bool)
            self.print_this = ""
            self.printed_infos = []
            self.world_is_changed = False
            self.run_num = 0
            self.caption = 'environment'
            self.goal = None
            self.start = None
            self.paths = []
            self.wall = []
            self.clean_grid = np.zeros([self.H, self.W], dtype=np.int8)
            self.curr_grid = self.clean_grid

    def set_the_env(self):
        drag = False  # flag used for wall-cells creation and cells cleaning
        clean = False  # flag used for cleaning the entire grid
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
                        # self.reset_world('soft')

                    # SHIFT released: end of walls drawing phase
                    if event.key == self.control["SHIFT"]:
                        drag = False
                        # self.reset_world('soft')

                    # A, D, P, G, T, S released: apply the selected algorithm
                    if event.key == self.control["A"]:
                        self.algorithm('A_star', self.color["RED"])
                    if event.key == self.control["D"]:
                        self.algorithm('Dijkstra', self.color["YELLOW"])
                    if event.key == self.control["P"]:
                        self.algorithm('A_star_PS', self.color["PURPLE"])
                    if event.key == self.control["G"]:
                        self.algorithm('greed_best_first', self.color["BLUE"])
                    if event.key == self.control["T"]:
                        self.algorithm('Theta_star', self.color["GREEN"])
                    if event.key == self.control["S"]:
                        pygame.image.save(self.screen, '{}.png'.format(self.caption))

                # when a key in the keyboard is pressed
                if event.type == pygame.KEYDOWN:
                    # CTRL pressed: start of cleaning phase
                    if event.key == self.control["CTRL"]:
                        clean = True
                        self.reset_world('soft')

                    # SHIFT pressed: allow wall creation phase
                    if event.key == self.control["SHIFT"]:
                        drag = True
                        self.reset_world('soft')

                    # R released: restart the scenario keeping the source, goal and walls
                    if event.key == self.control["R"]:
                        self.reset_world('soft')

                    # Q released: restart the scenario from scratch
                    if event.key == self.control["Q"]:
                        self.reset_world('hard')

                # when a mouse button is pressed
                if event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button in [self.control["LEFT_CLICK"], self.control["RIGHT_CLICK"]]:
                        self.curr_grid = self.clean_grid
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
                        self.update_grid(cell_type, pos, clean_grid=True)  # update the clean_grid
                        self.update_grid(cell_type, pos, clean_grid=False)  # update the current grid

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

                    # update both the current and the previous grid
                    pos = np.asarray(event.pos)[::-1] // (self.PIXELS + self.MARGIN)
                    self.update_grid(cell_type, pos, clean_grid=False)  # update the clean_grid
                    self.update_grid(cell_type, pos, clean_grid=True)  # update the current grid

            self.update_screen()

    def update_screen(self):
        self.screen.fill(self.color['BLACK'])
        self.draw()
        pygame.display.set_caption(self.caption)
        pygame.display.flip()
        self.clock.tick(self.update_speed)

    def step(self, data_algo, algorithm, color):
        done = False
        if (algorithm is 'A_star') or (algorithm is 'A_star_PS'):
            data_algo['frontier'], data_algo['inner'], data_algo['g_score'], data_algo['come_from'] = A_star(
                self.curr_grid,
                self.wall,
                self.goal,
                data_algo['h'],
                data_algo['frontier'],
                data_algo['inner'],
                data_algo['g_score'],
                data_algo['f_score'],
                data_algo['come_from'])
        if algorithm is 'Dijkstra':
            data_algo['frontier'], data_algo['inner'], data_algo['g_score'], data_algo['come_from'] = Dijkstra(
                self.curr_grid,
                self.wall, self.goal,
                data_algo['frontier'],
                data_algo['inner'],
                data_algo['g_score'],
                data_algo['come_from'])
        if algorithm is 'greed_best_first':
            data_algo['frontier'], data_algo['inner'], data_algo['come_from'] = greed_best_first(self.curr_grid,
                                                                                                 self.wall,
                                                                                                 self.goal,
                                                                                                 data_algo['h'],
                                                                                                 data_algo['frontier'],
                                                                                                 data_algo['inner'],
                                                                                                 data_algo['come_from'])
        if algorithm is 'Theta_star':
            data_algo['frontier'], data_algo['inner'], data_algo['g_score'], data_algo['come_from'] = Theta_star(
                self.curr_grid,
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
                self.update_grid('path', pos, clean_grid=False)

            self.paths.append([data_algo['come_from'], color])
            done = True
        else:
            for pos in data_algo['frontier']:
                if (pos is not self.start) and (pos is not self.goal):
                    self.update_grid('frontier', pos, clean_grid=False)
            for pos in data_algo['inner']:
                if (pos is not self.start) and (pos is not self.goal):
                    self.update_grid('inner', pos, clean_grid=False)

        return data_algo, done

    def algorithm(self, algorithm, color):
        if self.start is None or self.goal is None:
            self.curr_grid = self.clean_grid.copy()
            return

        self.reset_world('soft', new_run=True, keep_paths=True)

        # avoid increment when algorithm is applied two times on the same world
        if not self.applied_this_run[algorithm]:
            self.algorithm_runs[algorithm] += 1
        self.applied_this_run[algorithm] = True

        start = self.start
        goal = self.goal
        h = heuristic(self.clean_grid, goal)
        data_algo = {'h': h,
                     'frontier': {start},
                     'inner': set(),
                     'g_score': {start: 0},
                     'f_score': {start: h[start]},
                     'come_from': {start: None}}
        steps = 0

        self.curr_grid = self.clean_grid.copy()

        while len(data_algo['frontier']) > 0:

            if self.step_by_step:

                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:

                        if event.type == pygame.QUIT:
                            sys.exit(0)

                        if event.key == self.control["R"]:
                            self.paths = []
                            self.curr_grid = self.clean_grid
                            return

                        # update only when SPACE is pressed
                        if event.type == pygame.KEYDOWN and event.key == self.control["SPACE"]:

                            # apply one step of the selected algorithm
                            data_algo, done = self.step(data_algo, algorithm, color)
                            steps += 1

                            if done:
                                self.print_this, self.printed_infos = \
                                    build_print_line(algorithm, steps, self.paths, self.run_num, self.printed_infos)
                                self.caption = self.print_this
                                return

            else:

                data_algo, done = self.step(data_algo, algorithm, color)
                steps += 1

                if done:
                    self.print_this, self.printed_infos = \
                        build_print_line(algorithm, steps, self.paths, self.run_num, self.printed_infos)
                    self.caption = self.print_this
                    self.update_screen()
                    if self.gif:
                        pygame.image.save(self.screen, './temp/{}.bmp'.format(steps))
                        create_gif(self.run_num, algorithm)


                    return

            self.update_screen()
            if self.gif:
                pygame.image.save(self.screen, './temp/{}.bmp'.format(steps))


    def run(self):
        pygame.init()
        self.screen = pygame.display.set_mode(self.WINDOW_SIZE)
        pygame.display.set_caption(self.caption)
        repeat = True

        while repeat:
            self.start = None
            self.goal = None
            self.clean_grid = np.zeros([self.H, self.W], dtype=np.int8)
            self.curr_grid = self.clean_grid
            repeat = self.set_the_env()

        return 1
