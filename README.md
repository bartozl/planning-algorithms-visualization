# Planning Algorithms Visualization


## Problem definition

Finding a unique definition of the path planning problem is not trivial. In fact, it can be encountered in a lot of different fields, such as network routing, traffic organization, autonomous vehicles and so on. Anyway, as a general definition, it can be asserted that it is the problem of finding an _available_ path from a starting point (start) point to an ending one (goal). In the literature, many different techniques have been proposed to solve this problem; one of these consists in the application of a search algorithm over a graph.



## Project scope  
The idea behind this project, is to create a tool targeting people that are approaching this kind of algorithms for the first time. This is done by providing an interactive graphic tool that allows the user to create a customized environment in a grid world: the user can add a source cell, a goal cell, and the walls, and then choose the algorithm to apply. The algorithm execution is then visualized step by step.



## How to use

```
git clone https://github.com/bartozl/planning-algorithms-visualization.git
```

```
pip install -r requirements.txt
```

```
python play.py
```

- Mouse controls:

  - Left click: add start cell

  - Right click: add goal cell

  - Mouse middle click: add wall cell

  - Mouse middle click + mouse movement: add wall cells

- Keyboard controls:

  - SHIFT + mouse movement: add wall cells

  - CTRL + mouse movement: delete a cell

  -  R: reset the world keeping the source, the goal and the walls
  - Q: reset the world completely
  - S: take a screen-shot of the world
  - D: execute Dijkstra algorithm
  - G: execute Greed Best First algorithm
  - A: execute A* algorithm
  - P: execute A* PS algorithm
  - T: execute Theta* algorithm



## Algorithms

#### Keywords

* **Uninformed search**: there is no information about the distance between the nodes and the goal

* **Informed search**: a cost estimation for reaching the goal from the current node is made by an heuristic function.

* **Heuristic Function h(n)**: The quality of the heuristics affects these kinds of algorithms; the main  requirements for a good heuristic are **admissibility** (never overestimate the cost) and **consistency** (_h(goal) = 0 and h(n) <= (dist(n, p) + h(p)_), where p in neighbor(n)). 
  In a path-planning problem, the Euclidean distance from the goal always represents an admissible and consistent heuristic.

* **Cost Function f(n)**: the next node to be expanded is chosen in order to minimize this function.

  

#### Greed Best First

<img src="/home/lorenzo/university/projects/planning-algorithms-visualization/1_greed_best_first.gif" alt="1_greed_best_first" style="zoom:50%;" />

**Cost function**: f(n) = h(n)

The expansion only depends on the heuristic: the algorithm selects the path that appears to be the best at the moment.

**Advantages**: Easy to implement. Fast to be executed.

**Drawbacks**: Non optimal solution.



#### A*

<img src="/home/lorenzo/university/projects/planning-algorithms-visualization/1_A_star.gif" alt="1_A_star" style="zoom:50%;" />

**Cost function**: f(n) = h(n) + g(n)

The expansion depends on the heuristic h(n) and the cost of the path from the starting node and the current node n (_i.e._ g(n)). The expansion is implemented as follow:

- **Frontier** nodes (open set) = candidate nodes for the expansion in the next step (yellow cells in the animation)

- **Inner** nodes (closed set) = already visited nodes  (gray cells in the animation)

- **Unexplored** nodes are white in the animation

  In each iteration, the frontier's node with the lowest f(n) is chosen a the current node _curr_. If it is the goal, the algorithm terminates. Otherwise, the node is removed from the Frontier and added to the Inner set. For each _m_ in neighbor(_curr_), if _m_ is not an Inner node, the new g_new(_m_) value is computed and if it is lower then the previous one, f(m) is updated.

  Note: g_new(_m_) = g(_curr_) + dist(_curr_, _m_)

  If g_new(_m_) < g_prev(_m_) it means that a shorter path from start to _m_ has been found.

**Advantages**: Optimal solution for admissible heuristics.

**Drawbacks**: Unnatural paths (useless turns) in a grid world.



#### A* Post-Smoothing

<img src="/home/lorenzo/university/projects/planning-algorithms-visualization/1_A_star_PS.gif" alt="1_A_star_PS" style="zoom:50%;" />

After the A* solution is found, apply a post-smoothing in order to reduce the turns on the path by directly connecting nodes that are in the _sight of view_

**Advantages**: Improve the results of A* in a grid world

**Drawbacks**: The solutions is not necessary the best possible one.



#### Theta*

<img src="/home/lorenzo/university/projects/planning-algorithms-visualization/1_Theta_star.gif" alt="1_Theta_star" style="zoom:50%;" />

**Cost function**: f(n) = h(n) + g(n)

