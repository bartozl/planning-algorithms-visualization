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
  - Q: reset the world completely and print a recap of all the previous runs
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

* **h(n)** is the **heuristic function**: The quality of the heuristics affects these kinds of algorithms; the main  requirements for a good heuristic are **admissibility** (never overestimate the cost) and **consistency** (_h(goal) = 0 and h(n) <= (dist(n, p) + h(p)_), where p in neighborhood(n)). 
  In a path-planning problem, the Euclidean distance from the goal always represents an admissible and consistent heuristic.

* **g(n)** is the **distance from the START to the node n**. It is initialized as 0 for the start node, and Inf for the others. Then, when from the node _curr_ we are visiting a neighbor node _m_, the new g(m) is computed as sum(g(curr), dist(curr, m))
  
* **f(n)** is the **cost function**: the next node to be expanded is chosen in order to minimize this function.

* **Frontier** nodes (open set) = candidate nodes for the expansion in the next step (yellow cells in the animation)

* **Inner** nodes (closed set) = already visited nodes  (gray cells in the animation)

* **Unexplored** nodes are white in the animation

* **neighborhood(n)** = set of cells directly reachable from the n nodes.

  

  <h3 align="center", >
      Dijkstra
  </h3>

  <p align="center">
      <img src="https://i.postimg.cc/NMNS4G9J/1-Dijkstra.gif" width="300" align="center"/>
  </p>

  **Uninformed search**

  **Cost function**: f(n) = g(n)

  Since there is no information about the world, the expansion considers only the distance from the START node.

  

  <h3 align="center", >
      Greed Best First
  </h3>

  <p align=center>
      <img src="https://i.postimg.cc/3R7Y6ww3/1-greed-best-first.gif" width="300">
  </p>

**Informed search**

**Cost function**: f(n) = h(n)

The expansion only depends on the heuristic: the algorithm selects the path that appears to be the best at the moment.

**Advantages**: Easy to implement. Fast to be executed.

**Drawbacks**: Non optimal solution.



<h3 align="center", >
    A*
</h3>

<p align="center">
    <img src="https://i.postimg.cc/cJfdX30D/1-A-star.gif" width=300 align="center"/>
</p>

**Informed search**

**Cost function**: f(n) = h(n) + g(n)

The expansion depends on the heuristic h(n) and the cost of the path from the starting node (START) and the current node n (_i.e._ g(n)). The expansion is implemented as follow:

In each iteration, the **Frontier**'s node with the lowest f(n) is chosen a the current node _curr_. If it is the goal, the algorithm terminates. Otherwise, the node is removed from the Frontier and added to the Inner set. For each _m_ in neighborhood(_curr_), if _m_ is not an **Inner** node, the new **g score** for _m_ is computed (see keywords sec.) If **g_new(_m_)** it is lower then g(m), it mean that a shorter path from START and m has been found; then, f(m) is updated accordingly and the _m_ node is added to the frontier set.

**Advantages**: Optimal solution for admissible heuristics.

**Drawbacks**: Unnatural paths (useless turns) in a grid world.



<h3 align="center", >
    A* Post-smoothing
</h3>

<p align="center">
<img src="https://i.postimg.cc/prLxW187/1-A-star-PS.gif" width="300" align="center"/>
</p>

**Informed search**

After the A* solution is found, apply a post-smoothing in order to reduce the turns on the path by directly connecting nodes that are in the _sight of view_

**Advantages**: Improve the results of A* in a grid world

**Drawbacks**: The solution is not guaranteed to be optimal one.



<h3 align="center", >
    Theta*
</h3>

<p align="center">
    <img src="https://i.postimg.cc/Px4TkkLY/1-Theta-star.gif" width="300" align="center"/>
</p>

**Informed search**

**Cost function**: f(n) = h(n) + g(n)

The expansion is similar to A*, but in order to find smoother paths, the **g(m)** computation is different. Instead of computing new_g(m) as sum(g(curr), dist(curr, m)), we first check if _parent_curr_ (the node that connect the _curr_ node with the previous) is in **sight of view** with _m_. This would mean that the _m_ node is reachable also from the _parent_curr_ node, making useless the intermediate walk through _curr_. So, if this connection is possible, new_g(m) is computed as:

new_g(m) = sum(g_score(_parent_curr_), dist(_parent_curr,_ _m_))

Otherwise (_i.e._ there is not a sight of view between _parent_curr_ and _m_) the g(m) update follows the standard procedure.

**Advantages**: the smoothing procedure is embedded in the expansion process --> no post-smoothing phase.

**Drawbacks**: the solution is not guaranteed to be optimal one.



<h3 align="center", >
    Results from the previous gifs
</h3>

<p align="center">
    <img src="https://i.postimg.cc/7LkkCcDh/results.png" width="450" align=center/>
</p>



#### 					Results recap for 10 different runs (Q is pressed)

