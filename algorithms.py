from numpy import Inf
from utils import reconstruct_path, dist, get_neighborhood, line_of_sight


def Dijkstra(grid, wall, goal, frontier, inner, g_score, come_from):
	curr = next(iter(frontier))

	# choose as current the node with higher G score (the closer to reach)
	for node in frontier:
		if g_score[node] < g_score[curr]:
			curr = node

	# check if it is the  goal
	if curr == goal:
		path = reconstruct_path(curr, come_from)
		return None, None, None, path

	# switch the selected node from frontier to inner set
	frontier.discard(curr)
	inner.add(curr)

	# get all the nodes adjacent to the current one
	neighborhood = get_neighborhood(curr, grid, wall)

	for neighbour in neighborhood:
		# if a node is reachable for the first time, initialize its G score to Inf
		if neighbour not in g_score.keys():
			g_score[neighbour] = Inf

		# skip the inner nodes
		if neighbour in inner:
			continue

		# compute the new G score
		new_gScore = g_score[curr] + dist(curr, neighbour)

		# if a shorter path to reach the neighbour has been found --> update its G score
		if new_gScore < g_score[neighbour]:
			come_from[neighbour] = curr
			g_score[neighbour] = new_gScore
			if neighbour not in frontier:
				frontier.add(neighbour)

	return frontier, inner, g_score, come_from


def greed_best_first(grid, wall, goal, h, frontier, inner, come_from):
	curr = next(iter(frontier))
	# choose as current the node with higher heuristic value (euclidean distance from goal)
	for node in frontier:
		if h[node] < h[curr]:
			curr = node

	# check if it is the goal
	if curr == goal:
		path = reconstruct_path(curr, come_from)
		return None, None, path

	# switch the selected node from frontier to inner set
	frontier.discard(curr)
	inner.add(curr)

	# get all the nodes adjacent to the current one
	neighborhood = get_neighborhood(curr, grid, wall)

	for neighbour in neighborhood:
		# if a node is reachable for the first time, add it to the frontier
		if (neighbour not in inner) and (neighbour not in frontier):
			frontier.add(neighbour)
			come_from[neighbour] = curr

	return frontier, inner, come_from


def A_star(grid, wall, goal, h, frontier, inner, g_score, f_score, come_from):
	curr = next(iter(frontier))

	# choose as current the node with the higher F score, where F(n) = G(n) + H(n)
	for node in frontier:
		if f_score[node] < f_score[curr]:
			curr = node

	# check if it is the goal
	if curr == goal:
		path = reconstruct_path(curr, come_from)
		return None, None, None, path

	# switch the selected node from frontier to inner set
	frontier.discard(curr)
	inner.add(curr)

	# get all the nodes adjacent to the current one
	neighborhood = get_neighborhood(curr, grid, wall)

	for neighbour in neighborhood:
		# if a node is reachable for the first time, initialize its G score to Inf
		if neighbour not in g_score.keys():
			g_score[neighbour] = Inf

		# skip the inner nodes
		if neighbour in inner:
			continue

		# compute the new G score
		new_gScore = g_score[curr] + dist(curr, neighbour)

		# if a shorter path to reach the neighbour has been found --> update its F score
		if new_gScore < g_score[neighbour]:
			come_from[neighbour] = curr
			g_score[neighbour] = new_gScore
			f_score[neighbour] = new_gScore + h[neighbour]
			if neighbour not in frontier:
				frontier.add(neighbour)

	return frontier, inner, g_score, come_from


def Theta_star(grid, wall, start, goal, h, frontier, inner, g_score, f_score, come_from):
	curr = next(iter(frontier))

	# choose as current the node with the higher F score, where F(n) = G(n) + H(n)
	for node in frontier:
		if f_score[node] < f_score[curr]:
			curr = node

	# check if it is the goal
	if curr == goal:
		path = reconstruct_path(curr, come_from)
		return None, None, None, path

	# switch the selected node from frontier to inner set
	frontier.discard(curr)
	inner.add(curr)

	# get all the nodes adjacent to the current one
	neighborhood = get_neighborhood(curr, grid, wall)

	for neighbour in neighborhood:
		# if a node is reachable for the first time, initialize its G score to Inf
		if neighbour not in g_score.keys():
			g_score[neighbour] = Inf
			come_from[neighbour] = curr

		# skip the inner nodes
		if neighbour in inner:
			continue

		# store the parent of the current node
		parent_curr = come_from[curr] if curr is not start else start

		# check if the neighbor (n) is reachable from the parent of the current node (parent_curr)
		if line_of_sight(parent_curr, neighbour, wall):
			# compute the new G score for n, considering it reachable from parent_curr
			new_gScore = g_score[parent_curr] + dist(parent_curr, neighbour)
			# if the path from parent_curr --> n is shorter then curr --> n, update the F score accordingly
			if new_gScore < g_score[neighbour]:
				g_score[neighbour] = new_gScore
				f_score[neighbour] = new_gScore + h[neighbour]
				come_from[neighbour] = parent_curr
				frontier.add(neighbour)
		# the neighbor (n) is not reachable from parent_curr, then execute the standard F score update (as A*)
		else:
			new_gScore = g_score[curr] + dist(curr, neighbour)
			if new_gScore < g_score[neighbour]:
				g_score[neighbour] = new_gScore
				f_score[neighbour] = new_gScore + h[neighbour]
				come_from[neighbour] = curr
				frontier.add(neighbour)

	return frontier, inner, g_score, come_from


