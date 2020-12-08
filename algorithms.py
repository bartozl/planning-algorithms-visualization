from numpy import Inf
from utils import reconstruct_path, dist, get_neighborhood, line_of_sight


def greed_best_first(grid, wall, goal, h, frontier, inner, come_from):

	curr = next(iter(frontier))
	for node in frontier:
		if h[node] < h[curr]:
			curr = node

	if curr == goal:
		path = reconstruct_path(curr, come_from)
		return None, None, path

	frontier.discard(curr)
	inner.add(curr)
	neighborhood = get_neighborhood(curr, grid, wall)

	for neighbour in neighborhood:

		if (neighbour not in inner) and (neighbour not in frontier):
			frontier.add(neighbour)
			come_from[neighbour] = curr

	return frontier, inner, come_from


def A_star(grid, wall, goal, h, frontier, inner, g_score, f_score, come_from):

	curr = next(iter(frontier))
	for node in frontier:
		if f_score[node] < f_score[curr]:
			curr = node

	if curr == goal:
		path = reconstruct_path(curr, come_from)
		return None, None, None, path

	frontier.discard(curr)
	inner.add(curr)
	neighborhood = get_neighborhood(curr, grid, wall)

	for neighbour in neighborhood:

		if neighbour not in g_score.keys():
			g_score[neighbour] = Inf

		if neighbour in inner:
			continue

		new_gScore = g_score[curr] + dist(curr, neighbour)

		if new_gScore < g_score[neighbour]:
			come_from[neighbour] = curr
			g_score[neighbour] = new_gScore
			f_score[neighbour] = new_gScore + h[neighbour]
			if neighbour not in frontier:
				frontier.add(neighbour)

	return frontier, inner, g_score, come_from


def Dijkstra(grid, wall, goal, frontier, inner, g_score, come_from):

	curr = next(iter(frontier))
	for node in frontier:
		if g_score[node] < g_score[curr]:
			curr = node

	if curr == goal:
		path = reconstruct_path(curr, come_from)
		return None, None, None, path

	frontier.discard(curr)
	inner.add(curr)
	neighborhood = get_neighborhood(curr, grid, wall)

	for neighbour in neighborhood:

		if neighbour not in g_score.keys():
			g_score[neighbour] = Inf

		if neighbour in inner:
			continue

		new_gScore = g_score[curr] + dist(curr, neighbour)

		if new_gScore < g_score[neighbour]:
			come_from[neighbour] = curr
			g_score[neighbour] = new_gScore
			if neighbour not in frontier:
				frontier.add(neighbour)

	return frontier, inner, g_score, come_from


def Theta_star(grid, wall, start, goal, h, frontier, inner, g_score, f_score, come_from):
	curr = next(iter(frontier))
	for node in frontier:
		if f_score[node] < f_score[curr]:
			curr = node

	if curr == goal:
		path = reconstruct_path(curr, come_from)
		return None, None, None, path

	frontier.discard(curr)
	inner.add(curr)
	neighborhood = get_neighborhood(curr, grid, wall)

	for neighbour in neighborhood:

		if neighbour not in g_score.keys():
			g_score[neighbour] = Inf
			come_from[neighbour] = curr

		if neighbour in inner:
			continue

		parent_curr = come_from[curr] if curr is not start else start

		if line_of_sight(parent_curr, neighbour, wall):
			new_gScore = g_score[parent_curr] + dist(parent_curr, neighbour)
			if new_gScore < g_score[neighbour]:
				g_score[neighbour] = new_gScore
				f_score[neighbour] = new_gScore + h[neighbour]
				come_from[neighbour] = parent_curr
				frontier.add(neighbour)
		else:
			new_gScore = g_score[curr] + dist(curr, neighbour)
			if new_gScore < g_score[neighbour]:
				g_score[neighbour] = new_gScore
				f_score[neighbour] = new_gScore + h[neighbour]
				come_from[neighbour] = curr
				frontier.add(neighbour)

	return frontier, inner, g_score, come_from




