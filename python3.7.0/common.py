#!/usr/bin/env python3

from typing import Tuple, Dict, Set, List

import math
import random
import itertools
import heapq
import copy

Edge = Tuple[Tuple[float, float], Tuple[float, float]]


class EasyPriorityQueue:
    """ Priority queue that allows removal (which, shockingly,
        none of the built-in queues seem to permit).

        Written using the suggestions from:
        https://docs.python.org/3/library/heapq.html#priority-queue-implementation-notes"""

    REMOVED = '<removed-task>'     # placeholder for a removed item

    def __init__(self, reverse_tiebreaking_order=False):
        self.pq = list()                    # list of entries arranged in a heap
        self.entry_finder = dict()          # mapping of items to entries

        # Counter that we use to apply a unique number to each element.
        # This ensures that elements with equal priority are popped in
        # a deterministic order. See priority queue implementation
        # notes linked above for details.
        self.counter = itertools.count()

        # Determines whether nodes with the same priority are popped in
        # the same order they were pushed, or the reverse order:
        self.reverse_tiebreaking_order = reverse_tiebreaking_order

    def push(self, item, priority):
        assert item not in self.entry_finder
        assert item != self.REMOVED

        count = next(self.counter)
        if self.reverse_tiebreaking_order:
            count = -count

        entry = [priority, count, item]

        self.entry_finder[item] = entry
        heapq.heappush(self.pq, entry)

    def pop(self):
        while len(self.pq) > 0:
            priority, count, item = heapq.heappop(self.pq)
            if item is not self.REMOVED:
                del self.entry_finder[item]
                return item
        raise KeyError('pop from an empty priority queue')

    def remove(self, item):
        entry = self.entry_finder.pop(item)
        entry[-1] = self.REMOVED

    def change_priority(self, item, new_priority):
        # TODO: what does/should this method do if the item is not yet present?
        self.remove(item)
        self.push(item, new_priority)

    def push_or_change_priority(self, item, new_priority):
        if item in self:
            self.change_priority(item, new_priority)
        else:
            self.push(item, new_priority)

    def __contains__(self, item):
        return item in self.entry_finder

    def __len__(self):
        return len(self.entry_finder)

    def __str__(self):
        fullcopy = copy.deepcopy(self)
        item_to_priority = dict()
        while len(fullcopy.pq) > 0:
            priority, count, item = heapq.heappop(fullcopy.pq)
            if item is not fullcopy.REMOVED:
                del fullcopy.entry_finder[item]
                item_to_priority[item] = priority
        return str(item_to_priority)


def L2_norm(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """ Euclidean distance """
    a_x, a_y = a
    b_x, b_y = b
    d_x = abs(a_x - b_x)
    d_y = abs(a_y - b_y)
    return math.sqrt(d_x * d_x + d_y * d_y)


def find_path_a_star(get_neighbors, goal: Tuple[float, float], start) -> List[Tuple[float, float]]:
    """ Returns the path (from start to goal) AND the closed list & parent relationships. """
    open_list = EasyPriorityQueue()
    heuristic = L2_norm

    # A node's g-value is the cost of the shortest path (that we have
    # discovered so far) from the start to that node:
    g_values: Dict[Tuple, float] = dict()
    closed_list: Set[Tuple] = set()     # vertices that have been expanded
    parents: Dict[Tuple, Tuple] = dict()
    open_list.push(start, heuristic(start, goal))
    g_values[start] = 0

    def construct_path(start, goal, parents: Dict) -> List[Tuple[float, float]]:
        """ Use a dictionary of parental relationships to construct a path from the start to the goal."""
        assert isinstance(parents, dict)
        path = [goal]
        current = goal
        while current != start:
            current = parents[current]
            if current in path:
                raise Exception("Found a loop:", current,
                                "already in path", path)
            assert current not in path  # checking for loops
            path.append(current)
        path.reverse()
        return path

    while len(open_list) > 0:

        current = open_list.pop()
        closed_list.add(current)

        if current == goal:
            return construct_path(start, goal, parents)

        for neighbor in get_neighbors(current):
            if neighbor not in closed_list:

                edge_cost = L2_norm(current, neighbor)
                cost_from_current = g_values[current] + edge_cost
                priority = cost_from_current + heuristic(neighbor, goal)

                if neighbor in open_list:

                    # have we found a quicker way to reach the neighbor?
                    if cost_from_current < g_values[neighbor]:
                        parents[neighbor] = current
                        g_values[neighbor] = cost_from_current
                        open_list.change_priority(neighbor, priority)

                # never seen this vertex before:
                else:
                    g_values[neighbor] = cost_from_current
                    parents[neighbor] = current
                    open_list.push(neighbor, priority)

    # if we got here, we could not find a path:
    return []


def sample_graph() -> Set[Edge]:
    vertices = [(random.uniform(0.0, 1.0), random.uniform(0.0, 1.0))
                for i in range(0, 100)]
    vertices.append((0.0, 0.0))
    vertices.append((1.0, 1.0))
    edges = set()
    for v1 in vertices:
        for v2 in vertices:
            if v1 is not v2:
                if L2_norm(v1, v2) < 0.20:
                    # sort, so we don't get both (a, b) and (b, a)
                    pair = [v1, v2]
                    pair.sort()
                    edges.add((pair[0], pair[1]))
    return edges

###
# Example code. Requires the 'graph' variable (a set of edges) to be in scope.
###
'''
def get_neighbors(v: Tuple[float, float]) -> List[Tuple[float, float]]:
    neighbors = []
    for edge in graph:
        v1, v2 = edge
        if v == v1:
            neighbors.append(v2)
        if v == v2:
            neighbors.append(v1)
    return neighbors

def obtain_shortest_path_edges() -> List[Edge]:
    path = find_path_a_star(get_neighbors, goal=(1, 1), start=(0, 0))
    if len(path) == 0:
        raise Exception("No path found!")
    edges = []
    for v1, v2 in zip(path[:-1], path[1:]):
        edges.append((v1, v2))
    return edges

def remove_edges_from_graph(edges: List[Edge]):
    """ Due to floating-point equality, we must perform inexact comparisons
        to remove edges from the graph. """
    edges_to_remove = []
    for edge in edges:
        v1, v2 = edge
        pair = [v1, v2]
        pair.sort()
        for graph_v1, graph_v2 in graph:
            if L2_norm(v1, graph_v1) < 0.0001 and L2_norm(v2, graph_v2) < 0.0001:
                edges_to_remove.append(edge)
    for edge in edges_to_remove:
        graph.remove(edge)
'''