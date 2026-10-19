"""Reverse BFS flood fill on the robot's known map, never the hidden maze."""

from collections import deque
from typing import Dict, Optional

from maze import Cell, DIRECTIONS, KnownMaze, WALL, neighbor


def flood_fill(known: KnownMaze, goal: Cell) -> Dict[Cell, Optional[int]]:
    """Return optimistic distances to goal; None means unreachable.

    Each move has cost 1. UNKNOWN edges are treated as open for planning;
    the simulator separately requires a sensed-open edge before moving.
    """
    if not known.contains(goal):
        raise ValueError("Goal is outside the map")
    distances = {
        (x, y): None for y in range(known.size) for x in range(known.size)
    }
    distances[goal] = 0
    queue = deque([goal])
    while queue:
        cell = queue.popleft()
        for direction in DIRECTIONS:
            other = neighbor(cell, direction)
            if known.edge_state(cell, direction) == WALL:
                continue
            if distances[other] is None:
                distances[other] = distances[cell] + 1
                queue.append(other)
    return distances
