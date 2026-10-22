"""Compute a goal-distance field by reverse breadth-first search on the known map.

Unknown edges remain traversable during planning until a wall is observed.
"""

from collections import deque
from typing import Dict, Optional

from maze import Cell, DIRECTIONS, KnownMaze, WALL, neighbor


def flood_fill(known: KnownMaze, goal: Cell) -> Dict[Cell, Optional[int]]:
    """Return minimum move counts to goal on the optimistic known map.

    The result maps every cell to a distance, or None if disconnected from goal.
    Unknown edges are treated as open; known is not modified. Raise ValueError
    if goal is outside the map.
    """
    if not known.contains(goal):
        raise ValueError("Goal is outside the map")

    distances = {
        (x, y): None for y in range(known.size) for x in range(known.size)
    }
    # Reverse BFS yields distances toward the same goal for every reachable cell.
    distances[goal] = 0
    queue = deque([goal])

    while queue:
        cell = queue.popleft()
        for direction in DIRECTIONS:
            other = neighbor(cell, direction)
            # Optimism allows unexplored routes; boundary edges are already WALL.
            if known.edge_state(cell, direction) == WALL:
                continue
            if distances[other] is None:
                distances[other] = distances[cell] + 1
                queue.append(other)

    return distances
