"""Define the immutable maze and the robot's observed edge map.

Coordinates start at the top left; x increases rightward and y downward.
"""

from dataclasses import dataclass
from typing import Dict, FrozenSet, Mapping, Optional, Tuple

Cell = Tuple[int, int]
Direction = Tuple[int, int]
Edge = Tuple[Cell, Cell]
# East, south, west, north: also the deterministic move tie-break order.
DIRECTIONS = ((1, 0), (0, 1), (-1, 0), (0, -1))
UNKNOWN, OPEN, WALL = "unknown", "open", "wall"


def neighbor(cell: Cell, direction: Direction) -> Cell:
    """Return cell translated by direction, without validating adjacency or bounds."""
    return cell[0] + direction[0], cell[1] + direction[1]


def edge_key(a: Cell, b: Cell) -> Edge:
    """Return a canonical undirected edge between adjacent cells a and b.

    Raise ValueError if the cells are not orthogonally adjacent.
    """
    if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
        raise ValueError("An edge must connect two adjacent cells")
    # Both directions share one key, keeping wall and observation states symmetric.
    return tuple(sorted((a, b)))


@dataclass(frozen=True)
class Maze:
    """Store a static square maze with internal walls, start and goal cells.

    name and description identify the maze; size defines its bounds and walls
    contains undirected internal edges. An omitted goal defaults to the bottom
    right corner. Invalid geometry raises ValueError during initialization.
    """

    name: str
    description: str
    size: int = 8
    walls: FrozenSet[Edge] = frozenset()
    start: Cell = (0, 0)
    goal: Optional[Cell] = None

    def __post_init__(self):
        """Normalize walls and the default goal, rejecting invalid geometry.

        Raise ValueError for a nonpositive size, out-of-bounds start or goal,
        or walls whose endpoints are outside the maze or not adjacent.
        """
        if self.size < 1:
            raise ValueError("Maze size must be positive")
        if self.goal is None:
            object.__setattr__(self, "goal", (self.size - 1, self.size - 1))
        if not self.contains(self.start) or not self.contains(self.goal):
            raise ValueError("Start and goal must be inside the maze")
        normalized = set()
        for a, b in self.walls:
            if not self.contains(a) or not self.contains(b):
                raise ValueError("Internal walls must join cells inside the maze")
            normalized.add(edge_key(a, b))
        object.__setattr__(self, "walls", frozenset(normalized))

    def contains(self, cell: Cell) -> bool:
        """Return whether cell lies within this maze's bounds."""
        return 0 <= cell[0] < self.size and 0 <= cell[1] < self.size

    def sense(self, cell: Cell) -> Dict[Direction, str]:
        """Return the four edge states at cell as a direction-to-OPEN/WALL map.

        Observations are local and noiseless. Raise ValueError if cell is outside
        the maze.
        """
        if not self.contains(cell):
            raise ValueError("Cannot sense outside the maze")
        return {
            direction: WALL if (
                not self.contains(neighbor(cell, direction))
                or edge_key(cell, neighbor(cell, direction)) in self.walls
            ) else OPEN
            for direction in DIRECTIONS
        }


class KnownMaze:
    """Store observed internal edges for a maze of known size, start and goal.

    Unobserved internal edges are UNKNOWN; outer boundaries are always WALL.
    """

    def __init__(self, size: int, start: Cell, goal: Cell):
        """Initialize empty edge knowledge with size, start and goal, without validation."""
        self.size, self.start, self.goal = size, start, goal
        self._edges: Dict[Edge, str] = {}

    def contains(self, cell: Cell) -> bool:
        """Return whether cell lies within the known map's bounds."""
        return 0 <= cell[0] < self.size and 0 <= cell[1] < self.size

    def edge_state(self, cell: Cell, direction: Direction) -> str:
        """Return UNKNOWN, OPEN or WALL for the edge from cell in direction.

        Raise ValueError if cell is outside the map or direction is not cardinal.
        """
        if not self.contains(cell) or direction not in DIRECTIONS:
            raise ValueError("Invalid cell or direction")
        other = neighbor(cell, direction)
        # Boundary walls are implicit and need no stored observations.
        if not self.contains(other):
            return WALL
        return self._edges.get(edge_key(cell, other), UNKNOWN)

    def record(self, cell: Cell, observation: Mapping[Direction, str]) -> None:
        """Record direction-to-OPEN/WALL observations at cell in iteration order.

        Validate each entry; raise ValueError for invalid cells, directions or
        states, or contradictions. Earlier updates remain if a later entry fails.
        An empty observation is a no-op. Return None.
        """
        for direction, state in observation.items():
            previous = self.edge_state(cell, direction)
            if state not in (OPEN, WALL):
                raise ValueError("A sensor observation must be open or wall")
            if previous != UNKNOWN and previous != state:
                raise ValueError("Contradictory observation in a static maze")
            other = neighbor(cell, direction)
            if self.contains(other):
                self._edges[edge_key(cell, other)] = state


# Enclose the right-hand corridor below its entrance to force a return trip.
_dead_end_walls = {
    edge_key((6, y), (7, y)) for y in range(1, 6)
} | {edge_key((7, 5), (7, 6))}

PRESETS = {
    "open": Maze("Open maze", "No internal walls: the robot reaches the goal in 14 moves."),
    "detour": Maze(
        "Dead-end detour",
        "A hidden dead end makes the robot turn back and find another route.",
        walls=frozenset(_dead_end_walls),
    ),
    "blocked": Maze(
        "Unreachable goal",
        "The goal is enclosed. The robot stops when its observations prove no route exists.",
        walls=frozenset({edge_key((6, 7), (7, 7)), edge_key((7, 6), (7, 7))}),
    ),
}
