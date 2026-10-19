"""Maze walls and the robot's incomplete map. Coordinates are (x, y)."""

from dataclasses import dataclass
from typing import Dict, FrozenSet, Mapping, Optional, Tuple

Cell = Tuple[int, int]
Direction = Tuple[int, int]
Edge = Tuple[Cell, Cell]
DIRECTIONS = ((1, 0), (0, 1), (-1, 0), (0, -1))  # East, south, west, north.
UNKNOWN, OPEN, WALL = "unknown", "open", "wall"


def neighbor(cell: Cell, direction: Direction) -> Cell:
    return cell[0] + direction[0], cell[1] + direction[1]


def edge_key(a: Cell, b: Cell) -> Edge:
    """One shared key makes observations symmetric across adjacent cells."""
    if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
        raise ValueError("An edge must connect two adjacent cells")
    return tuple(sorted((a, b)))


@dataclass(frozen=True)
class Maze:
    name: str
    description: str
    size: int = 8
    walls: FrozenSet[Edge] = frozenset()
    start: Cell = (0, 0)
    goal: Optional[Cell] = None

    def __post_init__(self):
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
        return 0 <= cell[0] < self.size and 0 <= cell[1] < self.size

    def sense(self, cell: Cell) -> Dict[Direction, str]:
        """The ideal sensor reveals ONLY the four edges of the current cell."""
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
    """What the robot knows; internal edges start UNKNOWN, boundaries are WALL."""

    def __init__(self, size: int, start: Cell, goal: Cell):
        self.size, self.start, self.goal = size, start, goal
        self._edges: Dict[Edge, str] = {}

    def contains(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.size and 0 <= cell[1] < self.size

    def edge_state(self, cell: Cell, direction: Direction) -> str:
        if not self.contains(cell) or direction not in DIRECTIONS:
            raise ValueError("Invalid cell or direction")
        other = neighbor(cell, direction)
        if not self.contains(other):
            return WALL
        return self._edges.get(edge_key(cell, other), UNKNOWN)

    def record(self, cell: Cell, observation: Mapping[Direction, str]) -> None:
        for direction, state in observation.items():
            previous = self.edge_state(cell, direction)
            if state not in (OPEN, WALL):
                raise ValueError("A sensor observation must be open or wall")
            if previous != UNKNOWN and previous != state:
                raise ValueError("Contradictory observation in a static maze")
            other = neighbor(cell, direction)
            if self.contains(other):
                self._edges[edge_key(cell, other)] = state


# A corridor at the right edge ends at (7, 5), forcing the robot to backtrack.
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
