"""Coordinate local sensing, flood-fill replanning and discrete robot moves.

The planner receives only the knowledge map, never the hidden wall layout.
"""

from maze import DIRECTIONS, KnownMaze, Maze, OPEN, UNKNOWN, WALL, neighbor
from planner import flood_fill


class Simulation:
    """Manage an exploration run using local observations from a true Maze.

    Expose the knowledge map, distance field, position, path and terminal status
    for visualization; the first exploration route need not be globally shortest.
    """

    def __init__(self, maze: Maze):
        """Initialize a run from maze and sense and plan at its start without moving."""
        # Retain only the local sensing interface to the true maze.
        self._sense = maze.sense
        self.known = KnownMaze(maze.size, maze.start, maze.goal)
        self.position = maze.start
        self.heading = (1, 0)
        self.steps = 0
        self.visited = {self.position}
        self.path = [self.position]
        self.status = "exploring"
        self.last_event = "Start: sensed the four edges of the starting cell."
        self._observe_and_plan()

    def _observe_and_plan(self) -> int:
        """Sense the current cell, replan and update terminal status; return new wall count."""
        observation = self._sense(self.position)
        # Count before recording, while newly discovered walls are still UNKNOWN.
        new_walls = sum(
            state == WALL and self.known.edge_state(self.position, direction) == UNKNOWN
            for direction, state in observation.items()
        )
        self.known.record(self.position, observation)
        self.distances = flood_fill(self.known, self.known.goal)
        if self.position == self.known.goal:
            self.status = "reached"
            self.last_event = "Goal reached. The first exploration route need not be shortest."
        # If even the optimistic map has no route, the true maze cannot have one.
        elif self.distances[self.position] is None:
            self.status = "unreachable"
            self.last_event = "Observed walls prove the goal is unreachable. Robot stopped."
        return new_walls

    def step(self) -> bool:
        """Move at most one cell, then sense and replan at the new position.

        Return True if a move occurred, including a terminal move; return False
        without changing state if exploration has ended. Raise RuntimeError if
        no sensed-open neighbor decreases the pre-move distance by one.
        """
        if self.status != "exploring":
            return False
        current_distance = self.distances[self.position]
        for direction in DIRECTIONS:
            other = neighbor(self.position, direction)
            # Planning permits UNKNOWN edges, but executed moves require sensed OPEN.
            if (self.known.edge_state(self.position, direction) == OPEN
                    and self.distances.get(other) == current_distance - 1):
                revisiting = other in self.visited
                self.position, self.heading = other, direction
                self.steps += 1
                self.visited.add(other)
                self.path.append(other)
                # The move uses the old distance field; new walls may increase distances.
                new_walls = self._observe_and_plan()
                # Preserve the terminal message produced by sensing and replanning.
                if self.status == "exploring":
                    action = "Revisited" if revisiting else "Entered"
                    self.last_event = (
                        "{} cell ({}, {}); found {} new wall(s); recalculated distances."
                        .format(action, other[0], other[1], new_walls)
                    )
                return True
        raise RuntimeError("No sensed-open downhill neighbor; map invariant broken")
