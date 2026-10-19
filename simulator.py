"""One discrete sense-plan-move loop. No physics or learned model is used."""

from maze import DIRECTIONS, KnownMaze, Maze, OPEN, UNKNOWN, WALL, neighbor
from planner import flood_fill


class Simulation:
    def __init__(self, maze: Maze):
        # Only this ideal sensor can access the true maze. The planner cannot.
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
        observation = self._sense(self.position)
        new_walls = sum(
            state == WALL and self.known.edge_state(self.position, direction) == UNKNOWN
            for direction, state in observation.items()
        )
        self.known.record(self.position, observation)
        self.distances = flood_fill(self.known, self.known.goal)
        if self.position == self.known.goal:
            self.status = "reached"
            self.last_event = "Goal reached. The first exploration route need not be shortest."
        elif self.distances[self.position] is None:
            self.status = "unreachable"
            self.last_event = "Observed walls prove the goal is unreachable. Robot stopped."
        return new_walls

    def step(self) -> bool:
        """Move at most one cell, then sense and replan at the new position."""
        if self.status != "exploring":
            return False
        current_distance = self.distances[self.position]
        for direction in DIRECTIONS:
            other = neighbor(self.position, direction)
            if (self.known.edge_state(self.position, direction) == OPEN
                    and self.distances.get(other) == current_distance - 1):
                revisiting = other in self.visited
                self.position, self.heading = other, direction
                self.steps += 1
                self.visited.add(other)
                self.path.append(other)
                new_walls = self._observe_and_plan()
                if self.status == "exploring":
                    action = "Revisited" if revisiting else "Entered"
                    self.last_event = (
                        "{} cell ({}, {}); found {} new wall(s); recalculated distances."
                        .format(action, other[0], other[1], new_walls)
                    )
                return True
        # With consistent observations, a finite BFS distance always has a downhill edge.
        raise RuntimeError("No sensed-open downhill neighbor; map invariant broken")
