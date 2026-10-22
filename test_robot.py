"""Test local sensing, flood-fill distances and exploration invariants without the GUI.

Use an independent Dijkstra implementation as the distance and reachability oracle.
"""

import heapq
import random
import unittest

from maze import DIRECTIONS, KnownMaze, Maze, OPEN, PRESETS, UNKNOWN, WALL, edge_key
from planner import flood_fill
from simulator import Simulation


def actual_distances(maze):
    """Return {cell: distance or None} to maze.goal using the complete Maze.

    The heap-based Dijkstra oracle uses maze.walls directly and does not call
    the planner or consult the robot's knowledge map. None marks unreachable cells.
    """
    result = {(x, y): None for y in range(maze.size) for x in range(maze.size)}
    result[maze.goal] = 0
    frontier = [(0, maze.goal)]
    while frontier:
        distance, cell = heapq.heappop(frontier)
        if result[cell] != distance:
            continue
        x, y = cell
        for other in ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)):
            if not (0 <= other[0] < maze.size and 0 <= other[1] < maze.size):
                continue
            if tuple(sorted((cell, other))) in maze.walls:
                continue
            candidate = distance + 1
            if result[other] is None or candidate < result[other]:
                result[other] = candidate
                heapq.heappush(frontier, (candidate, other))
    return result


def random_maze(seed, size, wall_probability):
    """Return a reproducible Maze for seed, size and internal wall_probability.

    Each internal edge is sampled once. Maze raises ValueError if size is less
    than one; start and goal use its defaults.
    """
    rng = random.Random(seed)
    walls = set()
    for y in range(size):
        for x in range(size):
            # East and south cover each undirected internal edge exactly once.
            for other in ((x + 1, y), (x, y + 1)):
                if other[0] < size and other[1] < size:
                    if rng.random() < wall_probability:
                        walls.add(((x, y), other))
    return Maze("Random {}".format(seed), "Test maze", size, frozenset(walls))


class RobotTests(unittest.TestCase):
    """Verify map consistency, safe exploration and planner correctness."""

    def assert_local_knowledge(self, simulation, maze):
        """Check simulation observations against maze and the visited-cell boundary.

        Return None on success; raise AssertionError for inconsistent boundary,
        observed or unknown edges, including asymmetric shared-edge states.
        """
        for y in range(maze.size):
            for x in range(maze.size):
                cell = (x, y)
                for dx, dy in DIRECTIONS:
                    other = (x + dx, y + dy)
                    observed = simulation.known.edge_state(cell, (dx, dy))
                    if not maze.contains(other):
                        self.assertEqual(observed, WALL)
                    elif cell in simulation.visited or other in simulation.visited:
                        expected = WALL if tuple(sorted((cell, other))) in maze.walls else OPEN
                        self.assertEqual(observed, expected, (cell, other))
                    else:
                        self.assertEqual(observed, UNKNOWN, (cell, other))
                    if maze.contains(other):
                        reverse = simulation.known.edge_state(other, (-dx, -dy))
                        self.assertEqual(observed, reverse, (cell, other))

    def run_checked(self, maze):
        """Run maze to termination and return its Simulation after invariant checks.

        Raise AssertionError for unsafe moves, inconsistent knowledge or history,
        failure to terminate within the test guard, or changes after termination.
        """
        simulation = Simulation(maze)
        self.assert_local_knowledge(simulation, maze)
        # This bound prevents a hanging test; it is not a proven algorithmic bound.
        for _ in range(4 * maze.size ** 4):
            if simulation.status != "exploring":
                break
            # Preserve the decision-time map; step() senses and replans after moving.
            before = simulation.position
            distances_before = dict(simulation.distances)
            edges_before = {
                direction: simulation.known.edge_state(before, direction)
                for direction in DIRECTIONS
            }
            self.assertTrue(simulation.step())
            after = simulation.position
            direction = (after[0] - before[0], after[1] - before[1])
            self.assertIn(direction, DIRECTIONS)
            self.assertTrue(maze.contains(after))
            self.assertNotIn(tuple(sorted((before, after))), maze.walls)
            self.assertEqual(edges_before[direction], OPEN)
            self.assertEqual(distances_before[after], distances_before[before] - 1)
            self.assertEqual(simulation.steps, len(simulation.path) - 1)
            self.assertEqual(simulation.visited, set(simulation.path))
            self.assert_local_knowledge(simulation, maze)
        self.assertNotEqual(simulation.status, "exploring", "Robot failed to stop")
        # Terminal step() calls must leave position, step count and path unchanged.
        saved = (simulation.position, simulation.steps, list(simulation.path))
        self.assertFalse(simulation.step())
        self.assertEqual(saved, (simulation.position, simulation.steps, simulation.path))
        return simulation

    def test_open_preset_reaches_goal_in_fourteen_moves(self):
        """Verify that the open preset reaches the goal in exactly 14 moves."""
        simulation = self.run_checked(PRESETS["open"])
        self.assertEqual(simulation.status, "reached")
        self.assertEqual(simulation.position, PRESETS["open"].goal)
        self.assertEqual(simulation.steps, 14)

    def test_detour_preset_reaches_goal_and_revisits_cells(self):
        """Verify 26 moves and revisits on the detour preset with the fixed tie order."""
        simulation = self.run_checked(PRESETS["detour"])
        self.assertEqual(simulation.status, "reached")
        self.assertEqual(simulation.steps, 26)
        self.assertLess(len(simulation.visited), len(simulation.path))

    def test_blocked_preset_stops_as_unreachable(self):
        """Verify that an enclosed goal causes an unreachable stop before visiting it."""
        simulation = self.run_checked(PRESETS["blocked"])
        self.assertEqual(simulation.status, "unreachable")
        self.assertIsNone(simulation.distances[simulation.position])
        self.assertNotIn(PRESETS["blocked"].goal, simulation.visited)

    def test_hidden_wall_stays_unknown_until_an_endpoint_is_visited(self):
        """Verify that a hidden wall is observed only after visiting either endpoint."""
        a, b = (3, 2), (3, 3)
        maze = Maze("Hidden wall", "", 4, frozenset({(a, b)}))
        simulation = Simulation(maze)
        self.assertEqual(simulation.known.edge_state(a, (0, 1)), UNKNOWN)
        for _ in range(4 * maze.size ** 4):
            if a in simulation.visited or b in simulation.visited:
                break
            self.assertEqual(simulation.known.edge_state(a, (0, 1)), UNKNOWN)
            self.assertTrue(simulation.step())
        self.assertTrue(a in simulation.visited or b in simulation.visited)
        self.assertEqual(simulation.known.edge_state(a, (0, 1)), WALL)
        self.assertEqual(simulation.known.edge_state(b, (0, -1)), WALL)

    def test_walls_and_observations_are_symmetric(self):
        """Verify shared-edge symmetry and rejection of contradictory observations."""
        maze = Maze("Symmetry", "", 3, frozenset({((1, 0), (0, 0))}))
        self.assertEqual(maze.sense((0, 0))[(1, 0)], WALL)
        self.assertEqual(maze.sense((1, 0))[(-1, 0)], WALL)
        known = KnownMaze(maze.size, maze.start, maze.goal)
        known.record((0, 0), maze.sense((0, 0)))
        self.assertEqual(known.edge_state((1, 0), (-1, 0)), WALL)
        self.assertEqual(known.edge_state((0, 1), (0, -1)), OPEN)
        with self.assertRaises(ValueError):
            known.record((1, 0), {(-1, 0): OPEN})

    def test_start_at_goal_finishes_without_moving(self):
        """Verify zero-move completion when start equals goal in 1x1 and 3x3 mazes."""
        for size, cell in ((1, (0, 0)), (3, (1, 1))):
            with self.subTest(size=size):
                maze = Maze("Already there", "", size, start=cell, goal=cell)
                simulation = self.run_checked(maze)
                self.assertEqual(simulation.status, "reached")
                self.assertEqual(simulation.steps, 0)
                self.assertEqual(simulation.path, [cell])
                self.assertEqual(simulation.distances[cell], 0)

    def test_enclosed_start_stops_without_moving(self):
        """Verify that an enclosed start is classified as unreachable without moving."""
        walls = frozenset({edge_key((0, 0), (1, 0)), edge_key((0, 0), (0, 1))})
        simulation = self.run_checked(Maze("Enclosed start", "", 3, walls))
        self.assertEqual(simulation.status, "unreachable")
        self.assertEqual(simulation.steps, 0)

    def test_out_of_map_goal_is_rejected(self):
        """Verify that Maze and flood_fill reject goals beyond each map boundary."""
        for goal in ((-1, 0), (0, -1), (3, 0), (0, 3)):
            with self.subTest(goal=goal):
                with self.assertRaises(ValueError):
                    Maze("Invalid goal", "", 3, goal=goal)
                known = KnownMaze(3, (0, 0), (2, 2))
                with self.assertRaises(ValueError):
                    flood_fill(known, goal)

    def test_fully_known_distances_match_independent_dijkstra(self):
        """Verify fully known flood-fill distances against Dijkstra in 24 seeded cases."""
        # Five paired size/probability settings; extreme probabilities repeat maps.
        for seed in range(24):
            size = 2 + seed % 5
            probability = (0.0, 0.15, 0.35, 0.6, 1.0)[seed % 5]
            with self.subTest(seed=seed, size=size, walls=probability):
                maze = random_maze(seed, size, probability)
                known = KnownMaze(size, maze.start, maze.goal)
                # Equal-distance comparison requires both planners to know all walls.
                for y in range(size):
                    for x in range(size):
                        known.record((x, y), maze.sense((x, y)))
                self.assertEqual(flood_fill(known, maze.goal), actual_distances(maze))

    def test_exploration_reaches_exactly_the_reachable_random_goals(self):
        """Verify exploration outcomes against true reachability in 30 seeded cases."""
        outcomes = set()
        # One seeded case per size/probability pair, not exhaustive maze coverage.
        for seed in range(30):
            size = 2 + seed % 5
            probability = (0.05, 0.2, 0.35, 0.5, 0.75, 1.0)[seed % 6]
            with self.subTest(seed=seed, size=size, walls=probability):
                maze = random_maze(seed, size, probability)
                reachable = actual_distances(maze)[maze.start] is not None
                simulation = self.run_checked(maze)
                self.assertEqual(simulation.status, "reached" if reachable else "unreachable")
                if reachable:
                    self.assertEqual(simulation.position, maze.goal)
                outcomes.add(reachable)
        self.assertEqual(outcomes, {False, True})


if __name__ == "__main__":
    unittest.main()
