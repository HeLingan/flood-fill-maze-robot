# Flood Fill Maze Robot

A small Python simulation of a robot exploring an unknown maze. The window shows the robot's map, its route, and the flood-fill distances to the goal.

This is a learning project developed with AI assistance. It uses a classical search algorithm, with no machine learning. It does not control a physical robot or simulate motors, sensor noise, or movement physics.

## Run

You need Python 3 with a working Tkinter runtime and a desktop session. A modern Python/Tk build is recommended. No third-party Python packages or `pip install` commands are needed.

Open a terminal in this project folder. First check Tkinter:

```sh
python3 -m tkinter
```

A small test window should appear. Close it, then start the simulation:

```sh
python3 main.py
```

If Tkinter is missing, use a Python installation that includes Tk support. An old Apple Tk 8.5 runtime produced a blank window during development on macOS. If the window is blank, try a current Python/Tk build and repeat the Tkinter check.

## Controls

- **Maze:** choose `open`, `detour`, or `blocked`. Changing the maze resets the simulation.
- **Step:** move one cell, sense the new cell, and update the distances.
- **Run / Pause:** start or pause automatic steps.
- **Reset:** return to the start with a fresh map.

The blue triangle is the robot. Pale blue cells have been visited. `G` marks the goal. Dark lines are known walls, and dotted lines are unknown edges. Numbers show the current estimated distance to `G`; `∞` means no route exists in the current map.

## How it works

The robot knows the maze size, outer boundaries, start, and goal. Internal edges begin as **unknown**. An ideal sensor reveals only the four edges of the robot's current cell. The planner and display use this discovered map, not the hidden wall layout.

After each observation:

1. Record the four local edges as open or wall.
2. Treat unknown edges as open for planning.
3. Run breadth-first search (BFS) backwards from the goal to calculate a distance for each cell.
4. Move through a sensed-open edge to a cell whose distance is one smaller.
5. Sense again and recalculate the complete distance field.

Each move costs one. Movement is limited to east, south, west, and north, in that order when several choices have the same distance. There are no diagonal moves or turn costs.

Treating unknown edges as open makes the distances optimistic. New walls can increase them and force the robot to backtrack. The first exploration route is **not necessarily the globally shortest route**. BFS gives shortest distances on the current optimistic map; it gives the true shortest distances when the full map is known.

The robot stops when it reaches the goal, or when observed walls prove that no route to the goal exists even with all remaining unknown edges treated as open.

## Included mazes

All three presets use an 8 × 8 grid. With the fixed direction order above:

| Preset | What it demonstrates | Result | Moves |
| --- | --- | --- | ---: |
| `open` | No internal walls | Goal reached | 14 |
| `detour` | A hidden dead end and backtracking | Goal reached | 26 |
| `blocked` | An enclosed goal | Goal reported unreachable | 15 |

The last row counts moves made before stopping, not a path to the goal.

## Tests

Run the tests from this folder:

```sh
python3 -m unittest -v
```

The tests run without opening a GUI and do not require Tkinter. They check local sensing, safe moves, shared wall observations, preset outcomes, and special cases such as an enclosed start. They also compare fully known flood-fill distances with an independent Dijkstra implementation and check exploration results on deterministic random mazes.

These checks cover the discrete simulation. They do not validate a physical robot or compare planning speed with other algorithms.

## Files

| File | Purpose |
| --- | --- |
| `main.py` | Start the application and report Tkinter startup errors |
| `maze.py` | Maze data, local sensing, discovered map, and three presets |
| `planner.py` | Reverse BFS flood-fill distance calculation |
| `simulator.py` | Sense, plan, and move loop |
| `ui.py` | Tkinter window and playback controls |
| `test_robot.py` | Simulation and planner tests |
| `README.md` | Setup, algorithm, and limitations |
| `LICENSE` | MIT License |
| `.gitignore` | Ignore Python caches and local environment files |

## License

MIT License. Copyright © 2026 HeLingan.
