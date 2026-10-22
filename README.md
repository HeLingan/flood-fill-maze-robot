[English](README.md) | [中文](README.zh.md) | [Deutsch](README.de.md)

# Flood Fill Maze Robot

A Python robot simulator that explores an unknown 8×8 grid maze using local wall sensing and flood-fill path planning implemented as reverse breadth-first search (BFS) from the goal.

Maintainer: Lingan He. Developed with AI assistance.

## How it works

- `Maze` holds the complete layout; `KnownMaze` stores observed edges. The robot initially knows the dimensions, start, goal and outer boundary, and senses only the four edges of its current cell.
- Planning treats unknown edges as traversable. After each observation, reverse BFS recomputes the full distance field on this optimistic map; bidirectional edges and unit movement costs allow searching from the goal.
- Each move crosses a confirmed `OPEN` edge to a neighbor whose distance is one lower. Ties follow east, south, west, north.
- Exploration stops at the goal or when the current cell has no route even on the optimistic map, proving unreachability under the model assumptions.

## Project structure

| File | Responsibility |
| --- | --- |
| `main.py` | Application entry point and Tk startup diagnostics. |
| `maze.py` | True/known maps, local sensing and three presets. |
| `planner.py` | Reverse BFS distance computation. |
| `simulator.py` | Robot state and sense–plan–move cycle. |
| `ui.py` | Tkinter visualization, controls and timer. |
| `test_robot.py` | Behavioral tests and independent Dijkstra reference. |
| `README.md`, `README.zh.md`, `README.de.md` | English, Chinese and German documentation. |
| `LICENSE` | MIT license terms. |
| `.gitignore` | Exclusions for local files and Python caches. |

## Requirements & Usage

Python 3.9+; standard library only. The GUI requires Tkinter, a working Tcl/Tk installation and a graphical desktop. Run from the repository directory:

```sh
python3 main.py
```

On Windows, `py` can replace `python3`. Controls: **Step**, **Run/Pause**, **Reset** and preset selection.

- `open`: unobstructed navigation.
- `detour`: discovery of a hidden dead end, backtracking and replanning.
- `blocked`: detection of an unreachable, enclosed goal.

## Tests

```sh
python3 -m unittest -v
```

Ten test methods cover preset outcomes, local information boundaries, edge symmetry, contradictory observations, coincident start/goal, an enclosed start, invalid goals and terminal-state behavior. Per-step checks verify safe movement and consistent state. Two methods include 24 seeded subcases comparing fully known distances against independent Dijkstra results and 30 seeded subcases comparing exploration outcomes against true reachability. Subcases are not necessarily distinct mazes. GUI behavior is not covered by this suite.

## Limitations

Discrete grid simulation with static walls and ideal, noise-free sensors. No physical hardware, motors or kinematic model. The first exploration route is not guaranteed to be globally shortest; reaching the goal does not imply complete mapping.

## License

[MIT](LICENSE) — Copyright © 2026 HeLingan.
