"""Tkinter view of the robot's discovered maze, with manual or timed steps."""

import tkinter as tk
from tkinter import ttk

from maze import DIRECTIONS, PRESETS
from simulator import Simulation


class MazeApp:
    """Keep display and playback separate from the simulation logic."""

    TICK_MS = 500

    def __init__(self, root):
        self.root = root
        self.running = False
        self.timer = None
        self.preset_keys = list(PRESETS)
        self.map_name = tk.StringVar(value=self.preset_keys[0])
        self.stats = tk.StringVar()
        self.event = tk.StringVar()
        root.title("Flood Fill Maze Robot")
        root.geometry("640x790")
        root.minsize(420, 650)
        root.protocol("WM_DELETE_WINDOW", self._close)
        self._build_widgets()
        self._reset()

    def _build_widgets(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background="#f4f6fa")
        style.configure("TLabel", background="#f4f6fa", foreground="#26364a")
        style.configure("Title.TLabel", font=("Helvetica", 20, "bold"))
        style.configure("Muted.TLabel", foreground="#64748b")
        style.configure("TButton", padding=(12, 8))

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        body = ttk.Frame(self.root, padding=18)
        body.grid(sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.rowconfigure(4, weight=1)
        ttk.Label(body, text="Flood Fill Maze Robot", style="Title.TLabel").grid(
            row=0, sticky="w"
        )
        ttk.Label(body, text="Explore. Update distances. Choose the next cell.",
                  style="Muted.TLabel").grid(row=1, sticky="w", pady=(4, 14))

        selector = ttk.Frame(body)
        selector.grid(row=2, sticky="ew")
        selector.columnconfigure(1, weight=1)
        ttk.Label(selector, text="Maze").grid(row=0, column=0, padx=(0, 10))
        combo = ttk.Combobox(selector, textvariable=self.map_name,
                             values=self.preset_keys, state="readonly", width=18)
        combo.grid(row=0, column=1, sticky="ew")
        combo.bind("<<ComboboxSelected>>", lambda event: self._reset())

        controls = ttk.Frame(body)
        controls.grid(row=3, sticky="ew", pady=(12, 10))
        for column in range(3):
            controls.columnconfigure(column, weight=1, uniform="buttons")
        self.step_button = ttk.Button(controls, text="Step", command=self._step)
        self.step_button.grid(row=0, column=0, sticky="ew", padx=(0, 5))
        self.run_button = ttk.Button(controls, text="Run", command=self._toggle_run)
        self.run_button.grid(row=0, column=1, sticky="ew", padx=5)
        ttk.Button(controls, text="Reset", command=self._reset).grid(
            row=0, column=2, sticky="ew", padx=(5, 0)
        )
        self.canvas = tk.Canvas(body, background="#ffffff", highlightthickness=0,
                                width=500, height=500)
        self.canvas.grid(row=4, sticky="nsew")
        self.canvas.bind("<Configure>", self._draw)

        ttk.Label(body, text="Dotted: unknown edge   |   Dark: known wall",
                  style="Muted.TLabel").grid(row=5, sticky="w", pady=(10, 3))
        ttk.Label(body, text="Numbers: distance to G   |   Blue: robot / visited",
                  style="Muted.TLabel").grid(row=6, sticky="w")
        ttk.Label(body, textvariable=self.stats).grid(row=7, sticky="w", pady=(12, 4))
        self.event_label = ttk.Label(body, textvariable=self.event,
                                     style="Muted.TLabel", wraplength=560)
        self.event_label.grid(row=8, sticky="ew")
        body.bind("<Configure>", lambda event: self.event_label.configure(
            wraplength=max(250, event.width - 36)))

    def _pause(self):
        self.running = False
        if self.timer is not None:
            self.root.after_cancel(self.timer)
            self.timer = None

    def _reset(self):
        self._pause()
        self.sim = Simulation(PRESETS[self.map_name.get()])
        self._refresh()

    def _step(self):
        self.sim.step()
        if self.sim.status != "exploring":
            self._pause()
        self._refresh()

    def _toggle_run(self):
        if self.running:
            self._pause()
        elif self.sim.status == "exploring":
            self.running = True
            self.timer = self.root.after(self.TICK_MS, self._tick)
        self._refresh()

    def _tick(self):
        self.timer = None
        if not self.running:
            return
        self._step()
        if self.running:
            self.timer = self.root.after(self.TICK_MS, self._tick)

    def _refresh(self):
        active = self.sim.status == "exploring"
        self.step_button.configure(state="normal" if active and not self.running
                                   else "disabled")
        self.run_button.configure(text="Pause" if self.running else "Run",
                                  state="normal" if active else "disabled")
        labels = {"exploring": "Exploring", "reached": "Goal reached",
                  "unreachable": "Goal unreachable"}
        self.stats.set("Steps: {}    Visited: {}/{}    {}".format(
            self.sim.steps, len(self.sim.visited), self.sim.known.size ** 2,
            labels[self.sim.status]))
        self.event.set(self.sim.last_event)
        self._draw()

    def _draw(self, event=None):
        """Draw only observed walls; never access the preset's true wall map."""
        canvas = self.canvas
        canvas.delete("all")
        size = self.sim.known.size
        width, height = canvas.winfo_width(), canvas.winfo_height()
        side = min(width, height) - 28
        if side <= 0:
            return
        cell_size = side / size
        left, top = (width - side) / 2, (height - side) / 2

        def center(cell):
            x, y = cell
            return left + (x + 0.5) * cell_size, top + (y + 0.5) * cell_size

        for y in range(size):
            for x in range(size):
                cell = (x, y)
                x0, y0 = left + x * cell_size, top + y * cell_size
                fill = "#edf4ff" if cell in self.sim.visited else "#ffffff"
                if cell == self.sim.known.goal:
                    fill = "#def4e8"
                canvas.create_rectangle(x0, y0, x0 + cell_size, y0 + cell_size,
                                        fill=fill, outline="")
                distance = self.sim.distances.get(cell)
                canvas.create_text(x0 + 6, y0 + 5, anchor="nw",
                                   text="∞" if distance is None else str(distance),
                                   fill="#73849a", font=("Helvetica",
                                   max(8, int(cell_size * 0.18))))

        if len(self.sim.path) > 1:
            points = [coordinate for cell in self.sim.path for coordinate in center(cell)]
            canvas.create_line(*points, fill="#c4d8f4", width=2)
        gx, gy = center(self.sim.known.goal)
        canvas.create_text(gx, gy, text="G", fill="#218451",
                           font=("Helvetica", max(12, int(cell_size * 0.34)), "bold"))

        for y in range(size):
            for x in range(size):
                # East and south draw each interior edge once. Add top and left borders.
                for direction in DIRECTIONS:
                    dx, dy = direction
                    if dx == -1 and x != 0 or dy == -1 and y != 0:
                        continue
                    state = self.sim.known.edge_state((x, y), direction)
                    if state == "open":
                        continue
                    x0, y0 = left + x * cell_size, top + y * cell_size
                    if dx:
                        edge_x = x0 + (cell_size if dx == 1 else 0)
                        points = (edge_x, y0, edge_x, y0 + cell_size)
                    else:
                        edge_y = y0 + (cell_size if dy == 1 else 0)
                        points = (x0, edge_y, x0 + cell_size, edge_y)
                    canvas.create_line(*points,
                                       fill="#293a50" if state == "wall" else "#c2cbd7",
                                       width=3 if state == "wall" else 1,
                                       dash=() if state == "wall" else (3, 4))

        cx, cy = center(self.sim.position)
        dx, dy = self.sim.heading
        radius = cell_size * 0.28
        points = (cx + dx * radius, cy + dy * radius,
                  cx - dx * radius * 0.7 - dy * radius * 0.65,
                  cy - dy * radius * 0.7 + dx * radius * 0.65,
                  cx - dx * radius * 0.7 + dy * radius * 0.65,
                  cy - dy * radius * 0.7 - dx * radius * 0.65)
        canvas.create_polygon(*points, fill="#2563eb", outline="#1749b4", width=1)

    def _close(self):
        self._pause()
        self.root.destroy()


def launch():
    root = tk.Tk()
    MazeApp(root)
    root.mainloop()
