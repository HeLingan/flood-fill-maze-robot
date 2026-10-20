"""用 Tkinter 显示机器人已发现的迷宫，支持单步和定时运行。

对应 README 第 12 节。可以先看文件末尾的 launch()，再顺着
MazeApp.__init__() → _build_widgets() → _reset() 阅读启动过程。
点击 Step 时的调用顺序是 _step() → Simulation.step() → _refresh()。

这里负责按钮、文字和绘图；探测与移动由 simulator.py 负责，距离计算由
planner.py 负责。画图只能读取机器人已知的信息，不能提前显示真实的隐藏墙。
"""

import tkinter as tk
from tkinter import ttk

from maze import DIRECTIONS, PRESETS
from simulator import Simulation


class MazeApp:
    """把一个窗口的控件、显示状态和自动播放控制集中在同一个对象中。

    self.sim 保存当前 Simulation 对象；self.running 表示是否自动运行。
    方法名前的单下划线表示“主要供类内部使用”的约定，并不是权限限制。
    """

    # 毫秒：500 ms = 0.5 s，表示每次 after() 从预约到回调触发的约等待时间。
    # 下一次预约是在本步移动和重画之后设置的，因此相邻两步的实际间隔还包含
    # 这些处理耗时及事件循环可能带来的延迟；这不是实体机器人的速度控制模型。
    TICK_MS = 500

    def __init__(self, root):
        """接收已创建的主窗口，准备控件，并载入第一张地图。"""
        self.root = root
        self.running = False
        # after() 返回的预约标识保存在 timer 中，便于暂停时取消；None 表示无预约。
        self.timer = None
        self.preset_keys = list(PRESETS)
        # StringVar 是与 Tk 控件连接的文字变量，而不是普通 Python 字符串。
        # 控件会显示它的值；用 .get() 读取，用 .set(...) 更新界面上的文字。
        self.map_name = tk.StringVar(value=self.preset_keys[0])
        self.stats = tk.StringVar()
        self.event = tk.StringVar()
        root.title("Flood Fill Maze Robot")
        root.geometry("640x790")
        root.minsize(420, 650)
        # 把关闭按钮交给 _close 处理，确保先取消定时器再销毁窗口。
        root.protocol("WM_DELETE_WINDOW", self._close)
        # 必须先建按钮和画布；随后 _reset 会创建模拟器，并更新这些控件。
        self._build_widgets()
        self._reset()

    def _build_widgets(self):
        """创建标题、地图选择框、操作按钮、画布、图例和状态文字。"""
        # ttk.Style 统一管理外观。带点的名称（如 Title.TLabel）用于特定标签样式。
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background="#f4f6fa")
        style.configure("TLabel", background="#f4f6fa", foreground="#26364a")
        style.configure("Title.TLabel", font=("Helvetica", 20, "bold"))
        style.configure("Muted.TLabel", foreground="#64748b")
        style.configure("TButton", padding=(12, 8))

        # grid 是界面控件的布局网格，不是机器人走的迷宫网格。
        # weight=1 让这一行/列分到窗口增大的空间；sticky 指定向哪些方向贴齐。
        # n/s/e/w 分别是上/下/右/左，"nsew" 表示填满分配到的区域。
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        body = ttk.Frame(self.root, padding=18)
        body.grid(sticky="nsew")
        body.columnconfigure(0, weight=1)
        # row=4（从 0 开始计数）放画布，所以拉大窗口时主要让迷宫显示区域变大。
        body.rowconfigure(4, weight=1)
        ttk.Label(body, text="Flood Fill Maze Robot", style="Title.TLabel").grid(
            row=0, sticky="w"
        )
        ttk.Label(body, text="Explore. Update distances. Choose the next cell.",
                  style="Muted.TLabel").grid(row=1, sticky="w", pady=(4, 14))

        # Frame 是容器：地图选择区可以拥有自己的行列布局，不影响外层布局。
        selector = ttk.Frame(body)
        selector.grid(row=2, sticky="ew")
        selector.columnconfigure(1, weight=1)
        ttk.Label(selector, text="Maze").grid(row=0, column=0, padx=(0, 10))
        # readonly 让用户只能选预设名，避免手动输入一个不存在的字典键。
        combo = ttk.Combobox(selector, textvariable=self.map_name,
                             values=self.preset_keys, state="readonly", width=18)
        combo.grid(row=0, column=1, sticky="ew")
        # bind 会把事件对象传给回调；lambda 接收它，再调用不需要参数的 _reset。
        # 此处的 event 是一次选择事件，与 self.event（底部的文字变量）不同。
        combo.bind("<<ComboboxSelected>>", lambda event: self._reset())

        # 三列同属 uniform="buttons"，使三个按钮的布局列保持相同宽度。
        controls = ttk.Frame(body)
        controls.grid(row=3, sticky="ew", pady=(12, 10))
        for column in range(3):
            controls.columnconfigure(column, weight=1, uniform="buttons")
        # command=self._step 传入函数，等点击时执行；不能写 self._step()，
        # 否则会在创建按钮时立即走一步，再把返回值当作回调。
        self.step_button = ttk.Button(controls, text="Step", command=self._step)
        self.step_button.grid(row=0, column=0, sticky="ew", padx=(0, 5))
        self.run_button = ttk.Button(controls, text="Run", command=self._toggle_run)
        self.run_button.grid(row=0, column=1, sticky="ew", padx=5)
        ttk.Button(controls, text="Reset", command=self._reset).grid(
            row=0, column=2, sticky="ew", padx=(5, 0)
        )
        # Canvas 用像素坐标画图。500×500 是初始请求大小，布局会随窗口调整它。
        self.canvas = tk.Canvas(body, background="#ffffff", highlightthickness=0,
                                width=500, height=500)
        self.canvas.grid(row=4, sticky="nsew")
        # 画布大小改变时重画；Tk 会把事件对象传入 _draw(event)。
        self.canvas.bind("<Configure>", self._draw)

        # text 是固定文字，textvariable 绑定的文字会随 StringVar.set() 自动更新。
        ttk.Label(body, text="Dotted: unknown edge   |   Dark: known wall",
                  style="Muted.TLabel").grid(row=5, sticky="w", pady=(10, 3))
        ttk.Label(body, text="Numbers: distance to G   |   Blue: robot / visited",
                  style="Muted.TLabel").grid(row=6, sticky="w")
        ttk.Label(body, textvariable=self.stats).grid(row=7, sticky="w", pady=(12, 4))
        self.event_label = ttk.Label(body, textvariable=self.event,
                                     style="Muted.TLabel", wraplength=560)
        self.event_label.grid(row=8, sticky="ew")
        # 窗口缩窄后让提示文字换行；36 对应外层左右各 18 的留白。
        body.bind("<Configure>", lambda event: self.event_label.configure(
            wraplength=max(250, event.width - 36)))

    def _pause(self):
        """停止自动播放，并取消尚未执行的定时预约；不清除机器人进度。"""
        self.running = False
        if self.timer is not None:
            self.root.after_cancel(self.timer)
            # 清空已取消的预约标识，避免以后误用它；重复调用 _pause 也安全。
            self.timer = None

    def _reset(self):
        """为下拉框选中的地图重新创建模拟器，并立即显示初始状态。"""
        # 先取消旧预约，防止上一轮自动运行的回调推进新地图。
        self._pause()
        # 新对象重新初始化位置、已知地图、轨迹和步数；不只是把图像擦掉。
        # Simulation 初始化时已经探测起点周围，并计算第一份距离场。
        self.sim = Simulation(PRESETS[self.map_name.get()])
        self._refresh()

    def _step(self):
        """让模拟器尝试走一步，然后刷新显示；终止时也停止自动播放。"""
        # 这里不计算移动方向：Simulation.step() 会移动、探测并重新规划。
        self.sim.step()
        if self.sim.status != "exploring":
            self._pause()
        self._refresh()

    def _toggle_run(self):
        """响应 Run/Pause：运行中则暂停，仍在探索时则预约第一次自动移动。"""
        if self.running:
            self._pause()
        elif self.sim.status == "exploring":
            self.running = True
            # after 不会让窗口睡眠；它预约约 500 ms 后调用 _tick，并立即返回。
            # 等待期间 mainloop 仍能处理按钮点击；繁忙时实际触发可能更晚。
            self.timer = self.root.after(self.TICK_MS, self._tick)
        self._refresh()

    def _tick(self):
        """执行一次定时移动；只有仍处于运行状态时才预约下一次。"""
        # 当前预约已经触发，不再是一个等待中的预约，先清除它的标识。
        self.timer = None
        if not self.running:
            return
        # _step 可能因到达终点或确认不可达而把 running 设为 False。
        self._step()
        if self.running:
            # 每次只预约一次，不使用 while True 或 sleep 阻塞界面事件循环。
            self.timer = self.root.after(self.TICK_MS, self._tick)

    def _refresh(self):
        """把模拟器状态同步到按钮、统计文字和画布，不推进模拟器。"""
        active = self.sim.status == "exploring"
        # 自动运行时禁用 Step，避免手动推进与自动推进混在一起。
        # reached / unreachable 都是终止状态，此时 Step 和 Run 均禁用。
        self.step_button.configure(state="normal" if active and not self.running
                                   else "disabled")
        self.run_button.configure(text="Pause" if self.running else "Run",
                                  state="normal" if active else "disabled")
        labels = {"exploring": "Exploring", "reached": "Goal reached",
                  "unreachable": "Goal unreachable"}
        # steps 包含回头路；visited 是集合，因此 len(...) 只计不同格子的数量。
        # size ** 2 是正方形迷宫的总格数。format 把这些值填进字符串中的 {}。
        self.stats.set("Steps: {}    Visited: {}/{}    {}".format(
            self.sim.steps, len(self.sim.visited), self.sim.known.size ** 2,
            labels[self.sim.status]))
        self.event.set(self.sim.last_event)
        self._draw()

    def _draw(self, event=None):
        """根据已知地图重画全图，不读取预设中的真实隐藏墙。

        event=None 让本方法既能被 Tk 以事件回调形式调用，也能被
        _refresh() 直接调用。绘图不需要使用事件对象本身。
        """
        canvas = self.canvas
        # 每次清空再重画，避免残留上一步的机器人或旧的距离数字。
        canvas.delete("all")
        size = self.sim.known.size
        width, height = canvas.winfo_width(), canvas.winfo_height()
        # 取较短边保证棋盘是正方形；再减去 28，给该方向两侧各留 14 像素。
        # 刚创建控件时实际尺寸可能很小，下面的检查避免负尺寸参与绘图。
        side = min(width, height) - 28
        if side <= 0:
            return
        # 迷宫的 (x, y) 是格子编号；画布需要像素坐标。先求每格边长，
        # 再求棋盘左上角的偏移 (left, top)，这样棋盘始终居中。
        cell_size = side / size
        left, top = (width - side) / 2, (height - side) / 2

        def center(cell):
            """把格子坐标转换为其中心的画布像素坐标，供轨迹和机器人共用。"""
            x, y = cell
            # x/y 指向格子的左上角，增加半格（0.5）才到中心。
            return left + (x + 0.5) * cell_size, top + (y + 0.5) * cell_size

        # 第一层：格子底色和距离数字。先画背景，后画的图形才不会被它盖住。
        for y in range(size):
            for x in range(size):
                cell = (x, y)
                x0, y0 = left + x * cell_size, top + y * cell_size
                fill = "#edf4ff" if cell in self.sim.visited else "#ffffff"
                if cell == self.sim.known.goal:
                    fill = "#def4e8"
                canvas.create_rectangle(x0, y0, x0 + cell_size, y0 + cell_size,
                                        fill=fill, outline="")
                # 数字来自当前已知地图上的距离场；未去过的格子也可以有数字。
                # 它们是基于未知边暂按可通行处理的推算，不代表已经探测过。
                # None 表示在当前已知墙的约束下无法通往终点，显示成无穷大。
                distance = self.sim.distances.get(cell)
                canvas.create_text(x0 + 6, y0 + 5, anchor="nw",
                                   text="∞" if distance is None else str(distance),
                                   fill="#73849a", font=("Helvetica",
                                   max(8, int(cell_size * 0.18))))

        # 第二层：把完整的行走历史连接起来，所以回头走过的线段可能重叠。
        if len(self.sim.path) > 1:
            # 两层 for 把 [(cx, cy), ...] 展开为 [cx, cy, ...]；*points
            # 再把这串坐标展开为 create_line 的多个位置参数。
            points = [coordinate for cell in self.sim.path for coordinate in center(cell)]
            canvas.create_line(*points, fill="#c4d8f4", width=2)
        gx, gy = center(self.sim.known.goal)
        canvas.create_text(gx, gy, text="G", fill="#218451",
                           font=("Helvetica", max(12, int(cell_size * 0.34)), "bold"))

        # 第三层：画边。查询 known 而非 PRESETS 的真实墙，保证不泄露隐藏地图。
        for y in range(size):
            for x in range(size):
                # 相邻格子共享一条边，只画每格的东边和南边即可覆盖所有内部边。
                # 西边仅在 x==0 时补画，北边仅在 y==0 时补画，补齐外边界。
                for direction in DIRECTIONS:
                    dx, dy = direction
                    # and 的优先级高于 or；下式等价于：
                    # (dx == -1 and x != 0) or (dy == -1 and y != 0)。
                    # 也就是跳过非最左列的西边，或非最上行的北边，避免重复画线。
                    if dx == -1 and x != 0 or dy == -1 and y != 0:
                        continue
                    state = self.sim.known.edge_state((x, y), direction)
                    # 此处的 "open" / "wall" 与 maze.py 的 OPEN / WALL 常量
                    # 分别是相同的字符串；这里直接写字符串进行比较。
                    # 已知开放边不画线；wall 画深色实线；unknown 画浅色虚线。
                    if state == "open":
                        continue
                    x0, y0 = left + x * cell_size, top + y * cell_size
                    # 朝东/西（dx 非零）对应一条竖线；朝南/北对应一条横线。
                    # points 是线段的两个端点：(起点 x, 起点 y, 终点 x, 终点 y)。
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

        # 最后一层：用朝向向量 (dx, dy) 画三角形，表示机器人位置和前进方向。
        # 顶点在中心前方 radius 处；另两个点在后方 0.7*radius 处，
        # 沿垂直向量 (-dy, dx) 向两侧各偏移 0.65*radius，形成三角形底边。
        # 这只是显示方向，不模拟实体机器人的转向时间、轮速或运动动力学。
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
        """先取消尚未执行的定时任务，再销毁主窗口并结束其事件循环。"""
        self._pause()
        self.root.destroy()


def launch():
    """创建主窗口和 MazeApp，然后开始处理点击、定时器与重绘事件。"""
    root = tk.Tk()
    MazeApp(root)
    # mainloop 持续处理事件，窗口关闭后才返回；导入本文件本身不会调用 launch。
    root.mainloop()
