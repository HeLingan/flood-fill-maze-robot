"""组织机器人每一步的“感知 → 规划 → 移动”过程。

建议结合 README 第 7 节的实际路线和第 11 节的状态表阅读。
maze.py 提供真实地图和已知地图，planner.py 负责计算距离；本文件把它们
组合起来，决定机器人何时移动、移动到哪里、何时停止。

这里采用离散模拟：一次移动恰好进入相邻一格，不模拟电机、惯性或传感器
噪声，也没有训练出来的 AI 模型。界面的 Step 和 Run 最终都会调用 step()。
"""

from maze import DIRECTIONS, KnownMaze, Maze, OPEN, UNKNOWN, WALL, neighbor
from planner import flood_fill


class Simulation:
    """保存一次迷宫探索的全部状态，并提供“走一步”的入口。

    创建对象时传入一张真实 Maze；机器人通过局部感知逐步建立 known 地图。
    distances 只根据 known 计算，规划器不会获得真实地图中的隐藏墙壁。
    ui.py 读取这里的位置、轨迹、状态和距离来画图，不负责选择路线。
    """

    def __init__(self, maze: Maze):
        """用真实地图 maze 创建一轮探索，并先感知、规划起点。

        __init__ 是创建 Simulation(maze) 时自动执行的初始化方法，不返回
        探索结果。它建立下列 self 属性；self 表示当前这个模拟对象。
        初始化不计移动步数，但可能立即发现“起点就是终点”或“起点无路可走”。
        """
        # 保存的是方法本身，注意这里没有写 maze.sense(...) 来立即调用它。
        # 这个“绑定方法”记得自己所属的真实地图，以后只需传入机器人坐标。
        # 它是局部感知入口：每次只提供当前格四边的信息，不把整张地图交给规划器。
        self._sense = maze.sense
        # 尺寸、起点、终点是已知条件；内部的墙和通路要靠后续感知来发现。
        self.known = KnownMaze(maze.size, maze.start, maze.goal)
        self.position = maze.start
        # 坐标采用 (x, y)，(1, 0) 表示向东，即画面向右；朝向用于画机器人。
        self.heading = (1, 0)
        self.steps = 0
        # 集合 visited 自动去重，回答“去过哪些格子”；它不是禁止回头的名单。
        self.visited = {self.position}
        # 列表 path 按顺序保存每一次位置，允许重复；第一项是尚未移动时的起点。
        # 因此始终有 steps == len(path) - 1，可用来核对统计是否一致。
        self.path = [self.position]
        # 三种状态：exploring 正在探索，reached 已到终点，unreachable 确认不可达。
        self.status = "exploring"
        # last_event 是给界面显示的说明文字；真正控制流程的是 status。
        self.last_event = "Start: sensed the four edges of the starting cell."
        # 先探测起点的出口并生成 distances，保证第一次 step() 已有选择依据。
        self._observe_and_plan()

    def _observe_and_plan(self) -> int:
        """在当前位置感知四边、更新地图、重算距离，并检查终止条件。

        输入来自对象的 position 和 _sense；结果写回 known、distances，以及
        必要时的 status、last_event。返回值 int 是本次新发现的墙数，只用于
        显示提示，不是步数或距离。调用这个方法本身不会移动机器人。

        名称开头的单下划线表示“主要供本类内部使用”的约定，不是权限隔离。
        """
        # observation 是“方向 → 观测状态”的字典，当前四边都会得到 OPEN 或 WALL。
        observation = self._sense(self.position)
        # 必须在 record() 之前统计，否则 UNKNOWN 已被新观测覆盖，无法判断“新发现”。
        # 只有“本次有墙，而且之前未知”才计数；已知的外边界和重复见到的墙不计。
        # sum() 在这里将每次判断的 True 当作 1、False 当作 0，相加得到新墙数量。
        new_walls = sum(
            state == WALL and self.known.edge_state(self.position, direction) == UNKNOWN
            for direction, state in observation.items()
        )
        # 保存四边观测。KnownMaze 会让同一条边从相邻两格查询时得到一致结果。
        self.known.record(self.position, observation)
        # 用新的已知地图重新做一次洪水填充，替换整张旧距离表，而非只修改当前格。
        # 规划时 UNKNOWN 乐观地按可通行处理；实际移动仍必须使用已确认 OPEN 的边。
        self.distances = flood_fill(self.known, self.known.goal)
        # 到达终点优先判断：起点等于终点时，初始化也能正确以 0 步结束。
        if self.position == self.known.goal:
            self.status = "reached"
            self.last_event = "Goal reached. The first exploration route need not be shortest."
        # None 表示没有路径；数字 0 则表示就在终点，两者含义不同。
        # 连“所有未知边都能通行”的乐观地图都没有路，真实地图就更不可能有路。
        # 这是一种正常结束，不需要为了确认不可达而探测迷宫的每一条边。
        elif self.distances[self.position] is None:
            self.status = "unreachable"
            self.last_event = "Observed walls prove the goal is unreachable. Robot stopped."
        # 两个终止条件均未满足时，保持原来的 exploring 状态，等待下一次 step()。
        return new_walls

    def step(self) -> bool:
        """最多移动一格，然后在新位置感知并重新规划。

        没有额外参数，使用对象当前的地图、位置和距离表。返回 True 表示这次
        确实移动了一格，包括恰好到达终点的最后一步；并不表示“还会继续探索”。
        已经结束时返回 False，不改变位置、轨迹或步数。

        阅读重点是距离表的时间顺序：先用移动前的旧表选择邻居，移动后才生成
        新表。发现墙后，新表中的当前位置距离可能变大，README 7.2 有具体例子。
        """
        # 到达终点或确认不可达后，无论再调用多少次，都不继续移动。
        if self.status != "exploring":
            return False
        # exploring 状态下，这个值应为有限整数；None 已在上次感知时触发停止。
        # 这里和下面的邻居距离均来自同一张“移动前”的距离表。
        current_distance = self.distances[self.position]
        # 固定按东、南、西、北检查；若多个方向都满足条件，就取第一个。
        # 这解释了空地图中为什么先向右走，也使同一地图的演示路线可重复。
        for direction in DIRECTIONS:
            # 先计算候选邻居坐标；此时尚未更新 position，也尚未真的移动。
            other = neighbor(self.position, direction)
            # 两个条件必须同时成立：① 当前边已探测为通路；② 邻居距离正好小 1。
            # UNKNOWN 只在规划中假定可走，不能作为实际跨越一条边的依据。
            # and 会短路：边不开放时不再计算右侧条件；执行右侧时，边已确认 OPEN，
            # 所以相邻格必然在图内。get() 对不存在的键默认返回 None 是一层保险，
            # 并不是这里用来检测越界的手段；外边界已由 edge_state() 返回 WALL 阻止跨越。
            if (self.known.edge_state(self.position, direction) == OPEN
                    and self.distances.get(other) == current_distance - 1):
                # 先检查是否回访，再写入集合，否则每一次都会被判断为“已经访问过”。
                # 回访只影响提示文字，不影响路线选择；回头路由重新计算的距离自然产生。
                # 因此没有单独的“倒退模式”，也不会因为去过一个格子就拒绝再次进入。
                revisiting = other in self.visited
                # 选定邻居后才更新位置与朝向；一次成功移动使 steps 恰好加 1。
                self.position, self.heading = other, direction
                self.steps += 1
                # set.add() 保存不重复的访问记录；list.append() 保留本次移动的完整顺序。
                self.visited.add(other)
                self.path.append(other)
                # 到达新格后才探测它的四边：这一步可能发现隐藏墙，并改变整张距离表。
                # 例如 detour 第 12 步按旧表从 3 走到 2，重算后新位置的距离却变成 14。
                # 下一次 step() 使用这张新表，便可能选择回到刚刚来过的格子。
                new_walls = self._observe_and_plan()
                # 只在仍探索时写普通移动提示；保留上面刚生成的“到达/不可达”终止提示。
                if self.status == "exploring":
                    action = "Revisited" if revisiting else "Entered"
                    # format() 按顺序填入动作、横坐标、纵坐标和新墙数，供界面展示。
                    self.last_event = (
                        "{} cell ({}, {}); found {} new wall(s); recalculated distances."
                        .format(action, other[0], other[1], new_walls)
                    )
                # 立即结束本次调用，保证一次 step() 最多移动一格，不继续尝试其他方向。
                return True
        # 正常情况下不会来到这里：当前四边已经感知，且当前 BFS 距离有限，
        # 就一定至少有一个已确认开放、距离小 1 的邻居。若找不到，说明内部数据
        # 或算法的一致性被破坏；正常的不可达已由 status 处理，不会用这个异常表示。
        raise RuntimeError("No sensed-open downhill neighbor; map invariant broken")
