"""项目的自动测试；对应 README 第 14 节，不需要打开图形窗口。

在项目文件夹运行 ``python3 -m unittest -v``（Windows 可用 ``py``
代替 ``python3``）。``-v`` 显示各项测试名称；也可在 ``-m`` 前加上
可选参数 ``-B``，表示不写入字节码缓存。
unittest 会发现 RobotTests 中以 ``test_`` 开头的 10 个测试方法；辅助
方法不会单独算作测试。后两项还包含 24 和 30 个固定随机种子子案例。

测试的基本形式是“准备输入 → 运行被测代码 → 断言结果符合预期”。
断言失败就说明某条约定没有满足，需要阅读失败位置，而不是删掉断言。
为了独立核对距离，下面另写了读取真实地图的 Dijkstra 算法；它不调用
项目的 flood_fill，也不使用机器人的已知地图。测试能看真实地图，只为
核对机器人是否正确，并没有将完整地图交给运行中的规划器。

这些测试覆盖这里的离散模型和具体案例，不等于数学证明，也不检查
所有界面布局、按钮操作或实体机器人的传感器与电机。
"""

# 三个模块都来自 Python 标准库，不需要额外安装：
# heapq 提供最小堆，random 生成可复现的测试数据，unittest 组织测试。
import heapq
import random
import unittest

from maze import DIRECTIONS, KnownMaze, Maze, OPEN, PRESETS, UNKNOWN, WALL, edge_key
from planner import flood_fill
from simulator import Simulation


def actual_distances(maze):
    """用真实地图独立计算每格到终点的最短步数，作为测试参照。

    输入 maze 是完整的真实迷宫；返回“坐标 → 步数或 None”的字典。
    计算结束后仍为 None 的格子与终点不连通。这里的 Dijkstra 使用
    优先队列，始终优先取出当前距离最小的格子；planner.py 的 BFS 则
    使用先进先出的普通队列。由于本项目每次移动的代价都是 1，两者在
    同一张完整地图上应得到相同距离，但实现不同，有助于相互核对。
    """
    # 刚开始 None 只表示“尚未发现路径”；不能立刻判定该格不可达。
    result = {(x, y): None for y in range(maze.size) for x in range(maze.size)}
    # 从终点反向扩展，所以终点到自身的距离是 0。
    result[maze.goal] = 0
    # 堆内每项是 (距离, 坐标)，heapq 会先比较元组的第一个值。
    frontier = [(0, maze.goal)]
    while frontier:
        distance, cell = heapq.heappop(frontier)
        # 若曾为同一格加入较长候选，之后又找到了更短路径，较长候选
        # 可能还留在堆中。这种过期记录不能继续用于扩展；本项目的
        # 等代价图按此实现不会产生它，保留这个检查是 Dijkstra 的通用写法。
        if result[cell] != distance:
            continue
        x, y = cell
        # 独立枚举四邻居，不调用 planner.py 的邻居遍历或距离函数。
        for other in ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)):
            # 第一道检查：坐标必须在地图内，不能跨越外边界。
            if not (0 <= other[0] < maze.size and 0 <= other[1] < maze.size):
                continue
            # 第二道检查：真实地图中，两格之间不能有墙。
            # 对端点排序，让 (A, B) 与 (B, A) 指向同一条无向边。
            if tuple(sorted((cell, other))) in maze.walls:
                continue
            # 经过当前格再走一步，是到邻居的一条候选路径。
            candidate = distance + 1
            # 只在首次发现或确实更短时更新；这个步骤叫“松弛”。
            # 把改进后的距离入堆，稍后继续从这个邻居向外扩展。
            if result[other] is None or candidate < result[other]:
                result[other] = candidate
                heapq.heappush(frontier, (candidate, other))
    return result


def random_maze(seed, size, wall_probability):
    """按给定种子、边长和墙概率生成可复现的测试迷宫。

    seed 是随机种子；相同参数会重建相同地图，便于复现失败案例。
    wall_probability 为一条内部边成为墙的概率：0 是全开，1 是全封。
    返回一个 Maze；起点和终点使用 Maze 的默认设置。
    """
    # 使用独立随机数生成器，不改变其他代码使用的全局随机数状态。
    rng = random.Random(seed)
    walls = set()
    for y in range(size):
        for x in range(size):
            # 每格只考虑东、南两边，所有内部边就各检查一次。
            # 若再考虑西、北，同一条共享边会被重复随机决定。
            for other in ((x + 1, y), (x, y + 1)):
                # 排除最右列的东侧、最下行的南侧；外边界由 Maze 处理。
                if other[0] < size and other[1] < size:
                    # random() 的结果在 [0, 1) 内，小于概率时就加一堵墙。
                    if rng.random() < wall_probability:
                        walls.add(((x, y), other))
    return Maze("Random {}".format(seed), "Test maze", size, frozenset(walls))


class RobotTests(unittest.TestCase):
    """继承 unittest.TestCase，复用断言工具并组织 10 项核心行为测试。

    ``self.assertEqual(a, b)`` 要求两者相等，``assertTrue`` 要求为真，
    ``assertFalse`` 要求为假；失败时 unittest 会报告测试名和代码位置。
    两个不以 test_ 开头的方法供各项测试共用，避免重复安全检查。
    """

    def assert_local_knowledge(self, simulation, maze):
        """逐条核对已知地图，检查局部感知边界和共享边的一致性。

        simulation 是被检查的机器人状态；maze 仅供测试读取真值。
        机器人已知地图尺寸，所以外边界应始终是 WALL。内部边只要
        任一端的格子访问过，就应该已经感知；两端都没访问过则必须
        保持 UNKNOWN，不能提前知道远处有墙或无墙。该方法无返回值，
        检查不成立时由断言报告失败。
        """
        for y in range(maze.size):
            for x in range(maze.size):
                cell = (x, y)
                for dx, dy in DIRECTIONS:
                    other = (x + dx, y + dy)
                    observed = simulation.known.edge_state(cell, (dx, dy))
                    if not maze.contains(other):
                        # 情况一：邻居在地图外，当前位置这侧是边界墙。
                        self.assertEqual(observed, WALL)
                    elif cell in simulation.visited or other in simulation.visited:
                        # 情况二：至少一端访问过，观测必须与真实墙一致。
                        # assertEqual 的第三个参数是在失败时附带显示的
                        # 坐标，便于定位是哪一条边出了问题。
                        expected = WALL if tuple(sorted((cell, other))) in maze.walls else OPEN
                        self.assertEqual(observed, expected, (cell, other))
                    else:
                        # 情况三：两端均未访问，连“这里没墙”也不能偷看。
                        self.assertEqual(observed, UNKNOWN, (cell, other))
                    if maze.contains(other):
                        # A 的东侧就是 B 的西侧：反向查询必须得到同一状态。
                        # 取方向的相反数，就能从邻居指回当前格。
                        reverse = simulation.known.edge_state(other, (-dx, -dy))
                        self.assertEqual(observed, reverse, (cell, other))

    def run_checked(self, maze):
        """运行一张地图，并在每一步检查移动和地图记录是否符合约定。

        输入 Maze，返回已经停止的 Simulation，供具体测试继续检查
        到达状态、步数等结果。循环次数设有宽松上限，避免代码出现
        循环错误时测试永远挂起；这个上限是测试保护，不是对算法
        最坏步数的数学证明。超过上限还未停止，会明确判为测试失败。
        """
        simulation = Simulation(maze)
        # 初始化已经感知起点，尚未移动时也要检查信息是否正确。
        self.assert_local_knowledge(simulation, maze)
        for _ in range(4 * maze.size ** 4):
            if simulation.status != "exploring":
                break
            # 必须先保存“移动前”的位置、距离和边状态。step() 内部
            # 会移动、感知新位置并重算距离，直接查看移动后的距离无法
            # 验证“当时选择这一步是否合理”。dict(...) 保存字典副本。
            before = simulation.position
            distances_before = dict(simulation.distances)
            edges_before = {
                direction: simulation.known.edge_state(before, direction)
                for direction in DIRECTIONS
            }
            # 探索中一次 step() 应实际移动一格；返回 True 表示移动发生。
            self.assertTrue(simulation.step())
            after = simulation.position
            direction = (after[0] - before[0], after[1] - before[1])
            # 坐标差必须为四个单位方向之一，排除斜走、跳格和原地不动。
            self.assertIn(direction, DIRECTIONS)
            # 用真值检查没有越界、没有穿墙。
            self.assertTrue(maze.contains(after))
            self.assertNotIn(tuple(sorted((before, after))), maze.walls)
            # 即使真实没有墙，也不能跨越尚未感知的 UNKNOWN 边。
            self.assertEqual(edges_before[direction], OPEN)
            # 本次决策必须沿“旧距离减 1”的方向走；新距离可以因发现墙
            # 而变大，所以不能把这里的 distances_before 换成新距离。
            self.assertEqual(distances_before[after], distances_before[before] - 1)
            # 路径包含起点，所以步数少 1；visited 去重，path 保留回头路。
            self.assertEqual(simulation.steps, len(simulation.path) - 1)
            self.assertEqual(simulation.visited, set(simulation.path))
            self.assert_local_knowledge(simulation, maze)
        # 不允许因为循环上限到了，就把仍在探索的机器人误当作成功结束。
        self.assertNotEqual(simulation.status, "exploring", "Robot failed to stop")
        # 到达或不可达都属于终止状态：再次 step() 应返回 False，且位置、
        # 步数、路径保持不变。list(...) 保存路径副本，避免比较同一列表。
        saved = (simulation.position, simulation.steps, list(simulation.path))
        self.assertFalse(simulation.step())
        self.assertEqual(saved, (simulation.position, simulation.steps, simulation.path))
        return simulation

    def test_open_preset_reaches_goal_in_fourteen_moves(self):
        """测试 1：8 × 8 空地图应到达右下角，恰好走 14 步。"""
        simulation = self.run_checked(PRESETS["open"])
        self.assertEqual(simulation.status, "reached")
        self.assertEqual(simulation.position, PRESETS["open"].goal)
        # 从 (0, 0) 到 (7, 7)，横向 7 步加纵向 7 步，共 14 步。
        self.assertEqual(simulation.steps, 14)

    def test_detour_preset_reaches_goal_and_revisits_cells(self):
        """测试 2：默认方向顺序下，死路地图应回头绕行并在第 26 步到达。"""
        simulation = self.run_checked(PRESETS["detour"])
        self.assertEqual(simulation.status, "reached")
        self.assertEqual(simulation.steps, 26)
        # 集合只记不同格子，路径记录每次经过；前者更短说明确实有重访。
        # 26 是这张地图和当前平局选向规则的结果，不是所有策略的最短路。
        self.assertLess(len(simulation.visited), len(simulation.path))

    def test_blocked_preset_stops_as_unreachable(self):
        """测试 3：终点被墙包围时，应报告不可达，而不是假装到达。"""
        simulation = self.run_checked(PRESETS["blocked"])
        self.assertEqual(simulation.status, "unreachable")
        # 当前格到终点的距离为 None，且实际从未访问终点。
        self.assertIsNone(simulation.distances[simulation.position])
        self.assertNotIn(PRESETS["blocked"].goal, simulation.visited)

    def test_hidden_wall_stays_unknown_until_an_endpoint_is_visited(self):
        """测试 4：远处真实存在的墙，在机器人走到任一端之前必须保密。"""
        a, b = (3, 2), (3, 3)
        maze = Maze("Hidden wall", "", 4, frozenset({(a, b)}))
        simulation = Simulation(maze)
        # a 向南连接 b；初始化时机器人还在 (0, 0)，不能看见这条边。
        self.assertEqual(simulation.known.edge_state(a, (0, 1)), UNKNOWN)
        # 循环上限同样只是防止测试挂起；每步都检查墙没有提前泄露。
        for _ in range(4 * maze.size ** 4):
            if a in simulation.visited or b in simulation.visited:
                break
            self.assertEqual(simulation.known.edge_state(a, (0, 1)), UNKNOWN)
            self.assertTrue(simulation.step())
        # 抵达墙任一端后，两端反向查询都必须知道这里有墙。
        self.assertTrue(a in simulation.visited or b in simulation.visited)
        self.assertEqual(simulation.known.edge_state(a, (0, 1)), WALL)
        self.assertEqual(simulation.known.edge_state(b, (0, -1)), WALL)

    def test_walls_and_observations_are_symmetric(self):
        """测试 5：共享边从两端看应一致，静态地图的矛盾观测应报错。"""
        # 故意反着写墙的两个端点，顺便检查 Maze 是否会规范化边的顺序。
        maze = Maze("Symmetry", "", 3, frozenset({((1, 0), (0, 0))}))
        self.assertEqual(maze.sense((0, 0))[(1, 0)], WALL)
        self.assertEqual(maze.sense((1, 0))[(-1, 0)], WALL)
        known = KnownMaze(maze.size, maze.start, maze.goal)
        # 只在 (0, 0) 记录一次，也应能从邻格反向查到墙和开放边。
        known.record((0, 0), maze.sense((0, 0)))
        self.assertEqual(known.edge_state((1, 0), (-1, 0)), WALL)
        self.assertEqual(known.edge_state((0, 1), (0, -1)), OPEN)
        # assertRaises 表示“这段代码必须抛出指定异常才算通过”。
        # 已知是 WALL 的静态边不能随后被记录为 OPEN。
        with self.assertRaises(ValueError):
            known.record((1, 0), {(-1, 0): OPEN})

    def test_start_at_goal_finishes_without_moving(self):
        """测试 6：起点等于终点时，1 × 1 和 3 × 3 地图都应 0 步结束。"""
        for size, cell in ((1, (0, 0)), (3, (1, 1))):
            # subTest 将循环中的案例分别标记：失败报告会显示 size，
            # 一个案例的断言失败也不会阻止其余案例继续检查。
            with self.subTest(size=size):
                maze = Maze("Already there", "", size, start=cell, goal=cell)
                simulation = self.run_checked(maze)
                self.assertEqual(simulation.status, "reached")
                self.assertEqual(simulation.steps, 0)
                # 即使没移动，路径仍含起点；起点到自身的距离为 0。
                self.assertEqual(simulation.path, [cell])
                self.assertEqual(simulation.distances[cell], 0)

    def test_enclosed_start_stops_without_moving(self):
        """测试 7：起点的两条内部出口都堵住时，初始化就应判定不可达。"""
        # 左上角的北、西侧本来就是边界墙，只需再堵住东、南两侧。
        walls = frozenset({edge_key((0, 0), (1, 0)), edge_key((0, 0), (0, 1))})
        simulation = self.run_checked(Maze("Enclosed start", "", 3, walls))
        self.assertEqual(simulation.status, "unreachable")
        self.assertEqual(simulation.steps, 0)

    def test_out_of_map_goal_is_rejected(self):
        """测试 8：地图构造和洪水填充都应拒绝四个方向的越界终点。"""
        # 3 × 3 地图的合法坐标分量只有 0、1、2；-1 和 3 都越界。
        for goal in ((-1, 0), (0, -1), (3, 0), (0, 3)):
            with self.subTest(goal=goal):
                with self.assertRaises(ValueError):
                    Maze("Invalid goal", "", 3, goal=goal)
                # 用合法 KnownMaze 搭配非法 goal，单独检查规划器的验证。
                known = KnownMaze(3, (0, 0), (2, 2))
                with self.assertRaises(ValueError):
                    flood_fill(known, goal)

    def test_fully_known_distances_match_independent_dijkstra(self):
        """测试 9：用 24 个固定随机种子子案例，逐格比较 BFS 与 Dijkstra 距离。"""
        for seed in range(24):
            # % 是取余；尺寸和墙概率都由 seed % 5 选择，所以只会出现
            # 5 对固定参数，并未覆盖所有尺寸与概率的组合。
            # 概率为 0 的 5 个种子都生成同一张全开地图，概率为 1 的
            # 4 个种子也都生成同一张全封地图；24 个案例不代表 24 张不同地图。
            size = 2 + seed % 5
            probability = (0.0, 0.15, 0.35, 0.6, 1.0)[seed % 5]
            with self.subTest(seed=seed, size=size, walls=probability):
                maze = random_maze(seed, size, probability)
                known = KnownMaze(size, maze.start, maze.goal)
                # 本测试特意先感知所有格子，构造一张“完整已知地图”。
                # 只有掌握相同墙信息，才应与真实地图的距离逐格相等。
                # 这是测试准备过程，正常机器人仍然只感知自己到达的格子。
                for y in range(size):
                    for x in range(size):
                        known.record((x, y), maze.sense((x, y)))
                # 比较整个字典，既检查数字，也检查不可达格子的 None。
                self.assertEqual(flood_fill(known, maze.goal), actual_distances(maze))

    def test_exploration_reaches_exactly_the_reachable_random_goals(self):
        """测试 10：用 30 个固定随机种子子案例，核对探索结果与真实可达性。

        这里保留机器人的正常局部感知流程，仅把最终成功或不可达与
        独立参照进行比较，不要求第一次探索路径等于真实最短路径。
        """
        # 记录案例是否同时包含“能到”和“不能到”，防止只测到一种情况。
        outcomes = set()
        for seed in range(30):
            # 尺寸按 seed % 5 选择，概率按 seed % 6 选择；0～29 这
            # 30 个种子恰好覆盖 5 × 6 种参数组合，每种组合一个案例。
            # 这仍只是有限样例，不表示覆盖了每种参数下所有可能的地图。
            size = 2 + seed % 5
            probability = (0.05, 0.2, 0.35, 0.5, 0.75, 1.0)[seed % 6]
            with self.subTest(seed=seed, size=size, walls=probability):
                maze = random_maze(seed, size, probability)
                # 参照算法从终点反向计算；起点距离不是 None 就说明可达。
                reachable = actual_distances(maze)[maze.start] is not None
                # run_checked 在每次移动中同时核对安全和局部信息边界。
                simulation = self.run_checked(maze)
                self.assertEqual(simulation.status, "reached" if reachable else "unreachable")
                if reachable:
                    self.assertEqual(simulation.position, maze.goal)
                outcomes.add(reachable)
        self.assertEqual(outcomes, {False, True})


# 直接执行 python3 test_robot.py 时启动测试；被 unittest 导入收集时，
# __name__ 是模块名，下面不会再启动一轮测试，交由外部测试运行器负责。
if __name__ == "__main__":
    unittest.main()
