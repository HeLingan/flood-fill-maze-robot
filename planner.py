"""第 2 层：在机器人已知地图上进行反向 BFS，生成洪水填充距离场。

配合 README 第 6、10 节和第 13 节实验 A 阅读。
本模块只负责计算距离，不移动机器人，不绘制窗口，也不读取隐藏真实墙。
每次调用都重新计算全图；对于 8×8 的小迷宫，这种写法直观且足够使用。
"""

from collections import deque  # 双端队列；这里用 append + popleft 实现先进先出。
from typing import Dict, Optional

from maze import Cell, DIRECTIONS, KnownMaze, WALL, neighbor


def flood_fill(known: KnownMaze, goal: Cell) -> Dict[Cell, Optional[int]]:
    """输入知识地图和终点，返回 {格子坐标: 到终点的最少剩余步数}。

    每步代价都是 1，从终点按层向外扩散，首次标记的距离就是最短距离。
    这里的“最短”针对当前乐观地图：已知墙不可越过，未知边暂按通路处理。
    实际移动时，Simulation.step() 会另外要求跨越的边已经感知为 OPEN。

    返回值中，终点为 0，仍不可达的格子为 None；不会改变输入知识地图。
    若终点不在地图内则抛出 ValueError。边长 n 的地图，一次计算的时间
    和额外存储量都为 O(n²) 量级；这不是整个探索过程的总成本。
    """
    # 先拒绝非法终点，避免后面给地图外的坐标建立一个“起始距离”。
    if not known.contains(goal):
        raise ValueError("Goal is outside the map")

    # 第 1 步：为每个格子建立记录。两层 for 枚举所有 y、x，初值都是 None。
    # 在计算过程中，None 表示本轮还没扩散到；只有计算完才据此判断不可达。
    distances = {
        (x, y): None for y in range(known.size) for x in range(known.size)
    }
    # 第 2 步：到终点自身还需 0 步，并把终点放入待处理队列。
    distances[goal] = 0
    queue = deque([goal])

    # 第 3 步：只要还有待处理格子，就从队头取出一个，检查四个邻居。
    # 先进先出的顺序保证先处理距离 0，再处理距离 1，再处理距离 2……
    while queue:
        cell = queue.popleft()
        for direction in DIRECTIONS:
            other = neighbor(cell, direction)
            # 只拦住 WALL，因此 UNKNOWN 和 OPEN 都允许“规划扩散”。
            # 外边界也返回 WALL，所以越界邻居会在访问 distances[other] 前被跳过。
            if known.edge_state(cell, direction) == WALL:
                continue
            # 第 4 步：只给第一次发现的格子赋值，防止重复入队和来回扩散。
            if distances[other] is None:
                # 穿过一条边只花 1 步，邻居距离就是当前距离加 1。
                distances[other] = distances[cell] + 1
                # 新发现的格子排在队尾，等更近的这一层处理完后再轮到它。
                queue.append(other)

    # 队列清空：全部能连到终点的格子都已标记；其余 None 表示当前图上不可达。
    # 这是距离表，不是一条坐标路径；选择下一步是 simulator.py 的职责。
    return distances
