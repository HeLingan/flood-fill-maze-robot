"""地图模块：定义真实迷宫，以及机器人逐步建立的已知地图。

配合 README 第 5、9 节阅读；建议先读本文件，再读 planner.py。
所有格子使用 (x, y) 坐标：左上角是 (0, 0)，向右 x 增加，向下 y 增加。
墙位于两个相邻格子的公共边上，而不是把某个格子整体标为障碍物。

Maze 保存模拟世界的完整墙布局，sense() 只提供当前位置四边的观测。
KnownMaze 保存机器人已获得的信息，规划和绘图都只使用这张已知地图。
"""

from dataclasses import dataclass
from typing import Dict, FrozenSet, Mapping, Optional, Tuple

# 类型别名用于说明数据形状，不是额外创建三种带行为的对象。
# Cell 示例：(2, 3)；Direction 示例：(1, 0)；Edge 示例：((2, 3), (3, 3))。
Cell = Tuple[int, int]
Direction = Tuple[int, int]
Edge = Tuple[Cell, Cell]
# 东、南、西、北。这个顺序也是多条路线一样好时的选择顺序。
DIRECTIONS = ((1, 0), (0, 1), (-1, 0), (0, -1))
# 常量名称方便阅读；实际存储的值是右侧三个字符串。
# UNKNOWN 表示还不知道，绝不能把它理解成传感器已经确认的 OPEN。
UNKNOWN, OPEN, WALL = "unknown", "open", "wall"


def neighbor(cell: Cell, direction: Direction) -> Cell:
    """把坐标加上方向，返回相邻格；本函数不检查结果是否越界。

    例如 neighbor((2, 3), (1, 0)) 返回 (3, 3)，即向东一格。
    调用者用 contains() 判断是否越界，用 edge_state() 查询这条边
    是墙、通路还是未知；在地图内不代表两格之间一定能通行。
    """
    return cell[0] + direction[0], cell[1] + direction[1]


def edge_key(a: Cell, b: Cell) -> Edge:
    """把两个相邻格子转换成唯一的公共边标识；不相邻则报 ValueError。

    A→B 与 B→A 是同一条边。统一端点顺序后，墙集合和已知地图
    都只保存一份数据，从任一端查询都能得到一致的结果。
    """
    # 曼哈顿距离为 1：横、纵坐标差的绝对值之和必须为 1。
    # 这同时排除同一格、对角格和相隔多格的端点。
    if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:
        raise ValueError("An edge must connect two adjacent cells")
    # sorted() 返回排好序的列表；tuple() 再把它转成可作为字典键的元组。
    return tuple(sorted((a, b)))


@dataclass(frozen=True)
class Maze:
    """一张固定的真实地图：保存大小、起终点和完整的内部墙集合。

    dataclass 自动生成常用的初始化方法；frozen=True 限制普通字段赋值。
    墙使用 frozenset，不会在探索过程中改变。传感器可以查真实地图，
    但正常规划器只会收到 KnownMaze，不会收到这里的完整墙布局。
    """

    # 名称和说明帮助识别地图；当前界面下拉框显示的是 PRESETS 的键名。
    name: str
    description: str
    size: int = 8
    # 空集合表示没有内部墙；外边界由坐标判断自动处理。
    walls: FrozenSet[Edge] = frozenset()
    start: Cell = (0, 0)
    # 构造时可以省略 goal，__post_init__() 会把它补成右下角。
    goal: Optional[Cell] = None

    def __post_init__(self):
        """在数据类初始化后检查输入，补默认终点，并规范化所有墙。

        若尺寸、起终点或墙端点不合法，立即报错，避免错误流入寻路程序。
        本方法只整理这张地图的初始数据，不执行感知、移动或路径规划。
        """
        if self.size < 1:
            raise ValueError("Maze size must be positive")
        if self.goal is None:
            # 坐标从 0 开始，所以边长 8 的右下角是 (7, 7)。
            # 冻结的数据类不能普通赋值；这里只在初始化整理时显式设置字段。
            object.__setattr__(self, "goal", (self.size - 1, self.size - 1))
        if not self.contains(self.start) or not self.contains(self.goal):
            raise ValueError("Start and goal must be inside the maze")
        # 把可能顺序不一致的墙端点统一处理，并利用集合自动去重。
        normalized = set()
        for a, b in self.walls:
            if not self.contains(a) or not self.contains(b):
                raise ValueError("Internal walls must join cells inside the maze")
            # edge_key() 还会检查相邻性，因此地图内的对角端点也会被拒绝。
            normalized.add(edge_key(a, b))
        object.__setattr__(self, "walls", frozenset(normalized))

    def contains(self, cell: Cell) -> bool:
        """返回坐标是否在地图内；两个方向都必须位于 0 到 size-1 之间。"""
        return 0 <= cell[0] < self.size and 0 <= cell[1] < self.size

    def sense(self, cell: Cell) -> Dict[Direction, str]:
        """模拟理想传感器，返回指定格子四边的 {方向: OPEN 或 WALL}。

        输入必须是地图内的格子。结果仅包含这个格子的局部信息，
        不返回远处的墙；Simulation 正常运行时只传入机器人当前位置。
        本项目假设观测完全准确，没有测量噪声，也没有动态墙壁。
        """
        if not self.contains(cell):
            raise ValueError("Cannot sense outside the maze")
        # 字典推导式：对四个方向各生成一条观测。
        # 若相邻格越界，或公共边出现在真实墙集合里，就报告 WALL；否则 OPEN。
        # or 会短路：越界条件已为真时，不再执行后面的内部墙集合查询。
        return {
            direction: WALL if (
                not self.contains(neighbor(cell, direction))
                or edge_key(cell, neighbor(cell, direction)) in self.walls
            ) else OPEN
            for direction in DIRECTIONS
        }


class KnownMaze:
    """机器人的知识地图：内部边初始未知，外边界从一开始就已知为墙。

    只记录传感器已经确认的内部边；没有记录不代表通路，而是 UNKNOWN。
    这里没有保存真实地图的 walls 集合，因此规划器只能根据观测做决定。
    """

    def __init__(self, size: int, start: Cell, goal: Cell):
        """保存已知的尺寸和起终点，并创建一张尚未记录内部边的字典。"""
        self.size, self.start, self.goal = size, start, goal
        # 键是规范化的公共边，值是 OPEN 或 WALL。下划线表示内部使用约定。
        self._edges: Dict[Edge, str] = {}

    def contains(self, cell: Cell) -> bool:
        """返回坐标是否属于这张知识地图；规则与真实地图的范围相同。"""
        return 0 <= cell[0] < self.size and 0 <= cell[1] < self.size

    def edge_state(self, cell: Cell, direction: Direction) -> str:
        """查询某格某方向的边，返回 UNKNOWN、OPEN 或 WALL。

        格子或方向不合法时抛出 ValueError。边界墙由坐标直接推断，
        内部边则查观测字典；这个查询过程不会自行感知真实迷宫。
        """
        if not self.contains(cell) or direction not in DIRECTIONS:
            raise ValueError("Invalid cell or direction")
        other = neighbor(cell, direction)
        # 外边界不需要逐条存储，任何向地图外的边都直接返回 WALL。
        if not self.contains(other):
            return WALL
        # get() 查不到时使用默认值 UNKNOWN，保持“未知”和“确定可通行”的区别。
        return self._edges.get(edge_key(cell, other), UNKNOWN)

    def record(self, cell: Cell, observation: Mapping[Direction, str]) -> None:
        """把 {方向: 观测状态} 写入知识地图，没有返回值。

        正常运行由 Simulation 传入当前位置四边的感知结果。
        已确认的边不能在静态、无噪声模型中变成相反状态，矛盾时会报错。
        本方法只更新边信息；更新距离场由调用者另行执行 flood_fill()。
        """
        for direction, state in observation.items():
            # 先查询旧状态，这一步也会校验格子和方向是否合法。
            previous = self.edge_state(cell, direction)
            # 真实观测只能确认“通”或“堵”，不能把已经感知的边写回 UNKNOWN。
            if state not in (OPEN, WALL):
                raise ValueError("A sensor observation must be open or wall")
            if previous != UNKNOWN and previous != state:
                raise ValueError("Contradictory observation in a static maze")
            other = neighbor(cell, direction)
            # 只保存内部边。公共键让两端共享同一条观测，即使另一格还未访问。
            if self.contains(other):
                self._edges[edge_key(cell, other)] = state


# 预设地图对应 README 第 3、7 节；可在第 15 节练习中增加自己的预设。
# 下方五堵墙把右侧走廊与西边隔开：range(1, 6) 依次给出 y=1、2、3、4、5。
# 集合的 | 表示并集，再加入走廊底部 (7,5)-(7,6) 的墙，构成必须回头的死路。
_dead_end_walls = {
    edge_key((6, y), (7, y)) for y in range(1, 6)
} | {edge_key((7, 5), (7, 6))}

PRESETS = {
    # 省略内部墙集合：机器人按东优先的规则走 14 步到终点。
    "open": Maze("Open maze", "No internal walls: the robot reaches the goal in 14 moves."),
    # 在当前方向顺序下先进入右侧死路，再回访并绕行，共移动 26 步。
    "detour": Maze(
        "Dead-end detour",
        "A hidden dead end makes the robot turn back and find another route.",
        walls=frozenset(_dead_end_walls),
    ),
    # 终点右、下方已是外边界；再封住左、上两边，终点就完全不可达。
    "blocked": Maze(
        "Unreachable goal",
        "The goal is enclosed. The robot stops when its observations prove no route exists.",
        walls=frozenset({edge_key((6, 7), (7, 7)), edge_key((7, 6), (7, 7))}),
    ),
}
