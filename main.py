"""项目启动入口：在此文件所在目录执行 python3 main.py（README 第 12.1 节）。

本文件负责启动图形界面和报告相关错误；迷宫数据、寻路计算与机器人移动
分别在 maze.py、planner.py 和 simulator.py 中，不在此处实现。
"""

import sys


def main():
    """尝试启动界面：正常关闭返回 0，已识别的 Tk 环境错误返回 1。"""
    # 把 import ui 放在 try 内，因为 ui 导入 tkinter 时就可能失败。
    # 这样能给缺少 Tk 的初学者一个可读提示，而不只是 Python 的异常堆栈。
    try:
        import ui
    except ModuleNotFoundError as error:
        # 只处理 tkinter 及其底层扩展 _tkinter 缺失；其他模块缺失仍抛出原异常，
        # 避免把真正的代码/文件问题误报为“没有安装 Tkinter”。
        if error.name not in ("tkinter", "_tkinter"):
            raise
        # stderr 是标准错误输出，与普通结果使用的标准输出区分，便于诊断。
        print("Tkinter is missing from this Python installation.\n"
              "Install a Python build with Tk support, then run python3 main.py again.",
              file=sys.stderr)
        return 1
    # 成功导入还不保证窗口可用：例如没有图形桌面时，创建 Tk 窗口可能失败。
    try:
        ui.launch()
    except ui.tk.TclError as error:
        # Tcl/Tk 是 Tkinter 使用的底层界面系统；保留具体错误文字以帮助排查。
        print("The Tkinter window could not start: {}\n"
              "Run this program in a desktop session with working Tk support.".format(error),
              file=sys.stderr)
        return 1
    # ui.launch() 的事件循环在窗口关闭后返回，这时程序正常结束。
    return 0


if __name__ == "__main__":
    # 直接运行本文件时 __name__ 是 "__main__"，才会进入此分支。
    # 其他文件执行 import main 时不会自动弹出窗口。
    # SystemExit 把 main() 的返回值交给操作系统：0 表示成功，1 表示失败。
    # 这里主动退出是正常的入口写法，并不等于程序运行到此处就发生崩溃。
    raise SystemExit(main())
