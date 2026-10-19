"""Launch the flood-fill maze demonstration: python3 main.py."""

import sys


def main():
    try:
        import ui
    except ModuleNotFoundError as error:
        if error.name not in ("tkinter", "_tkinter"):
            raise
        print("Tkinter is missing from this Python installation.\n"
              "Install a Python build with Tk support, then run python3 main.py again.",
              file=sys.stderr)
        return 1
    try:
        ui.launch()
    except ui.tk.TclError as error:
        print("The Tkinter window could not start: {}\n"
              "Run this program in a desktop session with working Tk support.".format(error),
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
