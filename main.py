"""Launch the Tkinter application and report unavailable Tk support."""

import sys


def main():
    """Run the GUI; return 0 on normal closure or 1 for handled Tk errors.

    Takes no arguments. Missing Tk imports and TclError from ui.launch() are
    reported; ModuleNotFoundError for unrelated imports propagates.
    """
    try:
        import ui
    except ModuleNotFoundError as error:
        # Missing application modules must not be reported as missing Tk support.
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
