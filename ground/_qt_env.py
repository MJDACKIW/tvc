"""Fail clearly, instead of with Qt's "Could not find the Qt platform plugin", when PyQt6's
plugins carry the macOS hidden flag.

This repo sits in an iCloud-synced "Documents - <Mac name>" folder, where everything inside a
dot-directory (.venv, .git, ...) ends up with UF_HIDDEN set (`ls -lO` shows "hidden"). Qt's
plugin scan skips hidden files, so a venv named .venv here finds no platform plugin at all
(cocoa or offscreen). A venv whose path has no dot-directory in it, such as ground/venv, is
unaffected.
"""
import stat
import sys
from pathlib import Path


def check_plugins_not_hidden():
    try:
        import PyQt6
    except ImportError:
        return
    platforms = Path(PyQt6.__file__).resolve().parent / "Qt6" / "plugins" / "platforms"
    try:
        hidden = any(p.stat().st_flags & stat.UF_HIDDEN for p in platforms.iterdir())
    except (OSError, AttributeError):  # AttributeError: no st_flags off macOS/BSD
        return
    if hidden:
        sys.exit(f"gs.py: PyQt6's Qt plugins are flagged hidden ({platforms}), so Qt will not "
                 "load them. This happens to venvs inside a dot-directory in this iCloud-synced "
                 "folder. Recreate the venv as ground/venv (see ground/README.md).")
