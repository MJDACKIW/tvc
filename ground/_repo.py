"""Makes protocol/, tools/, and sim/ importable from ground/ (the repo is not a package)."""
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
for _sub in ("protocol", "tools", "sim", "ground"):
    _p = str(REPO_ROOT / _sub)
    if _p not in sys.path:
        sys.path.insert(0, _p)
