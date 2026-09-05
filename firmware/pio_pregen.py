"""PlatformIO pre-build step: regenerate core/params.h (from params.yaml) and the parity
fixture header (from tools/parity_test.py) before every build/test, so neither can go stale
and neither ever shows up as an uncommitted diff. See SPEC.md Section 2 and Section 6.1.

SCons exec's extra_scripts as source text rather than importing them as a module, so
`__file__` is not defined here; "#" is SCons's own name for the project root (where
platformio.ini lives), one level below the repo root.
"""
Import("env")

import subprocess
from pathlib import Path

repo_root = Path(env.Dir("#").abspath).parent
subprocess.run(["python3", str(repo_root / "tools" / "gen_params.py")], check=True, cwd=repo_root)
subprocess.run(
    ["python3", str(repo_root / "tools" / "parity_test.py"), "--fixture-only"],
    check=True,
    cwd=repo_root,
)
