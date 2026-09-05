"""PlatformIO pre-build step: regenerate core/params.h from params.yaml before every
build/test, so it can never go stale. See SPEC.md Section 2.

SCons exec's extra_scripts as source text rather than importing them as a module, so
`__file__` is not defined here; "#" is SCons's own name for the project root (where
platformio.ini lives), one level below the repo root.
"""
Import("env")

import subprocess
from pathlib import Path

repo_root = Path(env.Dir("#").abspath).parent
subprocess.run(["python3", str(repo_root / "tools" / "gen_params.py")], check=True, cwd=repo_root)
