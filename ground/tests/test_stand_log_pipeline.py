"""synth_sd_log -> decode_log -> iae_compare, end to end (SPEC.md Section 7).

tools/decode_log.py writes the .npz schema sim/vehicle.py's load_stand_log reads, so a decoded
stand log feeds tools/iae_compare.py with no glue. The synthetic log is made up (its encoder
traces a lagged copy of the gimbal command, and its numbers mean nothing physically); this
proves the plumbing, not the physics: the schema, the params-hash guard, the pyro/burn-start
keys, and that the measured angle is the encoder and never the Kalman estimate.

    ground/venv/bin/python -m pytest ground/tests/test_stand_log_pipeline.py
"""
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLS = REPO_ROOT / "tools"


def run(*args, check=True):
    return subprocess.run([sys.executable, *map(str, args)], capture_output=True, text=True,
                          cwd=REPO_ROOT, check=check)


@pytest.fixture(scope="module")
def synth_bin(tmp_path_factory):
    out = tmp_path_factory.mktemp("synth") / "SF_SYNTH.BIN"
    run(TOOLS / "synth_sd_log.py", "--out", out)
    return out


@pytest.fixture(scope="module")
def decoded(synth_bin, tmp_path_factory):
    d = tmp_path_factory.mktemp("decoded")
    for axis in ("pitch", "yaw"):
        run(TOOLS / "decode_log.py", synth_bin, "--no-plot", "--axis", axis,
            "--npz", d / f"{axis}.npz")
    return d


def test_npz_has_the_load_stand_log_schema(decoded):
    sys.path.insert(0, str(REPO_ROOT / "sim"))
    from vehicle import load_stand_log

    log = load_stand_log(decoded / "pitch.npz")  # raises KeyError on a missing key
    assert len(log["time_s"]) == len(log["thrust_n"]) == len(log["encoder_angle_deg"])
    assert len(log["params_hash"]) == 64
    assert log["thrust_n"].max() > 25.0


def test_measured_theta_is_the_encoder_never_the_kalman_estimate(decoded):
    for axis, enc, xhat in (("pitch", "enc_p", "xhat_p"), ("yaw", "enc_y", "xhat_y")):
        d = np.load(decoded / f"{axis}.npz")
        assert str(d["axis"]) == axis
        assert np.array_equal(d["encoder_angle_deg"], d[enc])
        assert not np.array_equal(d["encoder_angle_deg"], d[xhat])


def test_pyro_on_time_is_set_and_burn_start_is_reserved_nan(decoded):
    d = np.load(decoded / "pitch.npz")
    t, pyro = d["time_s"], d["pyro"]
    assert float(d["pyro_on_time_s"]) == t[np.nonzero(pyro)[0][0]]
    assert np.isnan(float(d["burn_start_time_s"]))  # phototransistor, not fitted yet


def test_iae_compare_runs_on_the_decoded_log(decoded, tmp_path):
    result = run(TOOLS / "iae_compare.py", decoded / "pitch.npz", "--theta0-from-log",
                 "--out-dir", tmp_path)
    assert "IAE, measured" in result.stdout and "IAE, simulated" in result.stdout
    assert (tmp_path / "iae_compare_pitch.png").exists()


def test_iae_compare_refuses_a_log_from_different_params(decoded, tmp_path):
    d = dict(np.load(decoded / "pitch.npz"))
    d["params_hash"] = np.str_("0" * 64)
    stale = tmp_path / "stale.npz"
    np.savez(stale, **d)
    result = run(TOOLS / "iae_compare.py", stale, "--out-dir", tmp_path, check=False)
    assert result.returncode != 0
    assert "refusing to run" in result.stderr
