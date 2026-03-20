import math
from unittest.mock import MagicMock

import pytest
from wpilib import Timer

from commands.auto_tune_hood import AutoTuneHoodCommand, RELAY_OUTPUT, REQUIRED_CROSSINGS


@pytest.fixture
def hood():
    h = MagicMock()
    h.is_homed = True
    h.limits_set = True
    h.min_rotations = 0.0
    h.max_rotations = 4.0
    return h


@pytest.fixture
def cmd(hood):
    c = AutoTuneHoodCommand.__new__(AutoTuneHoodCommand)
    c.hood = hood
    c._midpoint = 0.0
    c._crossings = []
    c._above = False
    c._timer = Timer()
    c._timed_out = False
    return c


def test_gain_calculation(cmd, hood):
    """Given known crossing timestamps, verify Z-N PD gains are correct."""
    hood.get_current_position.return_value = 1.0  # below midpoint (2.0)
    cmd.initialize()

    # Inject known crossings: half-period of 0.25s
    cmd._crossings = [0.5 + i * 0.25 for i in range(REQUIRED_CROSSINGS)]
    cmd._timed_out = False

    cmd.end(interrupted=False)

    tu = 0.5  # full period = 2 * 0.25s half-period
    ku = (4.0 * RELAY_OUTPUT) / math.pi
    expected_p = 0.6 * ku
    expected_d = 0.075 * ku * tu

    hood.set_pid_gains.assert_called_once()
    actual_p = hood.set_pid_gains.call_args[0][0]
    actual_d = hood.set_pid_gains.call_args[0][2]
    assert abs(actual_p - expected_p) < 1e-6
    assert abs(actual_d - expected_d) < 1e-6


def test_timeout_no_gains(cmd, hood):
    """Timeout → gains NOT applied."""
    hood.get_current_position.return_value = 1.0
    cmd.initialize()
    cmd._timed_out = True

    cmd.end(interrupted=False)
    hood.set_pid_gains.assert_not_called()


def test_interrupted_no_gains(cmd, hood):
    """Interrupted → gains NOT applied."""
    hood.get_current_position.return_value = 1.0
    cmd.initialize()
    cmd._crossings = [0.5 + i * 0.25 for i in range(REQUIRED_CROSSINGS)]

    cmd.end(interrupted=True)
    hood.set_pid_gains.assert_not_called()
