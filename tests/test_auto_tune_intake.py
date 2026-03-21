import math
from unittest.mock import MagicMock

import pytest
from wpilib import Timer

from commands.auto_tune_intake import RELAY_OUTPUT, REQUIRED_CROSSINGS, AutoTuneIntakeCommand


@pytest.fixture
def intake():
    i = MagicMock()
    i.homed = True
    i.limits_set = True
    i.min_rotations = -4.0
    i.max_rotations = 0.0
    return i


@pytest.fixture
def cmd(intake):
    c = AutoTuneIntakeCommand.__new__(AutoTuneIntakeCommand)
    c.intake = intake
    c._midpoint = 0.0
    c._crossings = []
    c._above = False
    c._timer = Timer()
    c._timed_out = False
    return c


def test_gain_calculation(cmd, intake):
    """Given known crossing timestamps, verify Z-N PD gains are correct."""
    intake.get_arm_position.return_value = -3.0  # below midpoint
    cmd.initialize()

    cmd._crossings = [0.5 + i * 0.25 for i in range(REQUIRED_CROSSINGS)]
    cmd._timed_out = False

    cmd.end(interrupted=False)

    tu = 0.5
    ku = (4.0 * RELAY_OUTPUT) / math.pi
    expected_p = 0.6 * ku
    expected_d = 0.075 * ku * tu

    intake.set_arm_pid_gains.assert_called_once()
    actual_p = intake.set_arm_pid_gains.call_args[0][0]
    actual_d = intake.set_arm_pid_gains.call_args[0][2]
    assert abs(actual_p - expected_p) < 1e-6
    assert abs(actual_d - expected_d) < 1e-6


def test_timeout_no_gains(cmd, intake):
    """Timeout → gains NOT applied."""
    intake.get_arm_position.return_value = -3.0
    cmd.initialize()
    cmd._timed_out = True

    cmd.end(interrupted=False)
    intake.set_arm_pid_gains.assert_not_called()


def test_interrupted_no_gains(cmd, intake):
    """Interrupted → gains NOT applied."""
    intake.get_arm_position.return_value = -3.0
    cmd.initialize()
    cmd._crossings = [0.5 + i * 0.25 for i in range(REQUIRED_CROSSINGS)]

    cmd.end(interrupted=True)
    intake.set_arm_pid_gains.assert_not_called()
