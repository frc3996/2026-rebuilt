from unittest.mock import MagicMock

import pytest
from wpilib import Timer

from commands.home_intake import HomeIntake
from subsystems.intake import (
    STALL_CONFIRM_CYCLES,
    STALL_CURRENT_THRESHOLD,
    STALL_VELOCITY_THRESHOLD,
)


@pytest.fixture
def intake():
    i = MagicMock()
    i.homed = False
    i.up_down_motor = MagicMock()
    i.up_down_encoder = MagicMock()
    return i


@pytest.fixture
def cmd(intake):
    c = HomeIntake.__new__(HomeIntake)
    c.intake = intake
    c._stall_counter = 0
    c._timer = Timer()
    c._timed_out = False
    return c


def _simulate_stall(intake):
    intake.up_down_motor.getOutputCurrent.return_value = STALL_CURRENT_THRESHOLD + 1.0
    intake.up_down_encoder.getVelocity.return_value = STALL_VELOCITY_THRESHOLD / 2.0


def _simulate_no_stall(intake):
    intake.up_down_motor.getOutputCurrent.return_value = 1.0
    intake.up_down_encoder.getVelocity.return_value = 100.0


def test_stall_confirmed_after_n_cycles(cmd, intake):
    """Motor stalls for STALL_CONFIRM_CYCLES → encoder zeroed, homed."""
    cmd.initialize()
    _simulate_stall(intake)

    for _ in range(STALL_CONFIRM_CYCLES - 1):
        cmd.execute()
        assert not cmd.isFinished()

    cmd.execute()
    assert cmd.isFinished()

    cmd.end(interrupted=False)
    intake.up_down_encoder.setPosition.assert_called_once_with(0.0)
    assert intake.homed
    intake.enable_soft_limits.assert_called_once()


def test_counter_resets_on_condition_drop(cmd, intake):
    """If stall conditions drop mid-sequence, counter resets to 0."""
    cmd.initialize()
    _simulate_stall(intake)

    for _ in range(STALL_CONFIRM_CYCLES - 2):
        cmd.execute()

    _simulate_no_stall(intake)
    cmd.execute()
    assert not cmd.isFinished()

    _simulate_stall(intake)
    for _ in range(STALL_CONFIRM_CYCLES - 1):
        cmd.execute()
        assert not cmd.isFinished()

    cmd.execute()
    assert cmd.isFinished()


def test_timeout_does_not_home(cmd, intake):
    """Timeout → motor stopped, encoder NOT zeroed, homed stays False."""
    cmd.initialize()
    _simulate_no_stall(intake)

    cmd._timer = MagicMock()
    cmd._timer.hasElapsed.return_value = True
    cmd.execute()
    assert cmd.isFinished()

    cmd.end(interrupted=False)
    intake.up_down_encoder.setPosition.assert_not_called()
    assert not intake.homed
    intake.enable_soft_limits.assert_not_called()


def test_interrupted_does_not_home(cmd, intake):
    """Interrupted → motor stopped, encoder NOT zeroed."""
    cmd.initialize()
    _simulate_stall(intake)

    for _ in range(STALL_CONFIRM_CYCLES):
        cmd.execute()

    cmd.end(interrupted=True)
    intake.up_down_encoder.setPosition.assert_not_called()
    assert not intake.homed
    intake.enable_soft_limits.assert_not_called()
