from unittest.mock import MagicMock

import pytest
from wpilib import Timer

from commands.home_hood import HomeHood
from subsystems.hood import (
    STALL_CONFIRM_CYCLES,
    STALL_CURRENT_THRESHOLD,
    STALL_VELOCITY_THRESHOLD,
)


@pytest.fixture
def hood():
    h = MagicMock()
    h.is_homed = False
    return h


@pytest.fixture
def cmd(hood):
    c = HomeHood.__new__(HomeHood)
    c.hood = hood
    c._stall_counter = 0
    c._timer = Timer()
    c._timed_out = False
    return c


def _simulate_stall(hood):
    hood.get_output_current.return_value = STALL_CURRENT_THRESHOLD + 1.0
    hood.get_velocity.return_value = STALL_VELOCITY_THRESHOLD / 2.0


def _simulate_no_stall(hood):
    hood.get_output_current.return_value = 1.0
    hood.get_velocity.return_value = 100.0


def test_stall_confirmed_after_n_cycles(cmd, hood):
    """Motor stalls for STALL_CONFIRM_CYCLES → encoder zeroed, homed."""
    cmd.initialize()
    _simulate_stall(hood)

    for _ in range(STALL_CONFIRM_CYCLES - 1):
        cmd.execute()
        assert not cmd.isFinished()

    cmd.execute()
    assert cmd.isFinished()

    cmd.end(interrupted=False)
    hood.reset_encoder.assert_called_once()
    hood.enable_soft_limits.assert_called_once()


def test_counter_resets_on_condition_drop(cmd, hood):
    """If stall conditions drop mid-sequence, counter resets to 0."""
    cmd.initialize()
    _simulate_stall(hood)

    for _ in range(STALL_CONFIRM_CYCLES - 2):
        cmd.execute()

    # Break the stall — velocity goes high
    _simulate_no_stall(hood)
    cmd.execute()
    assert not cmd.isFinished()

    # Restart stall — need full STALL_CONFIRM_CYCLES again
    _simulate_stall(hood)
    for _ in range(STALL_CONFIRM_CYCLES - 1):
        cmd.execute()
        assert not cmd.isFinished()

    cmd.execute()
    assert cmd.isFinished()


def test_timeout_does_not_home(cmd, hood):
    """Timeout → motor stopped, encoder NOT zeroed, is_homed stays False."""
    cmd.initialize()
    _simulate_no_stall(hood)

    # Simulate timer elapsed
    cmd._timer = MagicMock()
    cmd._timer.hasElapsed.return_value = True
    cmd.execute()
    assert cmd.isFinished()

    cmd.end(interrupted=False)
    hood.reset_encoder.assert_not_called()
    assert not hood.is_homed
    hood.enable_soft_limits.assert_not_called()


def test_interrupted_does_not_home(cmd, hood):
    """Interrupted → motor stopped, encoder NOT zeroed."""
    cmd.initialize()
    _simulate_stall(hood)

    for _ in range(STALL_CONFIRM_CYCLES):
        cmd.execute()

    cmd.end(interrupted=True)
    hood.reset_encoder.assert_not_called()
    assert not hood.is_homed
    hood.enable_soft_limits.assert_not_called()
