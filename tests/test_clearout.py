from unittest.mock import MagicMock

import pytest

from commands.clearout import CLEAROUT_S, Clearout


@pytest.fixture
def shooter():
    return MagicMock()


@pytest.fixture
def kicker():
    return MagicMock()


@pytest.fixture
def indexer():
    return MagicMock()


def _make_cmd(shooter, kicker, indexer, rpm=2000.0):
    c = Clearout.__new__(Clearout)
    c.shooter = shooter
    c.kicker = kicker
    c.indexer = indexer
    c._rpm = rpm
    c._timer = MagicMock()
    c._timer.hasElapsed.return_value = False
    return c


def test_execute_reverses_conveyor(shooter, kicker, indexer):
    """Clearout should spin shooter+kicker forward and conveyor backward."""
    cmd = _make_cmd(shooter, kicker, indexer, rpm=2500.0)
    cmd.initialize()
    cmd.execute()

    shooter.set_target_speed.assert_called_with(2500.0)
    kicker.set_duty_cycle.assert_called_with(1.0)
    indexer.set_target_output.assert_called_with(-1.0)


def test_finishes_after_timeout(shooter, kicker, indexer):
    """Clearout should finish once the timer elapses."""
    cmd = _make_cmd(shooter, kicker, indexer)
    cmd.initialize()

    assert not cmd.isFinished()

    cmd._timer.hasElapsed.return_value = True
    assert cmd.isFinished()
    cmd._timer.hasElapsed.assert_called_with(CLEAROUT_S)


def test_end_stops_all_motors(shooter, kicker, indexer):
    """On end, all motors should be stopped."""
    cmd = _make_cmd(shooter, kicker, indexer)
    cmd.initialize()
    cmd.end(interrupted=False)

    shooter.stop.assert_called_once()
    kicker.stop.assert_called_once()
    indexer.stop.assert_called_once()


def test_end_stops_on_interrupt(shooter, kicker, indexer):
    """Motors should stop even when interrupted."""
    cmd = _make_cmd(shooter, kicker, indexer)
    cmd.initialize()
    cmd.end(interrupted=True)

    shooter.stop.assert_called_once()
    kicker.stop.assert_called_once()
    indexer.stop.assert_called_once()
