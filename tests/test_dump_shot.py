from unittest.mock import MagicMock

import pytest

from commands.dump_shot import DUMP_RPM, FEED_DELAY_S, DumpShot


@pytest.fixture
def shooter():
    return MagicMock()


@pytest.fixture
def kicker():
    return MagicMock()


@pytest.fixture
def indexer():
    return MagicMock()


@pytest.fixture
def hood():
    h = MagicMock()
    h.max_rotations = 1.5
    return h


def _make_cmd(shooter, kicker, indexer, hood):
    c = DumpShot.__new__(DumpShot)
    c.shooter = shooter
    c.kicker = kicker
    c.indexer = indexer
    c.hood = hood
    c._feed_timer = MagicMock()
    c._feed_timer.hasElapsed.return_value = False
    return c


def test_execute_sets_hood_max_and_rpm(shooter, kicker, indexer, hood):
    """DumpShot should set hood to max and shooter to DUMP_RPM."""
    cmd = _make_cmd(shooter, kicker, indexer, hood)
    cmd.initialize()
    cmd.execute()

    hood.set_target_position.assert_called_with(hood.max_rotations)
    shooter.set_target_speed.assert_called_with(DUMP_RPM)
    kicker.set_duty_cycle.assert_called_with(1.0)


def test_indexer_stopped_before_feed_delay(shooter, kicker, indexer, hood):
    """Indexer should not feed before FEED_DELAY_S."""
    cmd = _make_cmd(shooter, kicker, indexer, hood)
    cmd._feed_timer.hasElapsed.return_value = False
    cmd.initialize()
    cmd.execute()

    indexer.stop.assert_called_once()
    indexer.set_target_output.assert_not_called()


def test_indexer_feeds_after_delay(shooter, kicker, indexer, hood):
    """Indexer should feed forward after FEED_DELAY_S."""
    cmd = _make_cmd(shooter, kicker, indexer, hood)
    cmd._feed_timer.hasElapsed.return_value = True
    cmd.initialize()
    cmd.execute()

    indexer.set_target_output.assert_called_with(1.0)
    cmd._feed_timer.hasElapsed.assert_called_with(FEED_DELAY_S)


def test_never_finishes(shooter, kicker, indexer, hood):
    """DumpShot runs until interrupted."""
    cmd = _make_cmd(shooter, kicker, indexer, hood)
    assert not cmd.isFinished()


def test_end_stows_hood(shooter, kicker, indexer, hood):
    """End should stow the hood."""
    cmd = _make_cmd(shooter, kicker, indexer, hood)
    cmd.end(interrupted=False)

    hood.stow.assert_called_once()


def test_end_stows_hood_on_interrupt(shooter, kicker, indexer, hood):
    """Hood should be stowed even when interrupted."""
    cmd = _make_cmd(shooter, kicker, indexer, hood)
    cmd.end(interrupted=True)

    hood.stow.assert_called_once()
