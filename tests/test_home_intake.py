from unittest.mock import MagicMock

from commands.home_intake import HomeIntake


def test_home_intake_is_noop():
    """HomeIntake is now a no-op — finishes immediately."""
    intake = MagicMock()
    c = HomeIntake.__new__(HomeIntake)
    c.intake = intake
    assert c.isFinished()
