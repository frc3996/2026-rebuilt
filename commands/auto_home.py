import commands2

from commands.home_hood import HomeHood
from subsystems.hood import HoodSubSystem


class AutoHome(commands2.SequentialCommandGroup):
    """Homes the hood on startup."""

    def __init__(self, hood: HoodSubSystem):
        super().__init__(
            HomeHood(hood),
        )
