import commands2
from commands.home_hood import HomeHood
from commands.home_intake import HomeIntake
from subsystems.hood import HoodSubSystem
from subsystems.intake import IntakeSubSystem


class AutoHome(commands2.SequentialCommandGroup):
    """
    Homes the hood and intake arm in sequence on startup.
    After this command completes, both subsystems accept position commands.
    """

    def __init__(self, hood: HoodSubSystem, intake: IntakeSubSystem):
        super().__init__(
            HomeHood(hood),
            HomeIntake(intake),
        )
