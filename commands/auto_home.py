import commands2

from commands.home_hood import HomeHood
from commands.home_intake import HomeIntake
from commands.safe_retract_intake import SafeRetractIntake
from subsystems.hood import HoodSubSystem
from subsystems.intake import IntakeSubSystem


class AutoHome(commands2.SequentialCommandGroup):
    """
    Homes the hood and intake arm in sequence on startup,
    then safely retracts the intake arm at gentle duty cycle.
    """

    def __init__(self, hood: HoodSubSystem, intake: IntakeSubSystem):
        super().__init__(
            HomeHood(hood),
            HomeIntake(intake),
            SafeRetractIntake(intake),
        )
