import commands2

from subsystems.intake import IntakeSubSystem


class HomeIntake(commands2.Command):
    """
    No-op — intake no longer requires homing.
    Kept for backwards compatibility; finishes immediately.
    """

    def __init__(self, intake: IntakeSubSystem) -> None:
        super().__init__()
        self.intake = intake
        self.addRequirements(self.intake)

    def isFinished(self) -> bool:
        return True
