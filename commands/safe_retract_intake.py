import commands2
from wpilib import Timer

from subsystems.intake import STOW_POSITION, IntakeSubSystem

RETRACT_TIMEOUT_SECONDS = 2.0


class SafeRetractIntake(commands2.Command):
    """
    Retracts the intake arm to stow position using PID position control.
    Stops when the arm reaches STOW_POSITION or times out.
    On end, holds at current position via PID.
    """

    def __init__(self, intake: IntakeSubSystem) -> None:
        super().__init__()
        self.intake = intake
        self.addRequirements(self.intake)
        self._timer = Timer()

    def initialize(self) -> None:
        self._timer.restart()
        self.intake.set_arm_target_position(STOW_POSITION)

    def end(self, interrupted: bool) -> None:
        # Hold at whatever position we ended up at
        self.intake.set_arm_target_position(self.intake.get_arm_position())

    def isFinished(self) -> bool:
        if self._timer.hasElapsed(RETRACT_TIMEOUT_SECONDS):
            return True
        return self.intake.is_stowed
