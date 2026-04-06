import commands2
from wpilib import Timer

from subsystems.intake import STOW_POSITION, IntakeSubSystem

RETRACT_DUTYCYCLE = -0.30  # Duty cycle toward stow (negative = retract)  # TUNE
RETRACT_TIMEOUT_SECONDS = 2.0


class SafeRetractIntake(commands2.Command):
    """
    Retracts the intake arm to stow using duty cycle.
    Stops when the arm reaches STOW_POSITION, stalls, or times out.
    On end, holds at current position via PID.
    """

    def __init__(self, intake: IntakeSubSystem) -> None:
        super().__init__()
        self.intake = intake
        self.addRequirements(self.intake)
        self._timer = Timer()

    def initialize(self) -> None:
        self._timer.restart()
        if not self.intake.homed:
            return
        self.intake.set_arm_duty_cycle(RETRACT_DUTYCYCLE)

    def end(self, interrupted: bool) -> None:
        self.intake.stop_arm()
        self.intake.set_arm_target_position(self.intake.get_arm_position())

    def isFinished(self) -> bool:
        if not self.intake.homed:
            return True
        if self._timer.hasElapsed(RETRACT_TIMEOUT_SECONDS):
            return True
        if self.intake.is_stalled:
            return True
        return self.intake.get_arm_position() <= STOW_POSITION
