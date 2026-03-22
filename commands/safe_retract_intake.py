import commands2
from wpilib import Timer

from subsystems.intake import (
    STALL_CURRENT_THRESHOLD,
    STALL_VELOCITY_THRESHOLD,
    STOW_POSITION,
    IntakeSubSystem,
)

RETRACT_DUTYCYCLE = -0.15  # Gentle duty cycle toward stow (negative = retract)  # TUNE
RETRACT_TIMEOUT_SECONDS = 10.0
RETRACT_STALL_CYCLES = 10


class SafeRetractIntake(commands2.Command):
    """
    Slowly retracts the intake arm to stow using a gentle duty cycle.
    Stops when the arm reaches STOW_POSITION or stalls.
    Requires the arm to be homed first.
    """

    def __init__(self, intake: IntakeSubSystem) -> None:
        super().__init__()
        self.intake = intake
        self.addRequirements(self.intake)
        self._stall_counter: int = 0
        self._timer = Timer()
        self._timed_out: bool = False

    def initialize(self) -> None:
        self._stall_counter = 0
        self._timed_out = False
        self._timer.restart()

        if not self.intake.homed:
            self._timed_out = True
            return

        self.intake.set_arm_duty_cycle(RETRACT_DUTYCYCLE)

    def execute(self) -> None:
        if self._timer.hasElapsed(RETRACT_TIMEOUT_SECONDS):
            self._timed_out = True
            return

        current = self.intake.get_arm_current()
        velocity = abs(self.intake.get_arm_velocity())

        if velocity < STALL_VELOCITY_THRESHOLD or current > STALL_CURRENT_THRESHOLD:
            self._stall_counter += 1
        else:
            self._stall_counter = 0

    def end(self, interrupted: bool) -> None:
        self.intake.stop_arm()
        # Hold at current position via PID
        if self.intake.homed:
            self.intake.set_arm_target_position(self.intake.get_arm_position())

    def isFinished(self) -> bool:
        if self._timed_out:
            return True
        if self._stall_counter >= RETRACT_STALL_CYCLES:
            return True
        return self.intake.get_arm_position() <= STOW_POSITION
