import commands2
from wpilib import Timer

from subsystems.intake import (
    HOMING_TIMEOUT_SECONDS,
    HOMING_VOLTAGE,
    STALL_CONFIRM_CYCLES,
    STALL_CURRENT_THRESHOLD,
    STALL_VELOCITY_THRESHOLD,
    IntakeSubSystem,
)


class HomeIntake(commands2.Command):
    """
    Drives the intake arm toward its deployed hard stop using constant voltage.
    Detects stall via current + velocity thresholds held for multiple
    consecutive loops. On confirmed stall, zeros the encoder and marks homed.
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
        self.intake.disable_soft_limits()
        self.intake.set_arm_voltage(HOMING_VOLTAGE)

    def execute(self) -> None:
        current = self.intake.get_arm_current()
        velocity = abs(self.intake.get_arm_velocity())

        if current > STALL_CURRENT_THRESHOLD and velocity < STALL_VELOCITY_THRESHOLD:
            self._stall_counter += 1
        else:
            self._stall_counter = 0

        if self._timer.hasElapsed(HOMING_TIMEOUT_SECONDS):
            self._timed_out = True

    def end(self, interrupted: bool) -> None:
        self.intake.stop_arm()
        if not interrupted and not self._timed_out:
            self.intake.reset_arm_encoder()
            self.intake.enable_soft_limits()

    def isFinished(self) -> bool:
        if self._timed_out:
            return True
        return self._stall_counter >= STALL_CONFIRM_CYCLES
