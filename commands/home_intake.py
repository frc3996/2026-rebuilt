import commands2
from wpilib import Timer

from subsystems.intake import (
    HOMING_DUTYCYCLE,
    HOMING_STARTUP_SECONDS,
    HOMING_TIMEOUT_SECONDS,
    STALL_CONFIRM_CYCLES,
    STALL_CURRENT_THRESHOLD,
    STALL_VELOCITY_THRESHOLD,
    IntakeSubSystem,
)


class HomeIntake(commands2.Command):
    """
    Drives the intake arm toward its deployed hard stop at constant duty cycle.
    Waits for a startup grace period, then detects stall via velocity near zero
    or current spike. On confirmed stall, zeros the encoder and marks homed.
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
        self.intake.set_arm_duty_cycle(HOMING_DUTYCYCLE)

    def execute(self) -> None:
        if self._timer.hasElapsed(HOMING_TIMEOUT_SECONDS):
            self._timed_out = True
            return

        # Skip stall detection during startup — motor needs time to get moving
        if not self._timer.hasElapsed(HOMING_STARTUP_SECONDS):
            return

        current = self.intake.get_arm_current()
        velocity = abs(self.intake.get_arm_velocity())

        if velocity < STALL_VELOCITY_THRESHOLD or current > STALL_CURRENT_THRESHOLD:
            self._stall_counter += 1
        else:
            self._stall_counter = 0

    def end(self, interrupted: bool) -> None:
        self.intake.stop_arm()
        if not interrupted and not self._timed_out:
            self.intake.reset_arm_encoder()
            self.intake.enable_soft_limits()
        else:
            self.intake.homed = False

    def isFinished(self) -> bool:
        if self._timed_out:
            return True
        return self._stall_counter >= STALL_CONFIRM_CYCLES
