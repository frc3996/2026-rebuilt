import commands2
from wpilib import Timer

from subsystems.hood import (
    HOMING_DUTYCYCLE,
    HOMING_STARTUP_SECONDS,
    HOMING_TIMEOUT_SECONDS,
    STALL_CONFIRM_CYCLES,
    STALL_CURRENT_THRESHOLD,
    STALL_VELOCITY_THRESHOLD,
    HoodSubSystem,
)


class HomeHood(commands2.Command):
    """
    Drives the hood toward a hard stop at low duty cycle.
    Waits for a startup grace period, then detects stall via velocity
    staying near zero for multiple consecutive loops.
    On confirmed stall, zeros the encoder and marks homed.
    """

    def __init__(self, hood: HoodSubSystem) -> None:
        super().__init__()
        self.hood = hood
        self.addRequirements(self.hood)
        self._stall_counter: int = 0
        self._timer = Timer()
        self._timed_out: bool = False

    def initialize(self) -> None:
        self._stall_counter = 0
        self._timed_out = False
        self._timer.restart()
        self.hood.disable_soft_limits()
        self.hood.set_duty_cycle(HOMING_DUTYCYCLE)

    def execute(self) -> None:
        if self._timer.hasElapsed(HOMING_TIMEOUT_SECONDS):
            self._timed_out = True
            return

        # Skip stall detection during startup — motor needs time to get moving
        if not self._timer.hasElapsed(HOMING_STARTUP_SECONDS):
            return

        current = self.hood.get_output_current()
        velocity = abs(self.hood.get_velocity())

        if velocity < STALL_VELOCITY_THRESHOLD or current > STALL_CURRENT_THRESHOLD:
            self._stall_counter += 1
        else:
            self._stall_counter = 0

    def end(self, interrupted: bool) -> None:
        self.hood.stop()
        if not interrupted and not self._timed_out:
            self.hood.reset_encoder()
            self.hood.enable_soft_limits()
        else:
            self.hood.is_homed = False

    def isFinished(self) -> bool:
        if self._timed_out:
            return True
        return self._stall_counter >= STALL_CONFIRM_CYCLES
