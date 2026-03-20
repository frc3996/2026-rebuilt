import math

import commands2
from wpilib import Timer
from subsystems.hood import HoodSubSystem


# Auto-tune constants
RELAY_OUTPUT = 0.12  # Duty cycle for bang-bang oscillation  # TUNE
REQUIRED_CROSSINGS = 8  # 4 full cycles
AUTOTUNE_TIMEOUT_SECONDS = 15.0


class AutoTuneHoodCommand(commands2.Command):
    """
    Ziegler-Nichols relay auto-tune for the hood position PID.

    Oscillates the hood around its midpoint using bang-bang control,
    records zero-crossings, then computes and applies Z-N PD gains.
    Requires the hood to be homed and soft limits set before running.
    """

    def __init__(self, hood: HoodSubSystem) -> None:
        super().__init__()
        self.hood = hood
        self.addRequirements(self.hood)
        self._midpoint: float = 0.0
        self._crossings: list[float] = []
        self._above: bool = False
        self._timer = Timer()
        self._timed_out: bool = False

    def initialize(self) -> None:
        if not self.hood.is_homed or not self.hood.limits_set:
            self._timed_out = True
            return

        self._midpoint = (self.hood.min_rotations + self.hood.max_rotations) / 2.0
        self._crossings = []
        self._above = self.hood.get_current_position() >= self._midpoint
        self._timed_out = False
        self._timer.restart()

        # Start initial direction
        output = RELAY_OUTPUT if not self._above else -RELAY_OUTPUT
        self.hood.set_duty_cycle(output)

    def execute(self) -> None:
        pos = self.hood.get_current_position()
        now_above = pos >= self._midpoint

        # Detect zero-crossing
        if now_above != self._above:
            self._crossings.append(self._timer.get())
            self._above = now_above

        # Bang-bang: drive toward midpoint
        if now_above:
            self.hood.set_duty_cycle(-RELAY_OUTPUT)
        else:
            self.hood.set_duty_cycle(RELAY_OUTPUT)

        if self._timer.hasElapsed(AUTOTUNE_TIMEOUT_SECONDS):
            self._timed_out = True

    def end(self, interrupted: bool) -> None:
        self.hood.stop()

        if interrupted or self._timed_out:
            return

        if len(self._crossings) < 2:
            return

        # Compute average period from crossing timestamps
        periods = [
            self._crossings[i + 1] - self._crossings[i]
            for i in range(len(self._crossings) - 1)
        ]
        half_period = sum(periods) / len(periods)
        tu = half_period * 2.0  # Full period = 2 half-periods

        # Ultimate gain
        ku = (4.0 * RELAY_OUTPUT) / math.pi

        # Ziegler-Nichols PD gains
        kp = 0.6 * ku
        kd = 0.075 * ku * tu
        ki = 0.0

        self.hood.set_pid_gains(kp, ki, kd)

    def isFinished(self) -> bool:
        if self._timed_out:
            return True
        return len(self._crossings) >= REQUIRED_CROSSINGS
