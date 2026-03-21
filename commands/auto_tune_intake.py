import math

import commands2
import ntcore
from wpilib import Timer

from subsystems.intake import IntakeSubSystem

# Auto-tune constants
RELAY_OUTPUT = 0.12  # Duty cycle for bang-bang oscillation  # TUNE
REQUIRED_CROSSINGS = 8  # 4 full cycles
AUTOTUNE_TIMEOUT_SECONDS = 15.0


class AutoTuneIntakeCommand(commands2.Command):
    """
    Ziegler-Nichols relay auto-tune for the intake arm position PID.

    Oscillates the arm around its midpoint using bang-bang control,
    records zero-crossings, then computes and applies Z-N PD gains.
    Requires the arm to be homed before running.
    """

    def __init__(self, intake: IntakeSubSystem) -> None:
        super().__init__()
        self.intake = intake
        self.addRequirements(self.intake)
        self._midpoint: float = 0.0
        self._crossings: list[float] = []
        self._above: bool = False
        self._timer = Timer()
        self._timed_out: bool = False

        table = ntcore.NetworkTableInstance.getDefault().getTable("AutoTuneIntake")
        self._ku_pub = table.getDoubleTopic("Ku").publish()
        self._tu_pub = table.getDoubleTopic("Tu").publish()
        self._kp_pub = table.getDoubleTopic("Kp").publish()
        self._ki_pub = table.getDoubleTopic("Ki").publish()
        self._kd_pub = table.getDoubleTopic("Kd").publish()

    def initialize(self) -> None:
        if not self.intake.homed or not self.intake.limits_set:
            self._timed_out = True
            return

        self._midpoint = (self.intake.min_rotations + self.intake.max_rotations) / 2.0
        self._crossings = []
        self._above = self.intake.get_arm_position() >= self._midpoint
        self._timed_out = False
        self._timer.restart()

        output = RELAY_OUTPUT if not self._above else -RELAY_OUTPUT
        self.intake.set_arm_duty_cycle(output)

    def execute(self) -> None:
        pos = self.intake.get_arm_position()
        now_above = pos >= self._midpoint

        if now_above != self._above:
            self._crossings.append(self._timer.get())
            self._above = now_above

        if now_above:
            self.intake.set_arm_duty_cycle(-RELAY_OUTPUT)
        else:
            self.intake.set_arm_duty_cycle(RELAY_OUTPUT)

        if self._timer.hasElapsed(AUTOTUNE_TIMEOUT_SECONDS):
            self._timed_out = True

    def end(self, interrupted: bool) -> None:
        self.intake.stop_arm()

        if interrupted or self._timed_out:
            return

        if len(self._crossings) < 2:
            return

        periods = [self._crossings[i + 1] - self._crossings[i] for i in range(len(self._crossings) - 1)]
        half_period = sum(periods) / len(periods)
        tu = half_period * 2.0

        ku = (4.0 * RELAY_OUTPUT) / math.pi

        kp = 0.6 * ku
        kd = 0.075 * ku * tu
        ki = 0.0

        self.intake.set_arm_pid_gains(kp, ki, kd)

        self._ku_pub.set(ku)
        self._tu_pub.set(tu)
        self._kp_pub.set(kp)
        self._ki_pub.set(ki)
        self._kd_pub.set(kd)

    def isFinished(self) -> bool:
        if self._timed_out:
            return True
        return len(self._crossings) >= REQUIRED_CROSSINGS
