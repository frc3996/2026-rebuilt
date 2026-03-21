import math

import commands2
import ntcore
from wpilib import Timer

from constants import NEO_FREE_SPEED_RPM
from subsystems.shooter import ShooterSubSystem

# Auto-tune constants
RELAY_DELTA = 0.03  # Duty cycle swing on top of feedforward  # TUNE
TARGET_RPM = 1500.0  # RPM to oscillate around  # TUNE
SPINUP_SECONDS = 3.0  # Ramp up before starting oscillation  # TUNE
FF_BIAS = 0.05  # Extra duty cycle above theoretical FF to overcome friction  # TUNE
REQUIRED_CROSSINGS = 8  # 4 full cycles
AUTOTUNE_TIMEOUT_SECONDS = 15.0


class AutoTuneShooterCommand(commands2.Command):
    """
    Ziegler-Nichols relay auto-tune for the shooter velocity PID.

    Oscillates the flywheel around a target RPM using bang-bang control
    on top of feedforward. Measures oscillation period and amplitude,
    then computes and applies Z-N PI gains.
    """

    def __init__(self, shooter: ShooterSubSystem) -> None:
        super().__init__()
        self.shooter = shooter
        self.addRequirements(self.shooter)
        self._crossings: list[float] = []
        self._peaks: list[float] = []
        self._troughs: list[float] = []
        self._extremum: float = 0.0
        self._above: bool = False
        self._timer = Timer()
        self._timed_out: bool = False

        table = ntcore.NetworkTableInstance.getDefault().getTable("AutoTuneShooter")
        self._ku_pub = table.getDoubleTopic("Ku").publish()
        self._tu_pub = table.getDoubleTopic("Tu").publish()
        self._kp_pub = table.getDoubleTopic("Kp").publish()
        self._ki_pub = table.getDoubleTopic("Ki").publish()
        self._kd_pub = table.getDoubleTopic("Kd").publish()
        self._amplitude_pub = table.getDoubleTopic("Amplitude RPM").publish()

    def initialize(self) -> None:
        self._crossings = []
        self._peaks = []
        self._troughs = []
        self._timed_out = False
        self._spinning_up = True
        self._timer.restart()

        # Drive at feedforward + bias to spin up first
        ff = TARGET_RPM / NEO_FREE_SPEED_RPM + FF_BIAS
        self.shooter.set_duty_cycle(ff + RELAY_DELTA)

    def execute(self) -> None:
        velocity = self.shooter.get_current_speed()

        # Spin-up phase: wait until flywheel reaches target RPM
        if self._spinning_up:
            if velocity >= TARGET_RPM or self._timer.hasElapsed(SPINUP_SECONDS):
                self._spinning_up = False
                self._above = velocity >= TARGET_RPM
                self._extremum = velocity
                self._timer.restart()  # Reset timer for oscillation timeout
            return

        now_above = velocity >= TARGET_RPM

        # Detect zero-crossing (before updating extremum so we record the true peak/trough)
        if now_above != self._above:
            self._crossings.append(self._timer.get())
            if self._above:  # was above, now below — record peak
                self._peaks.append(self._extremum)
            else:  # was below, now above — record trough
                self._troughs.append(self._extremum)
            self._extremum = velocity
            self._above = now_above

        # Track extremum in current half-cycle
        if now_above:
            self._extremum = max(self._extremum, velocity)
        else:
            self._extremum = min(self._extremum, velocity)

        # Bang-bang around target with feedforward + bias
        ff = TARGET_RPM / NEO_FREE_SPEED_RPM + FF_BIAS
        if now_above:
            self.shooter.set_duty_cycle(ff - RELAY_DELTA)
        else:
            self.shooter.set_duty_cycle(ff + RELAY_DELTA)

        # Publish live PID estimates as crossings accumulate
        self._publish_pid()

        if self._timer.hasElapsed(AUTOTUNE_TIMEOUT_SECONDS):
            self._timed_out = True

    def _publish_pid(self) -> None:
        if len(self._crossings) < 2 or not self._peaks or not self._troughs:
            return

        avg_peak = sum(self._peaks) / len(self._peaks)
        avg_trough = sum(self._troughs) / len(self._troughs)
        amplitude = (avg_peak - avg_trough) / 2.0

        if amplitude < 1.0:
            return

        periods = [self._crossings[i + 1] - self._crossings[i] for i in range(len(self._crossings) - 1)]
        half_period = sum(periods) / len(periods)
        tu = half_period * 2.0
        ku = (4.0 * RELAY_DELTA) / (math.pi * amplitude)
        kp = 0.45 * ku
        ki = 0.54 * ku / tu

        self._ku_pub.set(ku)
        self._tu_pub.set(tu)
        self._kp_pub.set(kp)
        self._ki_pub.set(ki)
        self._kd_pub.set(0.0)
        self._amplitude_pub.set(amplitude)

    def end(self, interrupted: bool) -> None:
        self.shooter.stop()
        self._publish_pid()

        if interrupted or self._timed_out:
            return

        if len(self._crossings) < 2 or not self._peaks or not self._troughs:
            return

        avg_peak = sum(self._peaks) / len(self._peaks)
        avg_trough = sum(self._troughs) / len(self._troughs)
        amplitude = (avg_peak - avg_trough) / 2.0

        if amplitude < 1.0:
            return

        periods = [self._crossings[i + 1] - self._crossings[i] for i in range(len(self._crossings) - 1)]
        half_period = sum(periods) / len(periods)
        tu = half_period * 2.0
        ku = (4.0 * RELAY_DELTA) / (math.pi * amplitude)
        kp = 0.45 * ku
        ki = 0.54 * ku / tu
        kd = 0.0

        self.shooter.set_pid_gains(kp, ki, kd)

    def isFinished(self) -> bool:
        if self._timed_out:
            return True
        return len(self._crossings) >= REQUIRED_CROSSINGS
