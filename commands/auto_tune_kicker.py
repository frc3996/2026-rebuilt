import math

import commands2
import ntcore
from wpilib import Timer

from constants import NEO_FREE_SPEED_RPM
from subsystems.kicker import KickerSubSystem

# Auto-tune constants
RELAY_DELTA = 0.05  # Duty cycle swing on top of feedforward  # TUNE
TARGET_RPM = 3000.0  # RPM to oscillate around  # TUNE
REQUIRED_CROSSINGS = 8  # 4 full cycles
AUTOTUNE_TIMEOUT_SECONDS = 15.0


class AutoTuneKickerCommand(commands2.Command):
    """
    Ziegler-Nichols relay auto-tune for the kicker velocity PID.

    Oscillates both kicker flywheels around a target RPM using bang-bang
    control on top of feedforward. Measures oscillation period and amplitude,
    then computes and applies the same Z-N PI gains to both motors.
    """

    def __init__(self, kicker: KickerSubSystem) -> None:
        super().__init__()
        self.kicker = kicker
        self.addRequirements(self.kicker)
        self._crossings: list[float] = []
        self._peaks: list[float] = []
        self._troughs: list[float] = []
        self._extremum: float = 0.0
        self._above: bool = False
        self._timer = Timer()
        self._timed_out: bool = False

        table = ntcore.NetworkTableInstance.getDefault().getTable("AutoTuneKicker")
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
        self._above = self.kicker.get_current_speed() >= TARGET_RPM
        self._extremum = self.kicker.get_current_speed()
        self._timed_out = False
        self._timer.restart()

        ff = TARGET_RPM / NEO_FREE_SPEED_RPM
        output = ff - RELAY_DELTA if self._above else ff + RELAY_DELTA
        self.kicker.set_duty_cycle(output)

    def execute(self) -> None:
        velocity = self.kicker.get_current_speed()
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

        # Bang-bang around target with feedforward
        ff = TARGET_RPM / NEO_FREE_SPEED_RPM
        if now_above:
            self.kicker.set_duty_cycle(ff - RELAY_DELTA)
        else:
            self.kicker.set_duty_cycle(ff + RELAY_DELTA)

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
        self.kicker.stop()
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

        self.kicker.set_pid_gains(kp, ki, kd)

    def isFinished(self) -> bool:
        if self._timed_out:
            return True
        return len(self._crossings) >= REQUIRED_CROSSINGS
