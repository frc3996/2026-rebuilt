import commands2
import ntcore
from wpilib import Timer

# Calibration parameters
VOLTAGE_STEP = 0.5  # Volts per step
MAX_VOLTAGE = 10.0  # Max test voltage
SETTLE_SECONDS = 2.0  # Time to wait for steady state at each step
MIN_VELOCITY_RPM = 50.0  # Ignore steps where motor hasn't started spinning
CALIBRATION_TIMEOUT = 60.0  # Safety timeout


class CalibrateFF(commands2.Command):
    """
    Feedforward calibration via voltage ramp test.

    Steps through increasing voltages, records steady-state RPM at each step,
    then fits V = kS + kV * RPM via least-squares linear regression.
    Publishes kS, kV, and the SparkMax-ready kF value to NetworkTables.
    """

    def __init__(
        self,
        subsystem,
        nt_table_name: str,
        set_output=None,
        get_speed=None,
        stop=None,
    ) -> None:
        super().__init__()
        self._subsystem = subsystem
        self._set_output = set_output or subsystem.set_duty_cycle
        self._get_speed = get_speed or subsystem.get_current_speed
        self._stop = stop or subsystem.stop
        self.addRequirements(subsystem)

        self._timer = Timer()
        self._step_timer = Timer()
        self._current_voltage: float = 0.0
        self._data: list[tuple[float, float]] = []  # (voltage, rpm)
        self._done: bool = False
        self._timed_out: bool = False

        table = ntcore.NetworkTableInstance.getDefault().getTable(nt_table_name)
        self._ks_pub = table.getDoubleTopic("kS (volts)").publish()
        self._kv_pub = table.getDoubleTopic("kV (volts per RPM)").publish()
        self._kf_pub = table.getDoubleTopic("kF (SparkMax FF)").publish()
        self._step_pub = table.getDoubleTopic("Current Voltage").publish()
        self._rpm_pub = table.getDoubleTopic("Current RPM").publish()
        self._points_pub = table.getIntegerTopic("Data Points").publish()
        self._status_pub = table.getStringTopic("Status").publish()

    def initialize(self) -> None:
        self._data = []
        self._current_voltage = VOLTAGE_STEP
        self._done = False
        self._timed_out = False
        self._timer.restart()
        self._step_timer.restart()
        self._set_output(self._current_voltage / 12.0)
        self._status_pub.set("Running")

    def execute(self) -> None:
        if self._timer.hasElapsed(CALIBRATION_TIMEOUT):
            self._timed_out = True
            return

        rpm = self._get_speed()
        self._step_pub.set(self._current_voltage)
        self._rpm_pub.set(rpm)

        if not self._step_timer.hasElapsed(SETTLE_SECONDS):
            return

        if abs(rpm) > MIN_VELOCITY_RPM:
            self._data.append((self._current_voltage, abs(rpm)))
            self._points_pub.set(len(self._data))

        self._current_voltage += VOLTAGE_STEP
        if self._current_voltage > MAX_VOLTAGE:
            self._done = True
            return

        self._set_output(self._current_voltage / 12.0)
        self._step_timer.restart()

    def end(self, interrupted: bool) -> None:
        self._stop()

        if interrupted or self._timed_out or len(self._data) < 2:
            self._status_pub.set(
                "Timed out" if self._timed_out else
                "Interrupted" if interrupted else
                "Not enough data points"
            )
            return

        n = len(self._data)
        sum_v = sum(v for v, _ in self._data)
        sum_rpm = sum(r for _, r in self._data)
        sum_vr = sum(v * r for v, r in self._data)
        sum_r2 = sum(r * r for _, r in self._data)

        denom = n * sum_r2 - sum_rpm * sum_rpm
        if abs(denom) < 1e-12:
            self._status_pub.set("Regression failed")
            return

        kv = (n * sum_vr - sum_rpm * sum_v) / denom
        ks = (sum_v - kv * sum_rpm) / n
        kf = kv / 12.0

        self._ks_pub.set(ks)
        self._kv_pub.set(kv)
        self._kf_pub.set(kf)
        self._status_pub.set("Done")

    def isFinished(self) -> bool:
        return self._done or self._timed_out
