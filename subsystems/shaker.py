import rev
from commands2 import Subsystem

from constants import CANIds


class ShakerSubSystem(Subsystem):
    """
    Shaker Subsystem — single brushless motor with an offset weight.
    Spinning the motor vibrates the robot to help agitate balls through
    the indexer/kicker path.
    """

    SHAKER_OUTPUT = 1.0  # Duty cycle when running  # TUNE

    def __init__(self):
        super().__init__()
        self._motor = rev.SparkMax(CANIds.SHAKER, rev.SparkMax.MotorType.kBrushless)

        config = rev.SparkBaseConfig()
        config.voltageCompensation(12)
        config.smartCurrentLimit(30)
        config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        self._motor.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

    def run_shaker(self):
        """Spin the offset weight at full duty cycle."""
        self._motor.set(self.SHAKER_OUTPUT)

    def stop(self):
        """Stop the shaker motor."""
        self._motor.stopMotor()
