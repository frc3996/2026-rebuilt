import ntcore
import rev
from commands2 import Subsystem
from constants import CANIds


class IndexerSubSystem(Subsystem):
    """
    Indexer Subsystem — single brushed conveyor motor feeding balls into the kicker.
    No encoder — uses duty cycle (percent output) control.
    """

    def __init__(self):
        super().__init__()
        self._motor = rev.SparkMax(CANIds.INDEXER, rev.SparkMax.MotorType.kBrushed)

        config = rev.SparkBaseConfig()
        config.inverted(True)  # positive = toward kicker (upward)
        config.voltageCompensation(12.0)
        config.smartCurrentLimit(30)
        config.secondaryCurrentLimit(40)
        config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        self._motor.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self._target_output = 0.0

        table = ntcore.NetworkTableInstance.getDefault().getTable("Indexer")
        self._output_pub = table.getDoubleTopic("Conveyor Output").publish()
        self._amps_pub = table.getDoubleTopic("Conveyor Amps").publish()

    def set_target_output(self, output):
        """Set conveyor duty cycle output (-1.0 to 1.0)."""
        self._target_output = max(-1.0, min(output, 1.0))
        self._motor.set(self._target_output)

    def stop(self):
        self._target_output = 0.0
        self._motor.stopMotor()

    def periodic(self):
        self._output_pub.set(self._target_output)
        self._amps_pub.set(self._motor.getOutputCurrent())

    def simulationPeriodic(self):
        pass
