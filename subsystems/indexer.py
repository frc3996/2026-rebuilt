import ntcore
import rev
from commands2 import Subsystem


class IndexerSubSystem(Subsystem):
    """
    Indexer Subsystem — single brushed conveyor motor feeding balls into the kicker.
    No encoder — uses duty cycle (percent output) control.
    """

    def __init__(self):
        self.conveyor_motor = rev.SparkMax(42, rev.SparkMax.MotorType.kBrushed)

        self.conveyor_config = rev.SparkBaseConfig()
        self.conveyor_config.smartCurrentLimit(30)
        self.conveyor_config.secondaryCurrentLimit(40)
        self.conveyor_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        self.conveyor_motor.configure(
            self.conveyor_config,
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
        self.conveyor_motor.set(self._target_output)

    def stop(self):
        self._target_output = 0.0
        self.conveyor_motor.stopMotor()

    def periodic(self):
        self._output_pub.set(self._target_output)
        self._amps_pub.set(self.conveyor_motor.getOutputCurrent())

    def simulationPeriodic(self):
        pass
