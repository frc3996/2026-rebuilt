import ntcore
import wpilib
from commands2 import Subsystem

from constants import PneumaticChannels


class ClimbSubsystem(Subsystem):
    """
    Pneumatic climber — binary extend/retract via solenoid.
    """

    def __init__(self):
        super().__init__()
        self.solenoid = wpilib.Solenoid(
            moduleType=wpilib.PneumaticsModuleType.CTREPCM, channel=PneumaticChannels.CLIMBER_SOLENOID
        )
        self.compressor = wpilib.Compressor(wpilib.PneumaticsModuleType.CTREPCM)

        table = ntcore.NetworkTableInstance.getDefault().getTable("Climber")
        self._solenoid_pub = table.getBooleanTopic("Solenoid Extended").publish()
        self._compressor_pub = table.getBooleanTopic("Compressor Active").publish()

    def setState(self, state: bool):
        self.solenoid.set(state)

    def periodic(self):
        self._solenoid_pub.set(self.solenoid.get())
        self._compressor_pub.set(self.compressor.isEnabled())

    def simulationPeriodic(self):
        pass
