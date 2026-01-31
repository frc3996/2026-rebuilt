
import wpilib
from commands2 import Subsystem, Command
from wpilib.shuffleboard import Shuffleboard

class ClimbSubsystem(Subsystem):
    """
    Vision subsystem optimized for a single Limelight.
    No state-based tag filtering — always accepts all visible tags.
    """

    def __init__(self):
        # Initialization
        self.solenoid = wpilib.Solenoid(
            moduleType=wpilib.PneumaticsModuleType.CTREPCM, channel=2
        )

        # Compressor connected to a PH
        self.compressor = wpilib.Compressor(wpilib.PneumaticsModuleType.CTREPCM)

        tab = Shuffleboard.getTab("Climber")
        tab.add("Solenoid", self.solenoid)
        tab.add("Compressor", self.compressor)
        tab.addBoolean("Compressor Active", lambda: self.compressor.isEnabled())

    def periodic(self):
        # Called on every loop
        pass

    def setState(self, state: bool):
        self.solenoid.set(state)

    def simulationPeriodic(self):
        # Called on every simulation loop
        pass


