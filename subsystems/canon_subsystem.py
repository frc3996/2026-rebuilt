
import commands2
import commands2.cmd
import rev

import constants


class CanonSubsystem(commands2.Subsystem):
    """Shooter subsystem with velocity control"""

    # Create a new ArmSubsystem
    def __init__(self) -> None:
        self.motor = rev.SparkMax(constants.Shooter1ID, rev.SparkMax.MotorType.kBrushless)
        self.pid_config = rev.SparkMaxConfig()
        self.pid_config.closedLoop.pid(1, 0, 0, rev.ClosedLoopSlot.kSlot0)
        self.pid_config.closedLoop.outputRange(-100, 100, rev.ClosedLoopSlot.kSlot0)
        self.closed_loop = self.motor.getClosedLoopController()
        self.closed_loop.setReference(0, self.motor.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)

    def __set_target(self, target):
        self.closed_loop.setReference(target, self.motor.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)
        # print(f"SETTING TARGET TO {target}")

    def setTarget(self, target) -> commands2.Command:
        return commands2.cmd.runOnce(lambda: self.__set_target(target))
