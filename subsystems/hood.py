
from commands2 import Subsystem, Command
from wpilib.shuffleboard import Shuffleboard
import rev

class HoodSubSystem(Subsystem):
    """
    Hood subsystem
    """
    def __init__(self):
        # Initilization
        self.motor = rev.SparkMax(45, rev.SparkMax.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()
        self.closed_loop = self.motor.getClosedLoopController()
        
        self.global_config = rev.SparkBaseConfig()
        self.global_config.smartCurrentLimit(50)
        self.global_config.IdleMode(rev.SparkBaseConfig.IdleMode.kBrake)

        self.motor_config = rev.SparkBaseConfig()
        self.motor_config.apply(self.global_config)
        self.motor_config.encoder.positionConversionFactor(1)
        self.motor_config.encoder.velocityConversionFactor(1)
        self.motor_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kAbsoluteEncoder)
        self.motor_config.closedLoop.P(0.1, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.closedLoop.outputRange(-1, 1, rev.ClosedLoopSlot.kSlot0)
        self.motor.configure(self.motor_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        tab = Shuffleboard.getTab("Hood")
        tab.addDouble("Position Target [NA]", lambda: self.closed_loop.getSetpoint())
        tab.addDouble("Position Current [NA]", lambda: self.get_current_position())

    def get_current_position(self):
        self.encoder.getPosition()

    def set_target_position(self, target_position):
        self.closed_loop.setReference(target_position, rev.SparkBase.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0)


    def periodic(self):
        # Called on every loop
        pass

    def simulationPeriodic(self):
        # Called on every simulation loop
        pass
