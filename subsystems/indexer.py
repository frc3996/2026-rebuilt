
from commands2 import Subsystem, Command
from wpilib.shuffleboard import Shuffleboard
import rev

class IndexerSubSystem(Subsystem):
    """
    Indexer Subsystem
    """
    def __init__(self):
        # Initilization
        self.left_kicker_motor = rev.SparkMax(50, rev.SparkMax.MotorType.kBrushless)
        self.left_kicker_encoder = self.left_kicker_motor.getEncoder()
        self.left_kicker_closed_loop = self.left_kicker_motor.getClosedLoopController()

        self.right_kicker_motor = rev.SparkMax(51, rev.SparkMax.MotorType.kBrushless)
        self.right_kicker_encoder = self.right_kicker_motor.getEncoder()
        self.right_kicker_closed_loop = self.right_kicker_motor.getClosedLoopController()

        self.conveyor_motor = rev.SparkMax(52, rev.SparkMax.MotorType.kBrushless)
        self.conveyor_encoder = self.conveyor_motor.getEncoder()
        self.conveyor_closed_loop = self.conveyor_motor.getClosedLoopController()

        self.global_config = rev.SparkBaseConfig()
        self.global_config.smartCurrentLimit(50)
        self.global_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)

        self.left_kicker_config = rev.SparkBaseConfig()
        self.left_kicker_config.apply(self.global_config)
        self.left_kicker_config.encoder.positionConversionFactor(1)
        self.left_kicker_config.encoder.velocityConversionFactor(1)
        self.left_kicker_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        self.left_kicker_config.closedLoop.P(0.0001, rev.ClosedLoopSlot.kSlot0)
        self.left_kicker_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        self.left_kicker_config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        self.left_kicker_config.closedLoop.velocityFF(1.0 / 5767, rev.ClosedLoopSlot.kSlot0)
        self.left_kicker_config.closedLoop.outputRange(-1, 1, rev.ClosedLoopSlot.kSlot0)
        self.left_kicker_motor.configure(self.left_kicker_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self.right_kicker_config = rev.SparkBaseConfig()
        self.right_kicker_config.apply(self.global_config)
        self.right_kicker_config.encoder.positionConversionFactor(1)
        self.right_kicker_config.encoder.velocityConversionFactor(1)
        self.right_kicker_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        self.right_kicker_config.closedLoop.P(0.0001, rev.ClosedLoopSlot.kSlot0)
        self.right_kicker_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        self.right_kicker_config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        self.right_kicker_config.closedLoop.velocityFF(1.0 / 5767, rev.ClosedLoopSlot.kSlot0)
        self.right_kicker_config.closedLoop.outputRange(-1, 1, rev.ClosedLoopSlot.kSlot0)
        self.right_kicker_motor.configure(self.right_kicker_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self.conveyor_config = rev.SparkBaseConfig()
        self.conveyor_config.apply(self.global_config)
        self.conveyor_config.encoder.positionConversionFactor(1)
        self.conveyor_config.encoder.velocityConversionFactor(1)
        self.conveyor_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        self.conveyor_config.closedLoop.P(0.0001, rev.ClosedLoopSlot.kSlot0)
        self.conveyor_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        self.conveyor_config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        self.conveyor_config.closedLoop.velocityFF(1.0 / 5767, rev.ClosedLoopSlot.kSlot0)
        self.conveyor_config.closedLoop.outputRange(-1, 1, rev.ClosedLoopSlot.kSlot0)
        self.conveyor_motor.configure(self.conveyor_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        tab = Shuffleboard.getTab("Indexer")
        tab.addDouble("Left Kicker Velocity Target [RPM]", lambda: self.left_kicker_closed_loop.getSetpoint())
        tab.addDouble("Left Kicker Velocity Current [RPM]", lambda: self.get_left_kicker_current_speed())
        tab.addDouble("Right Kicker Velocity Target [RPM]", lambda: self.right_kicker_closed_loop.getSetpoint())
        tab.addDouble("Right Kicker Velocity Current [RPM]", lambda: self.get_right_kicker_current_speed())
        tab.addDouble("Conveyor Velocity Target [RPM]", lambda: self.conveyor_closed_loop.getSetpoint())
        tab.addDouble("Conveyor Velocity Current [RPM]", lambda: self.get_conveyor_current_speed())

    def get_left_kicker_current_speed(self):
        return self.left_kicker_encoder.getVelocity()

    def set_left_kicker_target_speed(self, target_velocity):
        self.left_kicker_closed_loop.setReference(target_velocity, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)

    def get_right_kicker_current_speed(self):
        return self.left_kicker_encoder.getVelocity()

    def set_right_kicker_target_speed(self, target_velocity):
        self.left_kicker_closed_loop.setReference(target_velocity, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)

    def get_conveyor_current_speed(self):
        return self.left_kicker_encoder.getVelocity()

    def set_conveyor_target_speed(self, target_velocity):
        self.left_kicker_closed_loop.setReference(target_velocity, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)

    def periodic(self):
        # Called on every loop
        pass

    def simulationPeriodic(self):
        # Called on every simulation loop
        pass
