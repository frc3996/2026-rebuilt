
from commands2 import Subsystem, Command
from wpilib.shuffleboard import Shuffleboard
import rev

class ShooterSubSystem(Subsystem):
    """
    Subsystem Demo
    """
    def __init__(self):
        # Initilization
        self.motor_left_1 = rev.SparkMax(40, rev.SparkMax.MotorType.kBrushless)
        self.motor_left_1_encoder = self.motor_left_1.getEncoder()
        self.motor_left_1_closed_loop = self.motor_left_1.getClosedLoopController()
        self.motor_left_2 = rev.SparkMax(41, rev.SparkMax.MotorType.kBrushless)
        self.motor_right_1 = rev.SparkMax(42, rev.SparkMax.MotorType.kBrushless)
        self.motor_right_2 = rev.SparkMax(43, rev.SparkMax.MotorType.kBrushless)

        self.global_config = rev.SparkBaseConfig()
        self.global_config.smartCurrentLimit(50)
        self.global_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)

        self.motor_left_1_config = rev.SparkBaseConfig()
        self.motor_left_1_config.apply(self.global_config)
        self.motor_left_1_config.encoder.positionConversionFactor(1)
        self.motor_left_1_config.encoder.velocityConversionFactor(1)
        self.motor_left_1_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        self.motor_left_1_config.closedLoop.P(0.0001, rev.ClosedLoopSlot.kSlot0)
        self.motor_left_1_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        self.motor_left_1_config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        self.motor_left_1_config.closedLoop.velocityFF(1.0 / 5767, rev.ClosedLoopSlot.kSlot0)
        self.motor_left_1_config.closedLoop.outputRange(-1, 1, rev.ClosedLoopSlot.kSlot0)
        self.motor_left_1.configure(self.motor_left_1_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self.motor_left_2_config = rev.SparkBaseConfig()
        self.motor_left_2_config.apply(self.global_config)
        self.motor_left_2_config.follow(40, False)
        self.motor_left_2.configure(self.motor_left_2_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self.motor_right_1_config = rev.SparkBaseConfig()
        self.motor_right_1_config.apply(self.global_config)
        self.motor_right_1_config.follow(40, False)
        self.motor_right_1.configure(self.motor_right_1_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self.motor_right_2_config = rev.SparkBaseConfig()
        self.motor_right_2_config.apply(self.global_config)
        self.motor_right_2_config.follow(40, False)
        self.motor_right_2.configure(self.motor_right_2_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        tab = Shuffleboard.getTab("Shooter")
        tab.addDouble("Velocity Target [RPM]", lambda: self.motor_left_1_closed_loop.getSetpoint())
        tab.addDouble("Velocity Current [RPM]", lambda: self.get_current_speed())

    def get_current_speed(self):
        return self.motor_left_1_encoder.getVelocity()

    def set_target_speed(self, target_velocity):
        self.motor_left_1_closed_loop.setReference(target_velocity, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)

    def periodic(self):
        # Called on every loop
        pass

    def simulationPeriodic(self):
        # Called on every simulation loop
        pass
