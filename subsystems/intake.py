
from commands2 import Subsystem, Command
from wpilib.shuffleboard import Shuffleboard
import rev

class IntakeSubSystem(Subsystem):
    """
    Intake Subsystem
    """
    def __init__(self):
        # Initilization
        self.up_down_motor = rev.SparkMax(55, rev.SparkMax.MotorType.kBrushed)
        self.up_down_closed_loop = self.up_down_motor.getClosedLoopController()

        self.roller_motor = rev.SparkMax(56, rev.SparkMax.MotorType.kBrushless)
        self.roller_encoder = self.roller_motor.getEncoder()
        self.roller_closed_loop = self.roller_motor.getClosedLoopController()

        self.global_config = rev.SparkBaseConfig()
        self.global_config.smartCurrentLimit(50)
        self.global_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)

        self.up_down_config = rev.SparkBaseConfig()
        self.up_down_config.apply(self.global_config)
        self.up_down_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        self.up_down_config.closedLoop.P(0.0001, rev.ClosedLoopSlot.kSlot0)
        self.up_down_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        self.up_down_config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        self.up_down_config.closedLoop.velocityFF(1.0 / 5767, rev.ClosedLoopSlot.kSlot0)
        self.up_down_config.closedLoop.outputRange(-1, 1, rev.ClosedLoopSlot.kSlot0)
        self.roller_motor.configure(self.up_down_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self.roller_config = rev.SparkBaseConfig()
        self.roller_config.apply(self.global_config)
        self.roller_config.encoder.positionConversionFactor(1)
        self.roller_config.encoder.velocityConversionFactor(1)
        self.roller_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        self.roller_config.closedLoop.P(0.0001, rev.ClosedLoopSlot.kSlot0)
        self.roller_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        self.roller_config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        self.roller_config.closedLoop.velocityFF(1.0 / 5767, rev.ClosedLoopSlot.kSlot0)
        self.roller_config.closedLoop.outputRange(-1, 1, rev.ClosedLoopSlot.kSlot0)
        self.roller_motor.configure(self.roller_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        tab = Shuffleboard.getTab("Intake")
        tab.addDouble("UpDown Amp. Target [A]", lambda: self.up_down_closed_loop.getSetpoint())
        tab.addDouble("UpDown Amp. Current [A]", lambda: self.up_down_motor.getOutputCurrent())
        tab.addDouble("Roller Velocity Target [RPM]", lambda: self.roller_closed_loop.getSetpoint())
        tab.addDouble("Roller Velocity Current [RPM]", lambda: self.get_roller_current_speed())

    def get_up_down_current_amp(self):
        return self.up_down_motor.getOutputCurrent()

    def set_up_down_target_amp(self, target_amp):
        self.up_down_closed_loop.setReference(target_amp, rev.SparkBase.ControlType.kCurrent, rev.ClosedLoopSlot.kSlot0)

    def get_roller_current_speed(self):
        return self.roller_encoder.getVelocity()

    def set_roller_target_speed(self, target_velocity):
        self.roller_closed_loop.setReference(target_velocity, rev.SparkBase.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0)

    def periodic(self):
        # Called on every loop
        pass

    def simulationPeriodic(self):
        # Called on every simulation loop
        pass
