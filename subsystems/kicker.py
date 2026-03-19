import ntcore
import rev
from commands2 import Subsystem


class KickerSubSystem(Subsystem):
    """
    Kicker Subsystem — two motors on a shared shaft feeding balls into the shooter.
    Motor 51 follows motor 50.
    """

    def __init__(self):
        self.motor = rev.SparkMax(50, rev.SparkMax.MotorType.kBrushless)
        self.motor_encoder = self.motor.getEncoder()
        self.motor_closed_loop = self.motor.getClosedLoopController()

        self.follower_motor = rev.SparkMax(51, rev.SparkMax.MotorType.kBrushless)

        self.motor_config = rev.SparkBaseConfig()
        self.motor_config.smartCurrentLimit(50)
        self.motor_config.secondaryCurrentLimit(60)
        self.motor_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        self.motor_config.closedLoop.setFeedbackSensor(
            rev.FeedbackSensor.kPrimaryEncoder
        )
        self.motor_config.closedLoop.P(0.0001, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.closedLoop.velocityFF(1.0 / 5767, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.closedLoop.outputRange(0, 1, rev.ClosedLoopSlot.kSlot0)
        self.motor.configure(
            self.motor_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self.follower_config = rev.SparkBaseConfig()
        self.follower_config.smartCurrentLimit(50)
        self.follower_config.secondaryCurrentLimit(60)
        self.follower_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        self.follower_config.follow(50, True)
        self.follower_motor.configure(
            self.follower_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self._target_speed = 0.0

        table = ntcore.NetworkTableInstance.getDefault().getTable("Kicker")
        self._velocity_pub = table.getDoubleTopic("Velocity RPM").publish()
        self._target_pub = table.getDoubleTopic("Target RPM").publish()
        self._amps_pub = table.getDoubleTopic("Amps").publish()
        self._temp_pub = table.getDoubleTopic("Temperature C").publish()

    def get_current_speed(self):
        return self.motor_encoder.getVelocity()

    def set_target_speed(self, target_velocity):
        self._target_speed = target_velocity
        self.motor_closed_loop.setReference(
            target_velocity,
            rev.SparkBase.ControlType.kVelocity,
            rev.ClosedLoopSlot.kSlot0,
        )

    def stop(self):
        self._target_speed = 0.0
        self.motor.stopMotor()

    def periodic(self):
        self._velocity_pub.set(self.motor_encoder.getVelocity())
        self._target_pub.set(self._target_speed)
        self._amps_pub.set(self.motor.getOutputCurrent())
        self._temp_pub.set(self.motor.getMotorTemperature())

    def simulationPeriodic(self):
        pass
