import ntcore
import rev
from commands2 import Subsystem

from constants import NEO_FREE_SPEED_RPM, CANIds


class ShooterSubSystem(Subsystem):
    """
    Shooter Subsystem — two motors on a shared flywheel shaft.
    Motor 43 follows motor 47.
    """

    def __init__(self):
        super().__init__()
        self._motor = rev.SparkMax(CANIds.SHOOTER_LEADER, rev.SparkMax.MotorType.kBrushless)
        self._encoder = self._motor.getEncoder()
        self._closed_loop = self._motor.getClosedLoopController()

        self._follower = rev.SparkMax(CANIds.SHOOTER_FOLLOWER, rev.SparkMax.MotorType.kBrushless)

        self._config = rev.SparkBaseConfig()
        self._config.voltageCompensation(11.0)
        self._config.smartCurrentLimit(50)
        self._config.secondaryCurrentLimit(60)
        self._config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        self._config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        self._config.closedLoop.P(0.0003, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.velocityFF(1.0 / NEO_FREE_SPEED_RPM, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.outputRange(0, 1, rev.ClosedLoopSlot.kSlot0)
        self._motor.configure(
            self._config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        follower_config = rev.SparkBaseConfig()
        follower_config.voltageCompensation(11.0)
        follower_config.smartCurrentLimit(50)
        follower_config.secondaryCurrentLimit(60)
        follower_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        follower_config.follow(CANIds.SHOOTER_LEADER, True)
        follower_config.signals.primaryEncoderPositionPeriodMs(500)
        follower_config.signals.primaryEncoderVelocityPeriodMs(500)
        self._follower.configure(
            follower_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self._target_speed = 0.0

        table = ntcore.NetworkTableInstance.getDefault().getTable("Shooter")
        self._velocity_pub = table.getDoubleTopic("Velocity RPM").publish()
        self._target_pub = table.getDoubleTopic("Target RPM").publish()
        self._amps_pub = table.getDoubleTopic("Amps").publish()
        self._temp_pub = table.getDoubleTopic("Temperature C").publish()

    def set_duty_cycle(self, output: float) -> None:
        self._motor.set(output)

    def set_pid_gains(self, kp: float, ki: float, kd: float) -> None:
        """Update velocity PID gains (slot 0) at runtime."""
        self._config.closedLoop.P(kp, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.I(ki, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.D(kd, rev.ClosedLoopSlot.kSlot0)
        self._motor.configure(
            self._config,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

    def get_current_speed(self):
        return self._encoder.getVelocity()

    def set_target_speed(self, target_velocity):
        self._target_speed = target_velocity
        self._closed_loop.setReference(
            target_velocity,
            rev.SparkBase.ControlType.kVelocity,
            rev.ClosedLoopSlot.kSlot0,
        )

    def stop(self):
        self._target_speed = 0.0
        self._motor.stopMotor()

    def periodic(self):
        self._velocity_pub.set(self._encoder.getVelocity())
        self._target_pub.set(self._target_speed)
        self._amps_pub.set(self._motor.getOutputCurrent())
        self._temp_pub.set(self._motor.getMotorTemperature())

    def simulationPeriodic(self):
        pass
