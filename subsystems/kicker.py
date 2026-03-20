import ntcore
import rev
from commands2 import Subsystem
from constants import CANIds, NEO_FREE_SPEED_RPM


class KickerSubSystem(Subsystem):
    """
    Kicker Subsystem — two motors on a shared shaft feeding balls into the shooter.
    Motor 48 follows motor 41.
    """

    def __init__(self):
        super().__init__()
        self._motor = rev.SparkMax(CANIds.KICKER_LEADER, rev.SparkMax.MotorType.kBrushless)
        self._encoder = self._motor.getEncoder()
        self._closed_loop = self._motor.getClosedLoopController()

        self._follower = rev.SparkMax(CANIds.KICKER_FOLLOWER, rev.SparkMax.MotorType.kBrushless)

        leader_config = rev.SparkBaseConfig()
        leader_config.voltageCompensation(12.0)
        leader_config.smartCurrentLimit(50)
        leader_config.secondaryCurrentLimit(60)
        leader_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        leader_config.closedLoop.setFeedbackSensor(
            rev.FeedbackSensor.kPrimaryEncoder
        )
        leader_config.closedLoop.P(0.0001, rev.ClosedLoopSlot.kSlot0)
        leader_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        leader_config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        leader_config.closedLoop.velocityFF(1.0 / NEO_FREE_SPEED_RPM, rev.ClosedLoopSlot.kSlot0)
        leader_config.closedLoop.outputRange(0, 1, rev.ClosedLoopSlot.kSlot0)
        self._motor.configure(
            leader_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        follower_config = rev.SparkBaseConfig()
        follower_config.voltageCompensation(12.0)
        follower_config.smartCurrentLimit(50)
        follower_config.secondaryCurrentLimit(60)
        follower_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        follower_config.follow(CANIds.KICKER_LEADER, True)
        follower_config.signals.primaryEncoderPositionPeriodMs(500)
        follower_config.signals.primaryEncoderVelocityPeriodMs(500)
        self._follower.configure(
            follower_config,
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
