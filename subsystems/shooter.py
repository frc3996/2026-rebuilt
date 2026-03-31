import ntcore
import rev
from commands2 import Subsystem

from constants import CANIds

# FF gain from CalibrateFF
KF = 1.9e-4

# P gains per slot — tune via TuneShot NT
KP_LOW = 7.5e-4  # Slot 0: < 1500 RPM
KP_MID = 5.0e-4  # Slot 1: 1500–3750 RPM
KP_HIGH = 3.5e-4  # Slot 2: > 3750 RPM

# Slot boundaries
_LOW_MID_BOUNDARY = 1500.0
_MID_HIGH_BOUNDARY = 3750.0


class ShooterSubSystem(Subsystem):
    """
    Shooter Subsystem — two motors on a shared flywheel shaft.
    Motor 43 follows motor 47.
    Uses 3 PID slots with different P gains for low/mid/high RPM.
    """

    def __init__(self):
        super().__init__()
        self._motor = rev.SparkMax(CANIds.SHOOTER_LEADER, rev.SparkMax.MotorType.kBrushless)
        self._encoder = self._motor.getEncoder()
        self._closed_loop = self._motor.getClosedLoopController()

        self._follower = rev.SparkMax(CANIds.SHOOTER_FOLLOWER, rev.SparkMax.MotorType.kBrushless)

        self._config = rev.SparkBaseConfig()
        self._config.voltageCompensation(10)
        self._config.smartCurrentLimit(30)
        self._config.secondaryCurrentLimit(40)
        self._config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        self._config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)

        # Slot 0: low RPM
        self._config.closedLoop.P(KP_LOW, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.velocityFF(KF, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.outputRange(0, 1, rev.ClosedLoopSlot.kSlot0)

        # Slot 1: mid RPM
        self._config.closedLoop.P(KP_MID, rev.ClosedLoopSlot.kSlot1)
        self._config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot1)
        self._config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot1)
        self._config.closedLoop.velocityFF(KF, rev.ClosedLoopSlot.kSlot1)
        self._config.closedLoop.outputRange(0, 1, rev.ClosedLoopSlot.kSlot1)

        # Slot 2: high RPM
        self._config.closedLoop.P(KP_HIGH, rev.ClosedLoopSlot.kSlot2)
        self._config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot2)
        self._config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot2)
        self._config.closedLoop.velocityFF(KF, rev.ClosedLoopSlot.kSlot2)
        self._config.closedLoop.outputRange(0, 1, rev.ClosedLoopSlot.kSlot2)

        self._motor.configure(
            self._config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        follower_config = rev.SparkBaseConfig()
        follower_config.voltageCompensation(10)
        follower_config.smartCurrentLimit(30)
        follower_config.secondaryCurrentLimit(40)
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

    @staticmethod
    def _slot_for_rpm(rpm: float) -> rev.ClosedLoopSlot:
        if rpm < _LOW_MID_BOUNDARY:
            return rev.ClosedLoopSlot.kSlot0
        if rpm < _MID_HIGH_BOUNDARY:
            return rev.ClosedLoopSlot.kSlot1
        return rev.ClosedLoopSlot.kSlot2

    def set_slot_p(self, slot: rev.ClosedLoopSlot, kp: float) -> None:
        """Update P gain for a single slot (lightweight CAN update)."""
        self._config.closedLoop.P(kp, slot)
        self._motor.configure(
            self._config,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

    def set_duty_cycle(self, output: float) -> None:
        self._motor.set(output)

    def get_current_speed(self):
        return self._encoder.getVelocity()

    def set_target_speed(self, target_velocity):
        self._target_speed = target_velocity
        self._closed_loop.setReference(
            target_velocity,
            rev.SparkBase.ControlType.kVelocity,
            self._slot_for_rpm(target_velocity),
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
