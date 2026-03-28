import ntcore
import rev
from commands2 import Subsystem

from constants import CANIds

# FF gains from CalibrateFF
RIGHT_KF = 1.8e-4
LEFT_KF = 1.9e-4

# P gains per slot — tune via TuneShot NT
# Right motor
RIGHT_KP_LOW = 1.5e-4   # Slot 0: < 1500 RPM
RIGHT_KP_MID = 1.0e-4   # Slot 1: 1500–3750 RPM
RIGHT_KP_HIGH = 0.7e-4  # Slot 2: > 3750 RPM

# Left motor
LEFT_KP_LOW = 1.5e-4    # Slot 0: < 1500 RPM
LEFT_KP_MID = 1.0e-4    # Slot 1: 1500–3750 RPM
LEFT_KP_HIGH = 0.7e-4   # Slot 2: > 3750 RPM

# Slot boundaries
_LOW_MID_BOUNDARY = 1500.0
_MID_HIGH_BOUNDARY = 3750.0


def _slot_for_rpm(rpm: float) -> rev.ClosedLoopSlot:
    if rpm < _LOW_MID_BOUNDARY:
        return rev.ClosedLoopSlot.kSlot0
    if rpm < _MID_HIGH_BOUNDARY:
        return rev.ClosedLoopSlot.kSlot1
    return rev.ClosedLoopSlot.kSlot2


def _configure_3_slots(config: rev.SparkBaseConfig, kp_low: float, kp_mid: float, kp_high: float, kf: float) -> None:
    """Configure slots 0/1/2 with different P gains, shared FF/I/D."""
    for slot, kp in [
        (rev.ClosedLoopSlot.kSlot0, kp_low),
        (rev.ClosedLoopSlot.kSlot1, kp_mid),
        (rev.ClosedLoopSlot.kSlot2, kp_high),
    ]:
        config.closedLoop.P(kp, slot)
        config.closedLoop.I(0, slot)
        config.closedLoop.D(0, slot)
        config.closedLoop.velocityFF(kf, slot)
        config.closedLoop.outputRange(0, 1, slot)


class KickerSubSystem(Subsystem):
    """
    Kicker Subsystem — two independent flywheels feeding balls into the shooter.
    Each motor has its own velocity PID with 3 slots for RPM-dependent P gains.
    """

    def __init__(self):
        super().__init__()
        self._right_motor = rev.SparkMax(CANIds.KICKER_LEADER, rev.SparkMax.MotorType.kBrushless)
        self._right_encoder = self._right_motor.getEncoder()
        self._right_closed_loop = self._right_motor.getClosedLoopController()

        self._left_motor = rev.SparkMax(CANIds.KICKER_FOLLOWER, rev.SparkMax.MotorType.kBrushless)
        self._left_encoder = self._left_motor.getEncoder()
        self._left_closed_loop = self._left_motor.getClosedLoopController()

        self._right_config = rev.SparkBaseConfig()
        self._right_config.voltageCompensation(10)
        self._right_config.smartCurrentLimit(50)
        self._right_config.secondaryCurrentLimit(60)
        self._right_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        self._right_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        _configure_3_slots(self._right_config, RIGHT_KP_LOW, RIGHT_KP_MID, RIGHT_KP_HIGH, RIGHT_KF)
        self._right_motor.configure(
            self._right_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self._left_config = rev.SparkBaseConfig()
        self._left_config.voltageCompensation(10)
        self._left_config.smartCurrentLimit(50)
        self._left_config.secondaryCurrentLimit(60)
        self._left_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        self._left_config.inverted(True)
        self._left_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        _configure_3_slots(self._left_config, LEFT_KP_LOW, LEFT_KP_MID, LEFT_KP_HIGH, LEFT_KF)
        self._left_motor.configure(
            self._left_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self._target_speed = 0.0

        table = ntcore.NetworkTableInstance.getDefault().getTable("Kicker")
        self._right_velocity_pub = table.getDoubleTopic("Right Velocity RPM").publish()
        self._left_velocity_pub = table.getDoubleTopic("Left Velocity RPM").publish()
        self._target_pub = table.getDoubleTopic("Target RPM").publish()
        self._right_amps_pub = table.getDoubleTopic("Right Amps").publish()
        self._left_amps_pub = table.getDoubleTopic("Left Amps").publish()

    def set_right_slot_p(self, slot: rev.ClosedLoopSlot, kp: float) -> None:
        self._right_config.closedLoop.P(kp, slot)
        self._right_motor.configure(
            self._right_config,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

    def set_left_slot_p(self, slot: rev.ClosedLoopSlot, kp: float) -> None:
        self._left_config.closedLoop.P(kp, slot)
        self._left_motor.configure(
            self._left_config,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

    def set_duty_cycle(self, output: float) -> None:
        self._right_motor.set(output)
        self._left_motor.set(output)

    def set_right_duty_cycle(self, output: float) -> None:
        self._right_motor.set(output)

    def set_left_duty_cycle(self, output: float) -> None:
        self._left_motor.set(output)

    def get_right_speed(self) -> float:
        return self._right_encoder.getVelocity()

    def get_left_speed(self) -> float:
        return self._left_encoder.getVelocity()

    def get_current_speed(self) -> float:
        return max(self._right_encoder.getVelocity(), self._left_encoder.getVelocity())

    def set_target_speed(self, target_velocity):
        self._target_speed = target_velocity
        slot = _slot_for_rpm(target_velocity)
        self._right_closed_loop.setReference(
            target_velocity,
            rev.SparkBase.ControlType.kVelocity,
            slot,
        )
        self._left_closed_loop.setReference(
            target_velocity,
            rev.SparkBase.ControlType.kVelocity,
            slot,
        )

    def stop(self):
        self._target_speed = 0.0
        self._right_motor.stopMotor()
        self._left_motor.stopMotor()

    def periodic(self):
        self._right_velocity_pub.set(self._right_encoder.getVelocity())
        self._left_velocity_pub.set(self._left_encoder.getVelocity())
        self._target_pub.set(self._target_speed)
        self._right_amps_pub.set(self._right_motor.getOutputCurrent())
        self._left_amps_pub.set(self._left_motor.getOutputCurrent())

    def simulationPeriodic(self):
        pass
