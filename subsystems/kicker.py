import ntcore
import rev
from commands2 import Subsystem

from constants import NEO_FREE_SPEED_RPM, CANIds


class KickerSubSystem(Subsystem):
    """
    Kicker Subsystem — two independent flywheels feeding balls into the shooter.
    Each motor has its own velocity PID loop running at the same target RPM.
    """

    def __init__(self):
        super().__init__()
        self._right_motor = rev.SparkMax(CANIds.KICKER_LEADER, rev.SparkMax.MotorType.kBrushless)
        self._right_encoder = self._right_motor.getEncoder()
        self._right_closed_loop = self._right_motor.getClosedLoopController()

        self._left_motor = rev.SparkMax(CANIds.KICKER_FOLLOWER, rev.SparkMax.MotorType.kBrushless)
        self._left_encoder = self._left_motor.getEncoder()
        self._left_closed_loop = self._left_motor.getClosedLoopController()

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

        self._right_motor.configure(
            self._config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # Invert left so both spin inward
        self._config.inverted(True)
        self._left_motor.configure(
            self._config,
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

    def set_duty_cycle(self, output: float) -> None:
        self._right_motor.set(output)
        self._left_motor.set(output)

    def set_pid_gains(self, kp: float, ki: float, kd: float) -> None:
        """Update velocity PID gains (slot 0) for both motors at runtime."""
        self._config.closedLoop.P(kp, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.I(ki, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.D(kd, rev.ClosedLoopSlot.kSlot0)
        # Config has inverted=True from init; apply to left first
        self._left_motor.configure(
            self._config,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )
        # Temporarily un-invert for right motor
        self._config.inverted(False)
        self._right_motor.configure(
            self._config,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )
        self._config.inverted(True)

    def get_current_speed(self):
        return max(self._right_encoder.getVelocity(), self._left_encoder.getVelocity())

    def set_target_speed(self, target_velocity):
        self._target_speed = target_velocity
        self._right_closed_loop.setReference(
            target_velocity,
            rev.SparkBase.ControlType.kVelocity,
            rev.ClosedLoopSlot.kSlot0,
        )
        self._left_closed_loop.setReference(
            target_velocity,
            rev.SparkBase.ControlType.kVelocity,
            rev.ClosedLoopSlot.kSlot0,
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
