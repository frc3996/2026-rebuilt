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

        motor_config = rev.SparkBaseConfig()
        motor_config.voltageCompensation(12.0)
        motor_config.smartCurrentLimit(50)
        motor_config.secondaryCurrentLimit(60)
        motor_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        motor_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        motor_config.closedLoop.P(0.0003, rev.ClosedLoopSlot.kSlot0)
        motor_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        motor_config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        motor_config.closedLoop.velocityFF(1.0 / NEO_FREE_SPEED_RPM, rev.ClosedLoopSlot.kSlot0)
        motor_config.closedLoop.outputRange(0, 1, rev.ClosedLoopSlot.kSlot0)

        self._right_motor.configure(
            motor_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # Invert left so both spin inward
        motor_config.inverted(True)
        self._left_motor.configure(
            motor_config,
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
