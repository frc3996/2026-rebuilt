import ntcore
import rev
from commands2 import Subsystem


class HoodSubSystem(Subsystem):
    """
    Hood subsystem — adjustable shooter angle with relative encoder.
    Must be homed against a hard stop before position control is valid.
    """

    STOW_POSITION = 0.0  # Motor turns at home position
    MIN_POSITION = 0.0  # Hard stop (home position) in motor turns
    MAX_POSITION = 5.0  # Maximum hood travel in motor turns — tune on robot
    HOMING_AMPS = -5.0
    HOMING_VELOCITY_THRESHOLD = 0.5

    def __init__(self):
        self.motor = rev.SparkMax(45, rev.SparkMax.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()
        self.closed_loop = self.motor.getClosedLoopController()
        self.homed = False

        self.motor_config = rev.SparkBaseConfig()
        self.motor_config.smartCurrentLimit(25)
        self.motor_config.secondaryCurrentLimit(30)
        self.motor_config.IdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.motor_config.closedLoop.setFeedbackSensor(
            rev.FeedbackSensor.kPrimaryEncoder
        )
        self.motor_config.closedLoop.P(0.1, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.closedLoop.D(0.01, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.closedLoop.outputRange(-1, 1, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.softLimit.forwardSoftLimit(self.MAX_POSITION)
        self.motor_config.softLimit.forwardSoftLimitEnabled(True)
        self.motor_config.softLimit.reverseSoftLimit(self.MIN_POSITION)
        self.motor_config.softLimit.reverseSoftLimitEnabled(True)
        self.motor.configure(
            self.motor_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self._target_position = 0.0

        table = ntcore.NetworkTableInstance.getDefault().getTable("Hood")
        self._position_pub = table.getDoubleTopic("Position Turns").publish()
        self._target_pub = table.getDoubleTopic("Target Turns").publish()
        self._velocity_pub = table.getDoubleTopic("Velocity RPM").publish()
        self._amps_pub = table.getDoubleTopic("Amps").publish()
        self._homed_pub = table.getBooleanTopic("Homed").publish()

    def get_current_position(self):
        return self.encoder.getPosition()

    def set_target_position(self, target_position):
        if not self.homed:
            return
        target_position = max(
            self.MIN_POSITION, min(target_position, self.MAX_POSITION)
        )
        self._target_position = target_position
        self.closed_loop.setReference(
            target_position,
            rev.SparkBase.ControlType.kPosition,
            rev.ClosedLoopSlot.kSlot0,
        )

    def set_target_amps(self, target_amps):
        self.closed_loop.setReference(
            target_amps, rev.SparkBase.ControlType.kCurrent, rev.ClosedLoopSlot.kSlot0
        )

    def reset_encoder(self):
        self.encoder.setPosition(0)
        self.homed = True

    def is_stalled(self):
        return abs(self.encoder.getVelocity()) < self.HOMING_VELOCITY_THRESHOLD

    def enable_soft_limits(self):
        """Enable soft limits after homing — encoder zero is now meaningful."""
        cfg = rev.SparkBaseConfig()
        cfg.softLimit.forwardSoftLimitEnabled(True)
        cfg.softLimit.reverseSoftLimitEnabled(True)
        self.motor.configure(
            cfg,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

    def disable_soft_limits(self):
        """Disable soft limits for homing — motor needs to move past current zero."""
        cfg = rev.SparkBaseConfig()
        cfg.softLimit.forwardSoftLimitEnabled(False)
        cfg.softLimit.reverseSoftLimitEnabled(False)
        self.motor.configure(
            cfg,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

    def stow(self):
        if self.homed:
            self.set_target_position(self.STOW_POSITION)
        else:
            self.motor.stopMotor()

    def stop(self):
        self._target_position = 0.0
        self.motor.stopMotor()

    def periodic(self):
        self._position_pub.set(self.encoder.getPosition())
        self._target_pub.set(self._target_position)
        self._velocity_pub.set(self.encoder.getVelocity())
        self._amps_pub.set(self.motor.getOutputCurrent())
        self._homed_pub.set(self.homed)

    def simulationPeriodic(self):
        pass
