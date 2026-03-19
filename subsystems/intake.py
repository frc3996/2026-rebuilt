import ntcore
import rev
from commands2 import Subsystem


class IntakeSubSystem(Subsystem):
    """
    Intake Subsystem
    """

    STOW_POSITION = 0.0  # Motor turns at home position
    DEPLOY_POSITION = 3.0  # Fully deployed arm position in motor turns — tune on robot
    MIN_POSITION = 0.0  # Hard stop (home/stowed position) in motor turns
    MAX_POSITION = 4.0  # Maximum arm travel in motor turns — tune on robot
    HOMING_AMPS = -5.0
    HOMING_VELOCITY_THRESHOLD = 0.5

    def __init__(self):
        self.up_down_motor = rev.SparkMax(46, rev.SparkMax.MotorType.kBrushless)
        self.up_down_encoder = self.up_down_motor.getEncoder()
        self.up_down_closed_loop = self.up_down_motor.getClosedLoopController()
        self.homed = False

        self.roller_motor = rev.SparkMax(43, rev.SparkMax.MotorType.kBrushless)
        self.roller_encoder = self.roller_motor.getEncoder()
        self.roller_closed_loop = self.roller_motor.getClosedLoopController()

        # Arm config — position controlled, brake mode
        self.up_down_config = rev.SparkBaseConfig()
        self.up_down_config.smartCurrentLimit(25)
        self.up_down_config.secondaryCurrentLimit(30)
        self.up_down_config.IdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.up_down_config.closedLoop.setFeedbackSensor(
            rev.FeedbackSensor.kPrimaryEncoder
        )
        self.up_down_config.closedLoop.P(0.1, rev.ClosedLoopSlot.kSlot0)
        self.up_down_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        self.up_down_config.closedLoop.D(0.01, rev.ClosedLoopSlot.kSlot0)
        self.up_down_config.closedLoop.outputRange(-1, 1, rev.ClosedLoopSlot.kSlot0)
        self.up_down_config.softLimit.forwardSoftLimit(self.MAX_POSITION)
        self.up_down_config.softLimit.forwardSoftLimitEnabled(True)
        self.up_down_config.softLimit.reverseSoftLimit(self.MIN_POSITION)
        self.up_down_config.softLimit.reverseSoftLimitEnabled(True)
        self.up_down_motor.configure(
            self.up_down_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # Roller config — velocity controlled, coast mode
        self.roller_config = rev.SparkBaseConfig()
        self.roller_config.smartCurrentLimit(50)
        self.roller_config.secondaryCurrentLimit(60)
        self.roller_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        self.roller_config.closedLoop.setFeedbackSensor(
            rev.FeedbackSensor.kPrimaryEncoder
        )
        self.roller_config.closedLoop.P(0.0001, rev.ClosedLoopSlot.kSlot0)
        self.roller_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        self.roller_config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        self.roller_config.closedLoop.velocityFF(1.0 / 5767, rev.ClosedLoopSlot.kSlot0)
        self.roller_config.closedLoop.outputRange(0, 1, rev.ClosedLoopSlot.kSlot0)
        self.roller_motor.configure(
            self.roller_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self._arm_target = 0.0
        self._roller_target = 0.0

        table = ntcore.NetworkTableInstance.getDefault().getTable("Intake")
        self._arm_amps_pub = table.getDoubleTopic("Arm Amps").publish()
        self._arm_position_pub = table.getDoubleTopic("Arm Position Turns").publish()
        self._arm_target_pub = table.getDoubleTopic("Arm Target Turns").publish()
        self._arm_velocity_pub = table.getDoubleTopic("Arm Velocity RPM").publish()
        self._arm_homed_pub = table.getBooleanTopic("Arm Homed").publish()
        self._roller_velocity_pub = table.getDoubleTopic(
            "Roller Velocity RPM"
        ).publish()
        self._roller_target_pub = table.getDoubleTopic("Roller Target RPM").publish()
        self._roller_amps_pub = table.getDoubleTopic("Roller Amps").publish()

    def get_up_down_current_amp(self):
        return self.up_down_motor.getOutputCurrent()

    def set_up_down_target_amp(self, target_amp):
        self.up_down_closed_loop.setReference(
            target_amp, rev.SparkBase.ControlType.kCurrent, rev.ClosedLoopSlot.kSlot0
        )

    def set_up_down_target_position(self, target_position):
        if not self.homed:
            return
        target_position = max(
            self.MIN_POSITION, min(target_position, self.MAX_POSITION)
        )
        self._arm_target = target_position
        self.up_down_closed_loop.setReference(
            target_position,
            rev.SparkBase.ControlType.kPosition,
            rev.ClosedLoopSlot.kSlot0,
        )

    def deploy(self):
        self.set_up_down_target_position(self.DEPLOY_POSITION)

    def reset_encoder(self):
        self.up_down_encoder.setPosition(0)
        self.homed = True

    def is_stalled(self):
        return abs(self.up_down_encoder.getVelocity()) < self.HOMING_VELOCITY_THRESHOLD

    def enable_soft_limits(self):
        """Enable soft limits after homing — encoder zero is now meaningful."""
        cfg = rev.SparkBaseConfig()
        cfg.softLimit.forwardSoftLimitEnabled(True)
        cfg.softLimit.reverseSoftLimitEnabled(True)
        self.up_down_motor.configure(
            cfg,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

    def disable_soft_limits(self):
        """Disable soft limits for homing — motor needs to move past current zero."""
        cfg = rev.SparkBaseConfig()
        cfg.softLimit.forwardSoftLimitEnabled(False)
        cfg.softLimit.reverseSoftLimitEnabled(False)
        self.up_down_motor.configure(
            cfg,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

    def stow(self):
        if self.homed:
            self.set_up_down_target_position(self.STOW_POSITION)
        else:
            self.up_down_motor.stopMotor()
        self.set_roller_target_speed(0)

    def stop(self):
        self._arm_target = 0.0
        self._roller_target = 0.0
        self.up_down_motor.stopMotor()
        self.roller_motor.stopMotor()

    def get_roller_current_speed(self):
        return self.roller_encoder.getVelocity()

    def set_roller_target_speed(self, target_velocity):
        self._roller_target = target_velocity
        self.roller_closed_loop.setReference(
            target_velocity,
            rev.SparkBase.ControlType.kVelocity,
            rev.ClosedLoopSlot.kSlot0,
        )

    def periodic(self):
        self._arm_amps_pub.set(self.up_down_motor.getOutputCurrent())
        self._arm_position_pub.set(self.up_down_encoder.getPosition())
        self._arm_target_pub.set(self._arm_target)
        self._arm_velocity_pub.set(self.up_down_encoder.getVelocity())
        self._arm_homed_pub.set(self.homed)
        self._roller_velocity_pub.set(self.roller_encoder.getVelocity())
        self._roller_target_pub.set(self._roller_target)
        self._roller_amps_pub.set(self.roller_motor.getOutputCurrent())

    def simulationPeriodic(self):
        pass
