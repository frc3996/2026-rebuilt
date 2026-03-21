import ntcore
import rev
from commands2 import Subsystem

from constants import NEO_FREE_SPEED_RPM, CANIds

# Homing constants
HOMING_DUTYCYCLE = 0.75  # Duty cycle toward deployed hard stop (positive)  # TUNE
STALL_CURRENT_THRESHOLD = 8.0  # Amps  # TUNE
STALL_VELOCITY_THRESHOLD = 20.0  # RPM  # TUNE
STALL_CONFIRM_CYCLES = 5  # Consecutive loops (~100ms at 20ms loop)
HOMING_TIMEOUT_SECONDS = 5.0

# Stall protection during position control
POSITION_STALL_CURRENT = 10.0  # Amps  # TUNE
POSITION_STALL_VELOCITY = 10.0  # RPM  # TUNE
POSITION_STALL_CYCLES = 10  # ~200ms at 20ms loop  # TUNE

# Position constants
DEPLOY_POSITION = 0.0  # Encoder zero at deployed hard stop (home position)
STOW_POSITION = -3.0  # Retracted/stowed position in motor turns  # TUNE

# PID defaults (slot 0 — position)
ARM_KP = 0.15  # TUNE
ARM_KI = 0.0
ARM_KD = 0.005  # TUNE



class IntakeSubSystem(Subsystem):
    """
    Intake subsystem — deployable arm with roller for game piece pickup.
    Arm must be homed against deployed hard stop before position control is valid.
    Soft limits are set by manually driving to each end and calling
    set_min_limit() / set_max_limit().
    """

    def __init__(self) -> None:
        super().__init__()
        self._arm_motor = rev.SparkMax(CANIds.INTAKE_ARM, rev.SparkMax.MotorType.kBrushless)
        self._arm_encoder = self._arm_motor.getEncoder()
        self._arm_closed_loop = self._arm_motor.getClosedLoopController()
        self.homed: bool = False
        self.limits_set: bool = False
        self._soft_limits_enabled: bool = True

        # Rotation limits — set via set_min_limit / set_max_limit
        self.min_rotations: float = -4.0  # default until calibrated  # TUNE
        self.max_rotations: float = 0.0  # deployed hard stop

        self._roller_motor = rev.SparkMax(CANIds.INTAKE_ROLLER, rev.SparkMax.MotorType.kBrushless)
        self._roller_encoder = self._roller_motor.getEncoder()
        self._roller_closed_loop = self._roller_motor.getClosedLoopController()

        # Arm config — position controlled, brake mode
        self._arm_config = rev.SparkBaseConfig()
        self._arm_config.inverted(True)  # positive = toward stow (retract)
        self._arm_config.voltageCompensation(12.0)
        self._arm_config.smartCurrentLimit(25)
        self._arm_config.secondaryCurrentLimit(30)
        self._arm_config.IdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self._arm_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        self._arm_config.closedLoop.P(ARM_KP, rev.ClosedLoopSlot.kSlot0)
        self._arm_config.closedLoop.I(ARM_KI, rev.ClosedLoopSlot.kSlot0)
        self._arm_config.closedLoop.D(ARM_KD, rev.ClosedLoopSlot.kSlot0)
        self._arm_config.closedLoop.outputRange(-1, 1, rev.ClosedLoopSlot.kSlot0)
        self._arm_config.softLimit.forwardSoftLimit(self.max_rotations)
        self._arm_config.softLimit.forwardSoftLimitEnabled(True)
        self._arm_config.softLimit.reverseSoftLimit(self.min_rotations)
        self._arm_config.softLimit.reverseSoftLimitEnabled(True)
        self._arm_motor.configure(
            self._arm_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # Roller config — velocity controlled, coast mode
        roller_config = rev.SparkBaseConfig()
        roller_config.voltageCompensation(12.0)
        roller_config.smartCurrentLimit(50)
        roller_config.secondaryCurrentLimit(60)
        roller_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        roller_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        roller_config.closedLoop.P(0.0003, rev.ClosedLoopSlot.kSlot0)
        roller_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        roller_config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        roller_config.closedLoop.velocityFF(1.0 / NEO_FREE_SPEED_RPM, rev.ClosedLoopSlot.kSlot0)
        roller_config.closedLoop.outputRange(0, 1, rev.ClosedLoopSlot.kSlot0)
        self._roller_motor.configure(
            roller_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self._arm_target: float = 0.0
        self._roller_target: float = 0.0
        self._arm_position_active: bool = False
        self._stall_cycle: int = 0
        self._stall_count: int = 0

        table = ntcore.NetworkTableInstance.getDefault().getTable("Intake")
        self._arm_amps_pub = table.getDoubleTopic("Arm Amps").publish()
        self._arm_position_pub = table.getDoubleTopic("Arm Position Turns").publish()
        self._arm_target_pub = table.getDoubleTopic("Arm Target Turns").publish()
        self._arm_velocity_pub = table.getDoubleTopic("Arm Velocity RPM").publish()
        self._arm_homed_pub = table.getBooleanTopic("Arm Homed").publish()
        self._arm_stall_cycle_pub = table.getIntegerTopic("Arm Stall Cycle").publish()
        self._arm_stall_count_pub = table.getIntegerTopic("Arm Stall Count").publish()
        self._arm_limits_set_pub = table.getBooleanTopic("Arm Limits Set").publish()
        self._roller_velocity_pub = table.getDoubleTopic("Roller Velocity RPM").publish()
        self._roller_target_pub = table.getDoubleTopic("Roller Target RPM").publish()
        self._roller_amps_pub = table.getDoubleTopic("Roller Amps").publish()

    # ── Low-level arm motor access (for homing / auto-tune commands) ──

    def set_arm_duty_cycle(self, output: float) -> None:
        self._arm_motor.set(output)

    def stop_arm(self) -> None:
        self._arm_motor.stopMotor()

    def get_arm_current(self) -> float:
        return self._arm_motor.getOutputCurrent()

    def get_arm_velocity(self) -> float:
        return self._arm_encoder.getVelocity()

    def get_arm_position(self) -> float:
        return self._arm_encoder.getPosition()

    def reset_arm_encoder(self) -> None:
        """Zero the arm encoder and mark homed."""
        self._arm_encoder.setPosition(0.0)
        self.homed = True

    def set_arm_pid_gains(self, kp: float, ki: float, kd: float) -> None:
        """Update arm position PID gains (slot 0) at runtime."""
        self._arm_closed_loop.setP(kp, rev.ClosedLoopSlot.kSlot0)
        self._arm_closed_loop.setI(ki, rev.ClosedLoopSlot.kSlot0)
        self._arm_closed_loop.setD(kd, rev.ClosedLoopSlot.kSlot0)

    # ── Arm control ────────────────────────────────────────────────

    def set_arm_target_position(self, target_position: float) -> None:
        """Drive arm to a position in motor turns (requires homing)."""
        if not self.homed:
            return
        target_position = max(self.min_rotations, min(target_position, self.max_rotations))
        self._arm_target = target_position
        self._arm_position_active = True
        self._arm_closed_loop.setReference(
            target_position,
            rev.SparkBase.ControlType.kPosition,
            rev.ClosedLoopSlot.kSlot0,
        )

    def deploy(self) -> None:
        """Move arm to deployed (home) position."""
        self.set_arm_target_position(DEPLOY_POSITION)

    # ── Homing helpers ─────────────────────────────────────────────

    def _apply_soft_limit_config(self) -> None:
        """Push current soft limit state to the motor controller."""
        self._arm_config.softLimit.forwardSoftLimit(self.max_rotations)
        self._arm_config.softLimit.forwardSoftLimitEnabled(self._soft_limits_enabled)
        self._arm_config.softLimit.reverseSoftLimit(self.min_rotations)
        self._arm_config.softLimit.reverseSoftLimitEnabled(self._soft_limits_enabled)
        self._arm_motor.configure(
            self._arm_config,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

    def enable_soft_limits(self) -> None:
        """Enable soft limits after homing."""
        self._soft_limits_enabled = True
        self._apply_soft_limit_config()

    def disable_soft_limits(self) -> None:
        """Disable soft limits for homing."""
        self._soft_limits_enabled = False
        self._apply_soft_limit_config()

    # ── Soft limit calibration ─────────────────────────────────────

    def set_min_limit(self) -> None:
        """Record current position as the reverse (min) soft limit."""
        self.min_rotations = self._arm_encoder.getPosition()
        self._soft_limits_enabled = True
        self._apply_soft_limit_config()
        self.limits_set = self.limits_set or (self.max_rotations != self.min_rotations)

    def set_max_limit(self) -> None:
        """Record current position as the forward (max) soft limit."""
        self.max_rotations = self._arm_encoder.getPosition()
        self._soft_limits_enabled = True
        self._apply_soft_limit_config()
        self.limits_set = self.limits_set or (self.max_rotations != self.min_rotations)

    # ── Default actions ────────────────────────────────────────────

    def hold(self) -> None:
        """Hold arm at current position if homed, otherwise stop. Stop roller."""
        if self.homed:
            if not self._arm_position_active:
                self.set_arm_target_position(self.get_arm_position())
        else:
            self._arm_motor.stopMotor()
        self._roller_motor.stopMotor()

    def stow(self) -> None:
        """Move arm to stow position if homed, otherwise stop. Also stops roller."""
        if self.homed:
            self.set_arm_target_position(STOW_POSITION)
        else:
            self._arm_motor.stopMotor()
        self.set_roller_target_speed(0)

    # ── Roller control ─────────────────────────────────────────────

    def set_roller_target_speed(self, target_velocity: float) -> None:
        self._roller_target = target_velocity
        self._roller_closed_loop.setReference(
            target_velocity,
            rev.SparkBase.ControlType.kVelocity,
            rev.ClosedLoopSlot.kSlot0,
        )

    # ── Periodic ───────────────────────────────────────────────────

    def periodic(self) -> None:
        arm_current = self._arm_motor.getOutputCurrent()
        arm_velocity = abs(self._arm_encoder.getVelocity())

        # Stall detection during position control
        if self._arm_position_active:
            if arm_current > POSITION_STALL_CURRENT and arm_velocity < POSITION_STALL_VELOCITY:
                self._stall_cycle += 1
            else:
                self._stall_cycle = 0
            if self._stall_cycle >= POSITION_STALL_CYCLES:
                self._stall_count += 1
                self._stall_cycle = 0
                self._arm_position_active = False
                self._arm_motor.stopMotor()

        self._arm_amps_pub.set(arm_current)
        self._arm_position_pub.set(self._arm_encoder.getPosition())
        self._arm_target_pub.set(self._arm_target)
        self._arm_velocity_pub.set(self._arm_encoder.getVelocity())
        self._arm_homed_pub.set(self.homed)
        self._arm_stall_cycle_pub.set(self._stall_cycle)
        self._arm_stall_count_pub.set(self._stall_count)
        self._arm_limits_set_pub.set(self.limits_set)
        self._roller_velocity_pub.set(self._roller_encoder.getVelocity())
        self._roller_target_pub.set(self._roller_target)
        self._roller_amps_pub.set(self._roller_motor.getOutputCurrent())

    def simulationPeriodic(self) -> None:
        pass
