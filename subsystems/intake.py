import math

import ntcore
import rev
from commands2 import Subsystem
from constants import CANIds


# Homing constants
HOMING_VOLTAGE = 0.75  # Volts toward deployed hard stop (positive)  # TUNE
STALL_CURRENT_THRESHOLD = 8.0  # Amps  # TUNE
STALL_VELOCITY_THRESHOLD = 20.0  # RPM  # TUNE
STALL_CONFIRM_CYCLES = 5  # Consecutive loops (~100ms at 20ms loop)
HOMING_TIMEOUT_SECONDS = 5.0

# Position constants
DEPLOY_POSITION = 0.0  # Encoder zero at deployed hard stop (home position)
STOW_POSITION = -3.0  # Retracted/stowed position in motor turns  # TUNE

# Angle constants
MIN_ANGLE_DEG = 0.0  # Degrees at min rotations (fully retracted)  # TUNE
MAX_ANGLE_DEG = 90.0  # Degrees at max rotations (fully deployed)  # TUNE

# Gravity feedforward
KG = 0.03  # Duty-cycle feedforward at horizontal  # TUNE

# PID defaults (slot 0 — position)
ARM_KP = 0.1  # TUNE
ARM_KI = 0.0
ARM_KD = 0.01  # TUNE


class IntakeSubSystem(Subsystem):
    """
    Intake subsystem — deployable arm with roller for game piece pickup.
    Arm must be homed against deployed hard stop before position control is valid.
    Soft limits are set by manually driving to each end and calling
    set_min_limit() / set_max_limit().
    """

    def __init__(self) -> None:
        super().__init__()
        self.up_down_motor = rev.SparkMax(CANIds.INTAKE_ARM, rev.SparkMax.MotorType.kBrushless)
        self.up_down_encoder = self.up_down_motor.getEncoder()
        self.up_down_closed_loop = self.up_down_motor.getClosedLoopController()
        self.homed: bool = False
        self.limits_set: bool = False

        # Rotation limits — set via set_min_limit / set_max_limit
        self.min_rotations: float = -4.0  # default until calibrated  # TUNE
        self.max_rotations: float = 0.0  # deployed hard stop

        self.roller_motor = rev.SparkMax(CANIds.INTAKE_ROLLER, rev.SparkMax.MotorType.kBrushless)
        self.roller_encoder = self.roller_motor.getEncoder()
        self.roller_closed_loop = self.roller_motor.getClosedLoopController()

        # Arm config — position controlled, brake mode
        self.up_down_config = rev.SparkBaseConfig()
        self.up_down_config.inverted(True)
        self.up_down_config.smartCurrentLimit(25)
        self.up_down_config.secondaryCurrentLimit(30)
        self.up_down_config.IdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.up_down_config.closedLoop.setFeedbackSensor(
            rev.FeedbackSensor.kPrimaryEncoder
        )
        self.up_down_config.closedLoop.P(ARM_KP, rev.ClosedLoopSlot.kSlot0)
        self.up_down_config.closedLoop.I(ARM_KI, rev.ClosedLoopSlot.kSlot0)
        self.up_down_config.closedLoop.D(ARM_KD, rev.ClosedLoopSlot.kSlot0)
        self.up_down_config.closedLoop.outputRange(-1, 1, rev.ClosedLoopSlot.kSlot0)
        self.up_down_config.softLimit.forwardSoftLimit(self.max_rotations)
        self.up_down_config.softLimit.forwardSoftLimitEnabled(True)
        self.up_down_config.softLimit.reverseSoftLimit(self.min_rotations)
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

        self._arm_target: float = 0.0
        self._roller_target: float = 0.0

        table = ntcore.NetworkTableInstance.getDefault().getTable("Intake")
        self._arm_amps_pub = table.getDoubleTopic("Arm Amps").publish()
        self._arm_position_pub = table.getDoubleTopic("Arm Position Turns").publish()
        self._arm_target_pub = table.getDoubleTopic("Arm Target Turns").publish()
        self._arm_velocity_pub = table.getDoubleTopic("Arm Velocity RPM").publish()
        self._arm_homed_pub = table.getBooleanTopic("Arm Homed").publish()
        self._arm_limits_set_pub = table.getBooleanTopic("Arm Limits Set").publish()
        self._roller_velocity_pub = table.getDoubleTopic(
            "Roller Velocity RPM"
        ).publish()
        self._roller_target_pub = table.getDoubleTopic("Roller Target RPM").publish()
        self._roller_amps_pub = table.getDoubleTopic("Roller Amps").publish()

    # ── Arm control ────────────────────────────────────────────────

    def get_up_down_current_amp(self) -> float:
        """Return current draw of the arm motor in amps."""
        return self.up_down_motor.getOutputCurrent()

    def set_up_down_target_amp(self, target_amp: float) -> None:
        """Drive arm with raw current control (amps)."""
        self.up_down_closed_loop.setReference(
            target_amp, rev.SparkBase.ControlType.kCurrent, rev.ClosedLoopSlot.kSlot0
        )

    def set_up_down_target_position(self, target_position: float) -> None:
        """Drive arm to a position in motor turns (requires homing)."""
        if not self.homed:
            return
        target_position = max(
            self.min_rotations, min(target_position, self.max_rotations)
        )
        self._arm_target = target_position
        self.up_down_closed_loop.setReference(
            target_position,
            rev.SparkBase.ControlType.kPosition,
            rev.ClosedLoopSlot.kSlot0,
        )

    def set_angle(self, degrees: float) -> None:
        """Drive arm to an angle in degrees with gravity feedforward.

        Requires homing and limits to be set. Clamps to
        [MIN_ANGLE_DEG, MAX_ANGLE_DEG] and linearly maps to rotations.
        """
        if not self.homed or not self.limits_set:
            return

        degrees = max(MIN_ANGLE_DEG, min(degrees, MAX_ANGLE_DEG))

        # Linear interpolation: degrees → rotations
        if MAX_ANGLE_DEG == MIN_ANGLE_DEG:
            target_rot = self.min_rotations
        else:
            t = (degrees - MIN_ANGLE_DEG) / (MAX_ANGLE_DEG - MIN_ANGLE_DEG)
            target_rot = self.min_rotations + t * (self.max_rotations - self.min_rotations)

        # Gravity feedforward from current angle
        current_rot = self.up_down_encoder.getPosition()
        if self.max_rotations == self.min_rotations:
            current_angle_deg = MIN_ANGLE_DEG
        else:
            t_cur = (current_rot - self.min_rotations) / (self.max_rotations - self.min_rotations)
            current_angle_deg = MIN_ANGLE_DEG + t_cur * (MAX_ANGLE_DEG - MIN_ANGLE_DEG)
        ff = KG * math.cos(math.radians(current_angle_deg))

        self._arm_target = target_rot
        self.up_down_closed_loop.setReference(
            target_rot,
            rev.SparkBase.ControlType.kPosition,
            rev.ClosedLoopSlot.kSlot0,
            ff,
        )

    def deploy(self) -> None:
        """Move arm to deployed (home) position."""
        self.set_up_down_target_position(DEPLOY_POSITION)

    # ── Homing helpers ─────────────────────────────────────────────

    def enable_soft_limits(self) -> None:
        """Enable soft limits after homing — encoder zero is now meaningful."""
        cfg = rev.SparkBaseConfig()
        cfg.softLimit.forwardSoftLimitEnabled(True)
        cfg.softLimit.reverseSoftLimitEnabled(True)
        self.up_down_motor.configure(
            cfg,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

    def disable_soft_limits(self) -> None:
        """Disable soft limits for homing — motor needs to move past current zero."""
        cfg = rev.SparkBaseConfig()
        cfg.softLimit.forwardSoftLimitEnabled(False)
        cfg.softLimit.reverseSoftLimitEnabled(False)
        self.up_down_motor.configure(
            cfg,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

    # ── Soft limit calibration ─────────────────────────────────────

    def set_min_limit(self) -> None:
        """Record current position as the reverse (min) soft limit."""
        self.min_rotations = self.up_down_encoder.getPosition()
        cfg = rev.SparkBaseConfig()
        cfg.softLimit.reverseSoftLimit(self.min_rotations)
        cfg.softLimit.reverseSoftLimitEnabled(True)
        self.up_down_motor.configure(
            cfg,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )
        self.limits_set = self.limits_set or (self.max_rotations != self.min_rotations)
        print(f"[Intake] Min limit set at {self.min_rotations:.3f} turns")

    def set_max_limit(self) -> None:
        """Record current position as the forward (max) soft limit."""
        self.max_rotations = self.up_down_encoder.getPosition()
        cfg = rev.SparkBaseConfig()
        cfg.softLimit.forwardSoftLimit(self.max_rotations)
        cfg.softLimit.forwardSoftLimitEnabled(True)
        self.up_down_motor.configure(
            cfg,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )
        self.limits_set = self.limits_set or (self.max_rotations != self.min_rotations)
        print(f"[Intake] Max limit set at {self.max_rotations:.3f} turns")

    # ── Default actions ────────────────────────────────────────────

    def stow(self) -> None:
        """Move arm to stow position if homed, otherwise stop. Also stops roller."""
        if self.homed:
            self.set_up_down_target_position(STOW_POSITION)
        else:
            self.up_down_motor.stopMotor()
        self.set_roller_target_speed(0)

    def stop(self) -> None:
        """Stop both arm and roller motors immediately."""
        self._arm_target = 0.0
        self._roller_target = 0.0
        self.up_down_motor.stopMotor()
        self.roller_motor.stopMotor()

    # ── Roller control ─────────────────────────────────────────────

    def get_roller_current_speed(self) -> float:
        """Return current roller velocity in RPM."""
        return self.roller_encoder.getVelocity()

    def set_roller_target_speed(self, target_velocity: float) -> None:
        """Set roller target velocity in RPM."""
        self._roller_target = target_velocity
        self.roller_closed_loop.setReference(
            target_velocity,
            rev.SparkBase.ControlType.kVelocity,
            rev.ClosedLoopSlot.kSlot0,
        )

    # ── Periodic ───────────────────────────────────────────────────

    def periodic(self) -> None:
        self._arm_amps_pub.set(self.up_down_motor.getOutputCurrent())
        self._arm_position_pub.set(self.up_down_encoder.getPosition())
        self._arm_target_pub.set(self._arm_target)
        self._arm_velocity_pub.set(self.up_down_encoder.getVelocity())
        self._arm_homed_pub.set(self.homed)
        self._arm_limits_set_pub.set(self.limits_set)
        self._roller_velocity_pub.set(self.roller_encoder.getVelocity())
        self._roller_target_pub.set(self._roller_target)
        self._roller_amps_pub.set(self.roller_motor.getOutputCurrent())

    def simulationPeriodic(self) -> None:
        pass
