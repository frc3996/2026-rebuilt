import math

import ntcore
import rev
from commands2 import Subsystem
from constants import CANIds


# Homing constants
HOMING_VOLTAGE = -0.75  # Volts toward hard stop  # TUNE
STALL_CURRENT_THRESHOLD = 8.0  # Amps  # TUNE
STALL_VELOCITY_THRESHOLD = 20.0  # RPM  # TUNE
STALL_CONFIRM_CYCLES = 5  # Consecutive loops (~100ms at 20ms loop)
HOMING_TIMEOUT_SECONDS = 5.0

# Position / angle constants
STOW_POSITION = 0.0  # Motor turns at home position
MIN_ANGLE_DEG = 0.0  # Degrees at min rotations  # TUNE
MAX_ANGLE_DEG = 45.0  # Degrees at max rotations  # TUNE

# Gravity feedforward
KG = 0.03  # Duty-cycle feedforward at horizontal  # TUNE

# PID defaults (slot 0 — position)
KP = 0.1  # TUNE
KI = 0.0
KD = 0.01  # TUNE


class HoodSubSystem(Subsystem):
    """
    Hood subsystem — adjustable shooter angle with relative encoder.
    Must be homed against a hard stop before position control is valid.
    Soft limits are set by manually driving to each end and calling
    set_min_limit() / set_max_limit().
    """

    def __init__(self) -> None:
        super().__init__()
        self.motor = rev.SparkMax(CANIds.HOOD, rev.SparkMax.MotorType.kBrushless)
        self.encoder = self.motor.getEncoder()
        self.closed_loop = self.motor.getClosedLoopController()
        self.is_homed: bool = False
        self.limits_set: bool = False

        # Rotation limits — set via set_min_limit / set_max_limit
        self.min_rotations: float = 0.0
        self.max_rotations: float = 5.0  # default until calibrated  # TUNE

        self.motor_config = rev.SparkBaseConfig()
        self.motor_config.smartCurrentLimit(25)
        self.motor_config.secondaryCurrentLimit(30)
        self.motor_config.IdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.motor_config.closedLoop.setFeedbackSensor(
            rev.FeedbackSensor.kPrimaryEncoder
        )
        self.motor_config.closedLoop.P(KP, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.closedLoop.I(KI, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.closedLoop.D(KD, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.closedLoop.outputRange(-1, 1, rev.ClosedLoopSlot.kSlot0)
        self.motor_config.softLimit.forwardSoftLimit(self.max_rotations)
        self.motor_config.softLimit.forwardSoftLimitEnabled(True)
        self.motor_config.softLimit.reverseSoftLimit(self.min_rotations)
        self.motor_config.softLimit.reverseSoftLimitEnabled(True)
        self.motor.configure(
            self.motor_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self._target_position: float = 0.0

        table = ntcore.NetworkTableInstance.getDefault().getTable("Hood")
        self._position_pub = table.getDoubleTopic("Position Turns").publish()
        self._target_pub = table.getDoubleTopic("Target Turns").publish()
        self._velocity_pub = table.getDoubleTopic("Velocity RPM").publish()
        self._amps_pub = table.getDoubleTopic("Amps").publish()
        self._homed_pub = table.getBooleanTopic("Homed").publish()
        self._limits_set_pub = table.getBooleanTopic("Limits Set").publish()

    # ── Position control ───────────────────────────────────────────

    def get_current_position(self) -> float:
        """Return current encoder position in motor turns."""
        return self.encoder.getPosition()

    def set_target_position(self, target_position: float) -> None:
        """Drive hood to a position in motor turns (requires homing)."""
        if not self.is_homed:
            return
        target_position = max(
            self.min_rotations, min(target_position, self.max_rotations)
        )
        self._target_position = target_position
        self.closed_loop.setReference(
            target_position,
            rev.SparkBase.ControlType.kPosition,
            rev.ClosedLoopSlot.kSlot0,
        )

    def set_angle(self, degrees: float) -> None:
        """Drive hood to an angle in degrees with gravity feedforward.

        Requires homing and limits to be set. Clamps to
        [MIN_ANGLE_DEG, MAX_ANGLE_DEG] and linearly maps to rotations.
        """
        if not self.is_homed or not self.limits_set:
            return

        degrees = max(MIN_ANGLE_DEG, min(degrees, MAX_ANGLE_DEG))

        # Linear interpolation: degrees → rotations
        if MAX_ANGLE_DEG == MIN_ANGLE_DEG:
            target_rot = self.min_rotations
        else:
            t = (degrees - MIN_ANGLE_DEG) / (MAX_ANGLE_DEG - MIN_ANGLE_DEG)
            target_rot = self.min_rotations + t * (self.max_rotations - self.min_rotations)

        # Gravity feedforward from current angle
        current_rot = self.encoder.getPosition()
        if self.max_rotations == self.min_rotations:
            current_angle_deg = MIN_ANGLE_DEG
        else:
            t_cur = (current_rot - self.min_rotations) / (self.max_rotations - self.min_rotations)
            current_angle_deg = MIN_ANGLE_DEG + t_cur * (MAX_ANGLE_DEG - MIN_ANGLE_DEG)
        ff = KG * math.cos(math.radians(current_angle_deg))

        self._target_position = target_rot
        self.closed_loop.setReference(
            target_rot,
            rev.SparkBase.ControlType.kPosition,
            rev.ClosedLoopSlot.kSlot0,
            ff,
        )

    def set_target_amps(self, target_amps: float) -> None:
        """Drive hood with raw current control (amps)."""
        self.closed_loop.setReference(
            target_amps, rev.SparkBase.ControlType.kCurrent, rev.ClosedLoopSlot.kSlot0
        )

    # ── Homing helpers ─────────────────────────────────────────────

    def reset_encoder(self) -> None:
        """Zero the encoder and mark homed."""
        self.encoder.setPosition(0)
        self.is_homed = True

    def enable_soft_limits(self) -> None:
        """Enable soft limits after homing — encoder zero is now meaningful."""
        cfg = rev.SparkBaseConfig()
        cfg.softLimit.forwardSoftLimitEnabled(True)
        cfg.softLimit.reverseSoftLimitEnabled(True)
        self.motor.configure(
            cfg,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

    def disable_soft_limits(self) -> None:
        """Disable soft limits for homing — motor needs to move past current zero."""
        cfg = rev.SparkBaseConfig()
        cfg.softLimit.forwardSoftLimitEnabled(False)
        cfg.softLimit.reverseSoftLimitEnabled(False)
        self.motor.configure(
            cfg,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )

    # ── Soft limit calibration ─────────────────────────────────────

    def set_min_limit(self) -> None:
        """Record current position as the reverse (min) soft limit."""
        self.min_rotations = self.encoder.getPosition()
        cfg = rev.SparkBaseConfig()
        cfg.softLimit.reverseSoftLimit(self.min_rotations)
        cfg.softLimit.reverseSoftLimitEnabled(True)
        self.motor.configure(
            cfg,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )
        self.limits_set = self.limits_set or (self.max_rotations != self.min_rotations)
        print(f"[Hood] Min limit set at {self.min_rotations:.3f} turns")

    def set_max_limit(self) -> None:
        """Record current position as the forward (max) soft limit."""
        self.max_rotations = self.encoder.getPosition()
        cfg = rev.SparkBaseConfig()
        cfg.softLimit.forwardSoftLimit(self.max_rotations)
        cfg.softLimit.forwardSoftLimitEnabled(True)
        self.motor.configure(
            cfg,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )
        self.limits_set = self.limits_set or (self.max_rotations != self.min_rotations)
        print(f"[Hood] Max limit set at {self.max_rotations:.3f} turns")

    # ── Default actions ────────────────────────────────────────────

    def stow(self) -> None:
        """Move to stow position if homed, otherwise stop."""
        if self.is_homed:
            self.set_target_position(STOW_POSITION)
        else:
            self.motor.stopMotor()

    def stop(self) -> None:
        """Stop the motor immediately."""
        self._target_position = 0.0
        self.motor.stopMotor()

    # ── Periodic ───────────────────────────────────────────────────

    def periodic(self) -> None:
        self._position_pub.set(self.encoder.getPosition())
        self._target_pub.set(self._target_position)
        self._velocity_pub.set(self.encoder.getVelocity())
        self._amps_pub.set(self.motor.getOutputCurrent())
        self._homed_pub.set(self.is_homed)
        self._limits_set_pub.set(self.limits_set)

    def simulationPeriodic(self) -> None:
        pass
