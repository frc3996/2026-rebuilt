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

# Stall protection during position control
POSITION_STALL_CURRENT = 10.0  # Amps — lower than homing, catches sustained load  # TUNE
POSITION_STALL_VELOCITY = 10.0  # RPM  # TUNE
POSITION_STALL_CYCLES = 10  # ~200ms at 20ms loop  # TUNE

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

# PID defaults (slot 1 — current control)
CURRENT_KP = 0.04  # TUNE
CURRENT_KI = 0.0
CURRENT_KD = 0.0


class HoodSubSystem(Subsystem):
    """
    Hood subsystem — adjustable shooter angle with relative encoder.
    Must be homed against a hard stop before position control is valid.
    Soft limits are set by manually driving to each end and calling
    set_min_limit() / set_max_limit().
    """

    def __init__(self) -> None:
        super().__init__()
        self._motor = rev.SparkMax(CANIds.HOOD, rev.SparkMax.MotorType.kBrushless)
        self._encoder = self._motor.getEncoder()
        self._closed_loop = self._motor.getClosedLoopController()
        self.is_homed: bool = False
        self.limits_set: bool = False
        self._soft_limits_enabled: bool = True

        # Rotation limits — set via set_min_limit / set_max_limit
        self.min_rotations: float = 0.0
        self.max_rotations: float = 5.0  # default until calibrated  # TUNE

        self._config = rev.SparkBaseConfig()
        self._config.voltageCompensation(12.0)
        self._config.smartCurrentLimit(25)
        self._config.secondaryCurrentLimit(30)
        self._config.IdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self._config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        self._config.closedLoop.P(KP, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.I(KI, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.D(KD, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.outputRange(-1, 1, rev.ClosedLoopSlot.kSlot0)
        self._config.closedLoop.P(CURRENT_KP, rev.ClosedLoopSlot.kSlot1)
        self._config.closedLoop.I(CURRENT_KI, rev.ClosedLoopSlot.kSlot1)
        self._config.closedLoop.D(CURRENT_KD, rev.ClosedLoopSlot.kSlot1)
        self._config.softLimit.forwardSoftLimit(self.max_rotations)
        self._config.softLimit.forwardSoftLimitEnabled(True)
        self._config.softLimit.reverseSoftLimit(self.min_rotations)
        self._config.softLimit.reverseSoftLimitEnabled(True)
        self._motor.configure(
            self._config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self._target_position: float = 0.0
        self._position_active: bool = False
        self._stall_counter: int = 0
        self._stall_count: int = 0

        table = ntcore.NetworkTableInstance.getDefault().getTable("Hood")
        self._position_pub = table.getDoubleTopic("Position Turns").publish()
        self._target_pub = table.getDoubleTopic("Target Turns").publish()
        self._velocity_pub = table.getDoubleTopic("Velocity RPM").publish()
        self._amps_pub = table.getDoubleTopic("Amps").publish()
        self._homed_pub = table.getBooleanTopic("Homed").publish()
        self._stall_count_pub = table.getIntegerTopic("Stall Count").publish()
        self._limits_set_pub = table.getBooleanTopic("Limits Set").publish()

    # ── Low-level motor access (for homing / auto-tune commands) ──

    def set_voltage(self, volts: float) -> None:
        self._motor.setVoltage(volts)

    def set_duty_cycle(self, output: float) -> None:
        self._motor.set(output)

    def get_output_current(self) -> float:
        return self._motor.getOutputCurrent()

    def get_velocity(self) -> float:
        return self._encoder.getVelocity()

    def set_pid_gains(self, kp: float, ki: float, kd: float) -> None:
        """Update position PID gains (slot 0) at runtime."""
        self._closed_loop.setP(kp, rev.ClosedLoopSlot.kSlot0)
        self._closed_loop.setI(ki, rev.ClosedLoopSlot.kSlot0)
        self._closed_loop.setD(kd, rev.ClosedLoopSlot.kSlot0)

    # ── Position control ───────────────────────────────────────────

    def get_current_position(self) -> float:
        return self._encoder.getPosition()

    def set_target_position(self, target_position: float) -> None:
        """Drive hood to a position in motor turns (requires homing)."""
        if not self.is_homed:
            return
        target_position = max(self.min_rotations, min(target_position, self.max_rotations))
        self._target_position = target_position
        self._position_active = True
        self._closed_loop.setReference(
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
        current_rot = self._encoder.getPosition()
        if self.max_rotations == self.min_rotations:
            current_angle_deg = MIN_ANGLE_DEG
        else:
            t_cur = (current_rot - self.min_rotations) / (self.max_rotations - self.min_rotations)
            current_angle_deg = MIN_ANGLE_DEG + t_cur * (MAX_ANGLE_DEG - MIN_ANGLE_DEG)
        ff = KG * math.cos(math.radians(current_angle_deg))

        self._target_position = target_rot
        self._position_active = True
        self._closed_loop.setReference(
            target_rot,
            rev.SparkBase.ControlType.kPosition,
            rev.ClosedLoopSlot.kSlot0,
            ff,
        )

    def set_target_amps(self, target_amps: float) -> None:
        """Drive hood with current control (slot 1)."""
        self._closed_loop.setReference(target_amps, rev.SparkBase.ControlType.kCurrent, rev.ClosedLoopSlot.kSlot1)

    # ── Homing helpers ─────────────────────────────────────────────

    def reset_encoder(self) -> None:
        """Zero the encoder and mark homed."""
        self._encoder.setPosition(0)
        self.is_homed = True

    def _apply_soft_limit_config(self) -> None:
        """Push current soft limit state to the motor controller."""
        self._config.softLimit.forwardSoftLimit(self.max_rotations)
        self._config.softLimit.forwardSoftLimitEnabled(self._soft_limits_enabled)
        self._config.softLimit.reverseSoftLimit(self.min_rotations)
        self._config.softLimit.reverseSoftLimitEnabled(self._soft_limits_enabled)
        self._motor.configure(
            self._config,
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
        self.min_rotations = self._encoder.getPosition()
        self._soft_limits_enabled = True
        self._apply_soft_limit_config()
        self.limits_set = self.limits_set or (self.max_rotations != self.min_rotations)

    def set_max_limit(self) -> None:
        """Record current position as the forward (max) soft limit."""
        self.max_rotations = self._encoder.getPosition()
        self._soft_limits_enabled = True
        self._apply_soft_limit_config()
        self.limits_set = self.limits_set or (self.max_rotations != self.min_rotations)

    # ── Default actions ────────────────────────────────────────────

    def stow(self) -> None:
        """Move to stow position if homed, otherwise stop."""
        if self.is_homed:
            self.set_target_position(STOW_POSITION)
        else:
            self._motor.stopMotor()

    def stop(self) -> None:
        self._target_position = 0.0
        self._position_active = False
        self._motor.stopMotor()

    # ── Periodic ───────────────────────────────────────────────────

    def periodic(self) -> None:
        current = self._motor.getOutputCurrent()
        velocity = abs(self._encoder.getVelocity())

        # Stall detection during position control
        if self._position_active:
            if current > POSITION_STALL_CURRENT and velocity < POSITION_STALL_VELOCITY:
                self._stall_counter += 1
            else:
                self._stall_counter = 0
            if self._stall_counter >= POSITION_STALL_CYCLES:
                self._stall_count += 1
                self._stall_counter = 0
                self._position_active = False
                self._motor.stopMotor()

        self._position_pub.set(self._encoder.getPosition())
        self._target_pub.set(self._target_position)
        self._velocity_pub.set(self._encoder.getVelocity())
        self._amps_pub.set(current)
        self._homed_pub.set(self.is_homed)
        self._stall_count_pub.set(self._stall_count)
        self._limits_set_pub.set(self.limits_set)

    def simulationPeriodic(self) -> None:
        pass
