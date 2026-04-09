import ntcore
import rev
from commands2 import Subsystem

from constants import DEBUG_NT, NEO_FREE_SPEED_RPM, CANIds

# Arm positions — absolute encoder reads 0‥1 turns
DEPLOY_POSITION = 0.81  # Fully deployed  # TUNE
STOW_POSITION = 0.525  # Retracted/stowed  # TUNE


# Position tolerance — "at target" threshold (absolute encoder turns)
POSITION_TOLERANCE = 0.02  # TUNE

# PID defaults (slot 0 — position, feedback from absolute encoder)
ARM_KP = 4.8
ARM_KI = 0
ARM_KD = 0


class IntakeSubSystem(Subsystem):
    """
    Intake subsystem — deployable arm with roller for game piece pickup.
    Uses an absolute encoder on the SparkMax data port for PID position
    control.  Two positions: DEPLOY_POSITION and STOW_POSITION.
    No homing required.
    """

    def __init__(self) -> None:
        super().__init__()
        # ── Arm motor + absolute encoder ───────────────────────────
        self._arm_motor = rev.SparkMax(
            CANIds.INTAKE_ARM, rev.SparkMax.MotorType.kBrushless
        )
        self._arm_encoder = self._arm_motor.getAbsoluteEncoder()
        self._arm_closed_loop = self._arm_motor.getClosedLoopController()

        # ── Roller motor ───────────────────────────────────────────
        self._roller_motor = rev.SparkMax(
            CANIds.INTAKE_ROLLER, rev.SparkMax.MotorType.kBrushless
        )
        self._roller_encoder = self._roller_motor.getEncoder()
        self._roller_closed_loop = self._roller_motor.getClosedLoopController()

        # ── Arm config ─────────────────────────────────────────────
        self._arm_config = rev.SparkMaxConfig()
        self._arm_config.inverted(True)
        self._arm_config.voltageCompensation(10)
        self._arm_config.smartCurrentLimit(10)
        self._arm_config.secondaryCurrentLimit(15)
        self._arm_config.IdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        # Absolute encoder on the data port
        self._arm_config.absoluteEncoder.setSparkMaxDataPortConfig()
        # PID feedback from absolute encoder
        self._arm_config.closedLoop.setFeedbackSensor(
            rev.FeedbackSensor.kAbsoluteEncoder
        )
        self._arm_config.closedLoop.P(ARM_KP, rev.ClosedLoopSlot.kSlot0)
        self._arm_config.closedLoop.I(ARM_KI, rev.ClosedLoopSlot.kSlot0)
        self._arm_config.closedLoop.D(ARM_KD, rev.ClosedLoopSlot.kSlot0)
        self._arm_config.closedLoop.outputRange(-1, 1, rev.ClosedLoopSlot.kSlot0)
        self._arm_motor.configure(
            self._arm_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # ── Roller config ─────────────────────────────────────────
        roller_config = rev.SparkBaseConfig()
        roller_config.voltageCompensation(10.5)
        roller_config.smartCurrentLimit(30)
        roller_config.secondaryCurrentLimit(40)
        roller_config.IdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
        roller_config.closedLoop.setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder)
        roller_config.closedLoop.P(0.0003, rev.ClosedLoopSlot.kSlot0)
        roller_config.closedLoop.I(0, rev.ClosedLoopSlot.kSlot0)
        roller_config.closedLoop.D(0, rev.ClosedLoopSlot.kSlot0)
        roller_config.closedLoop.velocityFF(
            1.0 / NEO_FREE_SPEED_RPM, rev.ClosedLoopSlot.kSlot0
        )
        roller_config.closedLoop.outputRange(0, 1, rev.ClosedLoopSlot.kSlot0)
        self._roller_motor.configure(
            roller_config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # ── State ─────────────────────────────────────────────────
        self._arm_target: float = 0.0
        self._roller_target: float = 0.0

        # ── NetworkTables telemetry ────────────────────────────────
        table = ntcore.NetworkTableInstance.getDefault().getTable("Intake")
        self._arm_amps_pub = table.getDoubleTopic("Arm Amps").publish()
        self._arm_position_pub = table.getDoubleTopic("Arm Position Turns").publish()
        self._arm_target_pub = table.getDoubleTopic("Arm Target Turns").publish()
        self._arm_velocity_pub = table.getDoubleTopic("Arm Velocity RPM").publish()
        self._roller_velocity_pub = table.getDoubleTopic(
            "Roller Velocity RPM"
        ).publish()
        self._roller_target_pub = table.getDoubleTopic("Roller Target RPM").publish()
        self._roller_amps_pub = table.getDoubleTopic("Roller Amps").publish()

        # ── NT-tunable PID gains ───────────────────────────────────
        # Publish defaults so they appear in the dashboard immediately
        table.getDoubleTopic("Arm kP").publish().set(ARM_KP)
        table.getDoubleTopic("Arm kI").publish().set(ARM_KI)
        table.getDoubleTopic("Arm kD").publish().set(ARM_KD)
        self._arm_kp_sub = table.getDoubleTopic("Arm kP").subscribe(ARM_KP)
        self._arm_ki_sub = table.getDoubleTopic("Arm kI").subscribe(ARM_KI)
        self._arm_kd_sub = table.getDoubleTopic("Arm kD").subscribe(ARM_KD)
        # Track last-applied values so we only flash the controller on change
        self._last_kp = ARM_KP
        self._last_ki = ARM_KI
        self._last_kd = ARM_KD

    # ── Low-level arm motor access (for auto-tune) ─────────────────

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

    def set_arm_pid_gains(self, kp: float, ki: float, kd: float) -> None:
        """Update arm position PID gains (slot 0) at runtime."""
        self._arm_config.closedLoop.P(kp, rev.ClosedLoopSlot.kSlot0)
        self._arm_config.closedLoop.I(ki, rev.ClosedLoopSlot.kSlot0)
        self._arm_config.closedLoop.D(kd, rev.ClosedLoopSlot.kSlot0)
        self._arm_motor.configure(
            self._arm_config,
            rev.ResetMode.kNoResetSafeParameters,
            rev.PersistMode.kNoPersistParameters,
        )
        self._last_kp = kp
        self._last_ki = ki
        self._last_kd = kd

    def update_pid_from_nt(self) -> None:
        """Read PID gains from NetworkTables and apply if changed.

        Call this from disabledPeriodic so gains are flashed while the
        robot is safe to reconfigure.
        """
        kp = self._arm_kp_sub.get()
        ki = self._arm_ki_sub.get()
        kd = self._arm_kd_sub.get()
        if kp != self._last_kp or ki != self._last_ki or kd != self._last_kd:
            self.set_arm_pid_gains(kp, ki, kd)

    # ── Arm position control ───────────────────────────────────────

    def set_arm_target_position(self, target_position: float) -> None:
        """Command arm to a position (absolute encoder turns) via on-controller PID."""
        self._arm_target = target_position
        self._arm_closed_loop.setReference(
            target_position,
            rev.SparkBase.ControlType.kPosition,
            rev.ClosedLoopSlot.kSlot0,
        )

    def deploy(self) -> None:
        """Move arm to DEPLOY_POSITION."""
        self.set_arm_target_position(DEPLOY_POSITION)

    def stow(self) -> None:
        """Move arm to STOW_POSITION and stop roller."""
        self.set_arm_target_position(STOW_POSITION)
        self.set_roller_target_speed(0)

    def is_at_position(self, target: float) -> bool:
        """True when arm is within POSITION_TOLERANCE of *target*."""
        return abs(self.get_arm_position() - target) < POSITION_TOLERANCE

    @property
    def is_deployed(self) -> bool:
        return self.is_at_position(DEPLOY_POSITION)

    @property
    def is_stowed(self) -> bool:
        return self.is_at_position(STOW_POSITION)

    # ── Default actions ────────────────────────────────────────────

    def hold(self) -> None:
        """Stop roller. Arm holds via brake mode."""
        self._roller_motor.stopMotor()

    # ── Roller control ─────────────────────────────────────────────

    def set_roller_duty_cycle(self, output: float) -> None:
        self._roller_motor.set(output)

    def set_roller_target_speed(self, target_velocity: float) -> None:
        self._roller_target = target_velocity
        self._roller_closed_loop.setReference(
            target_velocity,
            rev.SparkBase.ControlType.kVelocity,
            rev.ClosedLoopSlot.kSlot0,
        )

    # ── Periodic ───────────────────────────────────────────────────

    def periodic(self) -> None:
        if not DEBUG_NT:
            return
        self._arm_amps_pub.set(self._arm_motor.getOutputCurrent())
        self._arm_position_pub.set(self._arm_encoder.getPosition())
        self._arm_target_pub.set(self._arm_target)
        self._arm_velocity_pub.set(self._arm_encoder.getVelocity())
        self._roller_velocity_pub.set(self._roller_encoder.getVelocity())
        self._roller_target_pub.set(self._roller_target)
        self._roller_amps_pub.set(self._roller_motor.getOutputCurrent())

    def simulationPeriodic(self) -> None:
        pass
