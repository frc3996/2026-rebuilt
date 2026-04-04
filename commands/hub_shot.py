import math
import ntcore
from commands2 import Command, cmd
from ntcore import NetworkTableInstance
from pathplannerlib.controller import PPHolonomicDriveController
from wpilib import DriverStation, Timer
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

# Field dimensions: 651.22 × 317.69 inches
FIELD_LENGTH_M = 651.22 * 0.0254  # 16.541 m
HUB_X_M = 182.11 * 0.0254  # 4.626 m from alliance wall
HUB_Y_M = 158.84 * 0.0254  # 4.035 m — centered widthwise

BLUE_HUB = Translation2d(HUB_X_M, HUB_Y_M)
RED_HUB = Translation2d(FIELD_LENGTH_M - HUB_X_M, HUB_Y_M)

# Flywheel speed compensation constants
_FLYWHEEL_DIAMETER_M = 4 * 0.0254  # 4 inches
_FLYWHEEL_CIRCUMFERENCE = math.pi * _FLYWHEEL_DIAMETER_M
_GEAR_RATIO = 1.3  # motor:flywheel
_SLIP_FACTOR = 0.1  # TUNE via NT "Shoot/Slip Factor"
_LOOKAHEAD_S = 0.05  # 50ms pose prediction to compensate for control loop latency
_ITERATIONS = 2  # converge virtual distance ↔ RPM


def _linreg(xs: list[float], ys: list[float]) -> tuple[float, float]:
    """Least-squares linear regression. Returns (slope, intercept)."""
    n = len(xs)
    sx = sum(xs)
    sy = sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    slope = (n * sxy - sx * sy) / (n * sxx - sx * sx)
    intercept = (sy - slope * sx) / n
    return slope, intercept


def _rpm_to_exit_velocity(rpm: float, slip: float) -> float:
    return rpm * _GEAR_RATIO * _FLYWHEEL_CIRCUMFERENCE / 60.0 * slip


# ── Ballistics (linear regression from calibration data) ──────────────

# Calibration data: (distance_meters, hood_motor_turns, shooter_rpm)
SHOT_TABLE: list[tuple[float, float, int]] = [
    (1.74, 0.1, 2120),
    (2.70, 0.1, 2300),
    (3.72, 0.5, 2660),
    (4.48, 0.5, 2920),
]

_dists = [d for d, _, _ in SHOT_TABLE]
_RPM_SLOPE, _RPM_INTERCEPT = _linreg(_dists, [r for _, _, r in SHOT_TABLE])
_HOOD_SLOPE, _HOOD_INTERCEPT = _linreg(_dists, [h for _, h, _ in SHOT_TABLE])


def compute_ballistics(distance: float) -> tuple[float, float]:
    """Returns (shooter_rpm, hood_turns) for a given distance."""
    return _RPM_SLOPE * distance + _RPM_INTERCEPT, _HOOD_SLOPE * distance + _HOOD_INTERCEPT


# ── VirtualGoal ───────────────────────────────────────────────────────


class VirtualGoal:
    """Computes a virtual aiming point that compensates for robot motion during ball flight.

    Iterates ballistics ↔ virtual distance to break the circular dependency:
      distance → RPM → exit velocity → flight time → virtual distance → new RPM ...
    Converges in 2 iterations since the correction is small relative to raw distance.
    """

    def __init__(self, drivetrain):
        self._drivetrain = drivetrain

        # NT-tunable slip factor
        shoot_table = NetworkTableInstance.getDefault().getTable("Shoot")
        self._slip_factor_pub = shoot_table.getDoubleTopic("Slip Factor").publish()
        self._slip_factor_pub.set(_SLIP_FACTOR)
        self._slip_factor_sub = shoot_table.getDoubleTopic("Slip Factor").subscribe(_SLIP_FACTOR)

        # Last computed results — read by HubShot
        self.last_virtual_distance = 0.0
        self.last_raw_distance = 0.0
        self.last_rpm = 0.0
        self.last_hood_turns = 0.0

        # Telemetry
        vg_table = NetworkTableInstance.getDefault().getTable("VirtualGoal")
        self._vg_dist_pub = vg_table.getDoubleTopic("Virtual Distance").publish()
        self._raw_dist_pub = vg_table.getDoubleTopic("Raw Distance").publish()
        self._flight_time_pub = vg_table.getDoubleTopic("Flight Time").publish()
        self._vg_pose_pub = vg_table.getStructTopic("Pose", Pose2d).publish()

    @staticmethod
    def _get_hub():
        return (
            RED_HUB
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed
            else BLUE_HUB
        )

    def calculate(self) -> tuple[Rotation2d, float]:
        """Iterate ballistics ↔ virtual distance to convergence.

        Returns (aim_direction in operator perspective, angular_rate_feedforward).
        Updates last_virtual_distance, last_rpm, last_hood_turns.
        """
        hub = self._get_hub()
        state = self._drivetrain.get_state()
        speeds = state.speeds
        heading = state.pose.rotation()

        # Convert robot-relative speeds to field-relative
        cos_h, sin_h = heading.cos(), heading.sin()
        field_vx = speeds.vx * cos_h - speeds.vy * sin_h
        field_vy = speeds.vx * sin_h + speeds.vy * cos_h

        # Predict robot pose forward to compensate for control loop latency
        robot = state.pose.translation()
        pred_x = robot.X() + field_vx * _LOOKAHEAD_S
        pred_y = robot.Y() + field_vy * _LOOKAHEAD_S

        dx = hub.X() - pred_x
        dy = hub.Y() - pred_y
        raw_distance = math.hypot(dx, dy)

        self.last_raw_distance = raw_distance
        self._raw_dist_pub.set(raw_distance)

        is_red = DriverStation.getAlliance() == DriverStation.Alliance.kRed
        _flip = Rotation2d(math.pi)

        if raw_distance < 0.5:
            aim = Rotation2d(math.atan2(dy, dx))
            if is_red:
                aim = aim.rotateBy(_flip)
            rpm, hood = compute_ballistics(raw_distance)
            self._store(raw_distance, rpm, hood, 0.0)
            self._vg_pose_pub.set(Pose2d(hub.X(), hub.Y(), Rotation2d()))
            return aim, 0.0

        slip = self._slip_factor_sub.get()

        # Iterate: seed with raw distance, converge virtual distance ↔ RPM
        distance = raw_distance
        flight_time = 0.0
        for _ in range(_ITERATIONS):
            rpm, hood = compute_ballistics(distance)
            exit_velocity = _rpm_to_exit_velocity(rpm, slip)
            flight_time = raw_distance / exit_velocity

            vg_x = hub.X() - field_vx * flight_time
            vg_y = hub.Y() - field_vy * flight_time
            distance = math.hypot(vg_x - pred_x, vg_y - pred_y)

        vdx = vg_x - pred_x
        vdy = vg_y - pred_y
        aim = Rotation2d(math.atan2(vdy, vdx))
        if is_red:
            aim = aim.rotateBy(_flip)

        # Angular rate feedforward: d/dt atan2(dy, dx) from field-relative motion
        dist_sq = vdx * vdx + vdy * vdy
        ff = (vdx * field_vy - vdy * field_vx) / dist_sq

        self._store(distance, rpm, hood, flight_time)
        self._vg_pose_pub.set(Pose2d(vg_x, vg_y, aim))
        return aim, ff

    def _store(self, vdist, rpm, hood, flight_time):
        self.last_virtual_distance = vdist
        self.last_rpm = rpm
        self.last_hood_turns = hood
        self._vg_dist_pub.set(vdist)
        self._flight_time_pub.set(flight_time)


# ── HubShot Command ──────────────────────────────────────────────────


class HubShot(Command):
    """
    Command that coordinates shooter, kicker, conveyor, and hood to shoot at the hub.

    Reads converged RPM, hood angle, and virtual distance from VirtualGoal each cycle.
    Stages motors (shooter → kicker → conveyor) and adjusts hood position.
    """

    def __init__(self, shooter, kicker, indexer, hood, virtual_goal: VirtualGoal):
        super().__init__()
        self.shooter = shooter
        self.kicker = kicker
        self.indexer = indexer
        self.hood = hood
        self._virtual_goal = virtual_goal

        self.addRequirements(shooter, kicker, indexer, hood)

        self._feed_timer = Timer()

        table = ntcore.NetworkTableInstance.getDefault().getTable("Shoot")
        self._distance_pub = table.getDoubleTopic("Distance To Hub").publish()
        self._target_rpm_pub = table.getDoubleTopic("Target RPM").publish()
        self._current_rpm_pub = table.getDoubleTopic("Current RPM").publish()
        self._rpm_error_pub = table.getDoubleTopic("RPM Error").publish()
        self._target_hood_pub = table.getDoubleTopic("Target Hood Turns").publish()
        self._feeding_pub = table.getBooleanTopic("Feeding").publish()
        self._kicker_full_pub = table.getBooleanTopic("Kicker Full Speed").publish()
        self._kicker_full_pub.set(True)
        self._kicker_full_sub = table.getBooleanTopic("Kicker Full Speed").subscribe(False)

    FEED_DELAY_S = 2

    def initialize(self):
        self._feed_timer.restart()
        PPHolonomicDriveController.setRotationTargetOverride(self._aim_override)

    def _aim_override(self):
        """Rotation target override for PathPlanner — aims at virtual goal."""
        aim, _ = self._virtual_goal.calculate()
        return aim

    def execute(self):
        vg = self._virtual_goal
        target_rpm = vg.last_rpm
        target_hood = vg.last_hood_turns

        self.shooter.set_target_speed(target_rpm)
        self.hood.set_target_position(target_hood)
        if self._kicker_full_sub.get():
            self.kicker.set_duty_cycle(1.0)
        else:
            self.kicker.set_target_speed(target_rpm)

        if self._feed_timer.hasElapsed(self.FEED_DELAY_S):
            self.indexer.set_target_output(1.0)
        else:
            self.indexer.stop()

        current_rpm = self.shooter.get_current_speed()
        feeding = self._feed_timer.hasElapsed(self.FEED_DELAY_S)
        self._distance_pub.set(vg.last_virtual_distance)
        self._target_rpm_pub.set(target_rpm)
        self._current_rpm_pub.set(current_rpm)
        self._rpm_error_pub.set(abs(current_rpm - target_rpm))
        self._target_hood_pub.set(target_hood)
        self._feeding_pub.set(feeding)

    def end(self, interrupted: bool):
        PPHolonomicDriveController.setRotationTargetOverride(None)
        self.shooter.stop()
        self.kicker.stop()
        self.indexer.stop()
        self.hood.stow()
        self._feeding_pub.set(False)

    def isFinished(self) -> bool:
        return False
