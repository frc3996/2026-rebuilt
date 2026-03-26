from typing import ClassVar

import ntcore
from commands2 import Command
from wpilib import DriverStation
from wpimath.geometry import Translation2d

# Field dimensions: 651.22 × 317.69 inches
FIELD_LENGTH_M = 651.22 * 0.0254  # 16.541 m
HUB_X_M = 182.11 * 0.0254  # 4.626 m from alliance wall
HUB_Y_M = 158.84 * 0.0254  # 4.035 m — centered widthwise

BLUE_HUB = Translation2d(HUB_X_M, HUB_Y_M)
RED_HUB = Translation2d(FIELD_LENGTH_M - HUB_X_M, HUB_Y_M)


class HubShot(Command):
    """
    Command that coordinates shooter, kicker, conveyor, and hood to shoot at the hub.

    Each loop: recalculates distance and ballistics, stages motors
    (shooter → kicker → conveyor), and adjusts hood position.
    """

    SHOOTER_TOLERANCE_RPM = 75
    KICKER_TOLERANCE_RPM = 150

    # Lookup table: (distance_meters, hood_motor_turns, shooter_rpm)
    SHOT_TABLE: ClassVar[list[tuple[float, float, int]]] = [
        (1.94, 0.1, 2040),
        (1.96, 0.1, 2080),
        (2.48, 0.1, 2160),
        (3.25, 0.1, 2360),
        (3.67, 0.1, 2420),
    ]

    def __init__(self, shooter, kicker, indexer, hood, drivetrain):
        super().__init__()
        self.shooter = shooter
        self.kicker = kicker
        self.indexer = indexer
        self.hood = hood
        self.drivetrain = drivetrain

        self.addRequirements(shooter, kicker, indexer, hood)

        self.target_rpm = 0.0
        self.target_hood_turns = 0.0
        # 0 = spinning up shooter, 1 = spinning up kicker, 2 = feeding
        self._stage = 0

        table = ntcore.NetworkTableInstance.getDefault().getTable("Shoot")
        self._distance_pub = table.getDoubleTopic("Distance To Hub").publish()
        self._target_rpm_pub = table.getDoubleTopic("Target RPM").publish()
        self._current_rpm_pub = table.getDoubleTopic("Current RPM").publish()
        self._rpm_error_pub = table.getDoubleTopic("RPM Error").publish()
        self._target_hood_pub = table.getDoubleTopic("Target Hood Turns").publish()
        self._feeding_pub = table.getBooleanTopic("Feeding").publish()

    def initialize(self):
        self._stage = 0

    def execute(self):
        distance = self._get_hub_distance()
        self.target_rpm, self.target_hood_turns = self.compute_ballistics(distance)

        self.shooter.set_target_speed(self.target_rpm)
        self.hood.set_target_position(self.target_hood_turns)

        current_rpm = self.shooter.get_current_speed()
        shooter_ready = (
            current_rpm > self.target_rpm * 0.9
            and abs(current_rpm - self.target_rpm) < self.SHOOTER_TOLERANCE_RPM
        )

        # Stage 0 → 1: shooter at speed, start kicker
        if self._stage == 0 and shooter_ready:
            self._stage = 1

        # Stage 1 → 2: kicker at speed, start feeding
        if self._stage >= 1:
            self.kicker.set_target_speed(self.target_rpm)
            kicker_speed = self.kicker.get_current_speed()
            kicker_ready = abs(kicker_speed - self.target_rpm) < self.KICKER_TOLERANCE_RPM
            if self._stage == 1 and kicker_ready:
                self._stage = 2
        else:
            self.kicker.stop()

        if self._stage >= 2:
            self.indexer.set_target_output(1.0)
        else:
            self.indexer.stop()

        self._distance_pub.set(distance)
        self._target_rpm_pub.set(self.target_rpm)
        self._current_rpm_pub.set(current_rpm)
        self._rpm_error_pub.set(abs(current_rpm - self.target_rpm))
        self._target_hood_pub.set(self.target_hood_turns)
        self._feeding_pub.set(self._stage >= 2)

    def end(self, interrupted: bool):
        self.shooter.stop()
        self.kicker.stop()
        self.indexer.stop()
        self.hood.stow()
        self._feeding_pub.set(False)
        self._stage = 0

    def isFinished(self) -> bool:
        return False

    def _get_hub_distance(self) -> float:
        robot_pos = self.drivetrain.get_state().pose.translation()
        alliance = DriverStation.getAlliance()
        hub = RED_HUB if alliance == DriverStation.Alliance.kRed else BLUE_HUB
        return robot_pos.distance(hub)

    def compute_ballistics(self, distance: float) -> tuple[float, float]:
        if distance <= self.SHOT_TABLE[0][0]:
            return self.SHOT_TABLE[0][2], self.SHOT_TABLE[0][1]

        if distance >= self.SHOT_TABLE[-1][0]:
            return self.SHOT_TABLE[-1][2], self.SHOT_TABLE[-1][1]

        for i in range(len(self.SHOT_TABLE) - 1):
            dist1, turns1, rpm1 = self.SHOT_TABLE[i]
            dist2, turns2, rpm2 = self.SHOT_TABLE[i + 1]

            if dist1 <= distance <= dist2:
                t = (distance - dist1) / (dist2 - dist1)
                hood_turns = turns1 + t * (turns2 - turns1)
                shooter_rpm = rpm1 + t * (rpm2 - rpm1)
                return shooter_rpm, hood_turns

        return self.SHOT_TABLE[-1][2], self.SHOT_TABLE[-1][1]
