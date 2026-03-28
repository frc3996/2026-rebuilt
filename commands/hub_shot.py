from typing import ClassVar

import ntcore
from commands2 import Command
from wpilib import DriverStation, Timer
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

    # Lookup table: (distance_meters, hood_motor_turns, shooter_rpm)
    SHOT_TABLE: ClassVar[list[tuple[float, float, int]]] = [
        (1.74, 0.1, 2120),
        (2.70, 0.1, 2300),
        (3.72, 0.1, 2660),
        (4.48, 0.1, 2920)
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
        self.virtual_distance = 0.0  # set by robotcontainer each cycle
        self._feed_timer = Timer()

        table = ntcore.NetworkTableInstance.getDefault().getTable("Shoot")
        self._distance_pub = table.getDoubleTopic("Distance To Hub").publish()
        self._target_rpm_pub = table.getDoubleTopic("Target RPM").publish()
        self._current_rpm_pub = table.getDoubleTopic("Current RPM").publish()
        self._rpm_error_pub = table.getDoubleTopic("RPM Error").publish()
        self._target_hood_pub = table.getDoubleTopic("Target Hood Turns").publish()
        self._feeding_pub = table.getBooleanTopic("Feeding").publish()

    FEED_DELAY_S = 4.0

    def initialize(self):
        self._feed_timer.restart()

    def execute(self):
        distance = self.virtual_distance if self.virtual_distance > 0 else self._get_hub_distance()
        self.target_rpm, self.target_hood_turns = self.compute_ballistics(distance)

        self.shooter.set_target_speed(self.target_rpm)
        self.hood.set_target_position(self.target_hood_turns)
        self.kicker.set_target_speed(self.target_rpm)

        if self._feed_timer.hasElapsed(self.FEED_DELAY_S):
            self.indexer.set_target_output(1.0)
        else:
            self.indexer.stop()

        self._distance_pub.set(distance)
        current_rpm = self.shooter.get_current_speed()
        feeding = self._feed_timer.hasElapsed(self.FEED_DELAY_S)
        self._distance_pub.set(distance)
        self._target_rpm_pub.set(self.target_rpm)
        self._current_rpm_pub.set(current_rpm)
        self._rpm_error_pub.set(abs(current_rpm - self.target_rpm))
        self._target_hood_pub.set(self.target_hood_turns)
        self._feeding_pub.set(feeding)

    def end(self, interrupted: bool):
        self.shooter.stop()
        self.kicker.stop()
        self.indexer.stop()
        self.hood.stow()
        self._feeding_pub.set(False)

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
