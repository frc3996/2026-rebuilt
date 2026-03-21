from typing import ClassVar

import ntcore
from commands2 import Command

from constants import HUB_POSITION


class ShootAtHub(Command):
    """
    Command that coordinates shooter, kicker, conveyor, and hood to shoot at the hub.

    Each loop: recalculates distance and ballistics, stages motors
    (shooter → kicker → conveyor), and adjusts hood position.
    """

    SHOOTER_TARGET_RPM = 4000
    SHOOTER_TOLERANCE_RPM = 200
    KICKER_TOLERANCE_RPM = 500

    # Lookup table: (distance_meters, hood_motor_turns, shooter_rpm)
    SHOT_TABLE: ClassVar[list[tuple[float, float, int]]] = [
        (1.0, 1.0, 3500),  # Close shot
        (2.0, 2.0, 3800),  # Medium shot
        (3.0, 3.0, 4200),  # Far shot
        (4.0, 3.5, 4500),  # Very far shot
        (5.0, 4.0, 4800),  # Max range shot
    ]

    def __init__(self, shooter, kicker, indexer, hood, vision, drivetrain):
        super().__init__()
        self.shooter = shooter
        self.kicker = kicker
        self.indexer = indexer
        self.hood = hood
        self.vision = vision
        self.drivetrain = drivetrain

        self.addRequirements(shooter, kicker, indexer, hood)
        # vision and drivetrain are intentionally NOT requirements — the parallel
        # snap-angle command in robotcontainer owns the drivetrain while this runs.

        self.target_rpm = self.SHOOTER_TARGET_RPM
        self.target_hood_turns = 0.0
        self.feeding = False

        table = ntcore.NetworkTableInstance.getDefault().getTable("Shoot")
        self._distance_pub = table.getDoubleTopic("Distance To Hub").publish()
        self._target_rpm_pub = table.getDoubleTopic("Target RPM").publish()
        self._current_rpm_pub = table.getDoubleTopic("Current RPM").publish()
        self._rpm_error_pub = table.getDoubleTopic("RPM Error").publish()
        self._target_hood_pub = table.getDoubleTopic("Target Hood Turns").publish()
        self._feeding_pub = table.getBooleanTopic("Feeding").publish()

    def initialize(self):
        self.feeding = False

    def execute(self):
        distance = self.calculate_distance_to_hub()
        self.target_rpm, self.target_hood_turns = self.compute_ballistics(distance)

        self.shooter.set_target_speed(self.target_rpm)
        self.hood.set_target_position(self.target_hood_turns)

        current_rpm = self.shooter.get_current_speed()
        shooter_ready = abs(current_rpm - self.target_rpm) < self.SHOOTER_TOLERANCE_RPM

        # Stage 1: kicker spins up once shooter is at speed
        if shooter_ready:
            self.kicker.set_target_speed(self.target_rpm)
        else:
            self.kicker.stop()

        # Stage 2: conveyor feeds once kicker is at speed
        kicker_ready = abs(self.kicker.get_current_speed() - self.target_rpm) < self.KICKER_TOLERANCE_RPM
        self.feeding = shooter_ready and kicker_ready
        if self.feeding:
            self.indexer.set_target_output(0.5)
        else:
            self.indexer.stop()

        self._distance_pub.set(distance)
        self._target_rpm_pub.set(self.target_rpm)
        self._current_rpm_pub.set(current_rpm)
        self._rpm_error_pub.set(abs(current_rpm - self.target_rpm))
        self._target_hood_pub.set(self.target_hood_turns)
        self._feeding_pub.set(self.feeding)

    def end(self, interrupted: bool):
        self.shooter.stop()
        self.kicker.stop()
        self.indexer.stop()
        self.hood.stow()
        self._feeding_pub.set(False)

    def isFinished(self) -> bool:
        return False

    def calculate_distance_to_hub(self) -> float:
        robot_pose = self.drivetrain.get_state().pose
        return robot_pose.translation().distance(HUB_POSITION)

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
