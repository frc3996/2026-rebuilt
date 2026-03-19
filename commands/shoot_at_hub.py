from commands2 import Command
from wpimath.geometry import Pose2d, Translation2d
import ntcore
import math

class ShootAtHub(Command):
    """
    Command that coordinates shooter, indexer, hood, and vision to shoot at the hub.

    This command will:
    1. Spin up the shooter to target RPM
    2. Calculate distance to hub using vision/odometry
    3. Compute ballistics and adjust hood angle
    4. When shooter is ready (±200 RPM), feed ball with indexer
    """

    HUB_POSITION = Translation2d(8.23, 4.115)  # meters (center of field)

    SHOOTER_TARGET_RPM = 4000
    SHOOTER_TOLERANCE_RPM = 200

    # Lookup table: (distance_meters, hood_motor_turns, shooter_rpm)
    # Hood values are in motor turns from home position — tune on robot
    SHOT_TABLE = [
        (1.0, 1.0, 3500),   # Close shot
        (2.0, 2.0, 3800),   # Medium shot
        (3.0, 3.0, 4200),   # Far shot
        (4.0, 3.5, 4500),   # Very far shot
        (5.0, 4.0, 4800),   # Max range shot
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
        self.target_hood_angle = 0.0
        self.feeding = False

        table = ntcore.NetworkTableInstance.getDefault().getTable("Shoot")
        self._distance_pub = table.getDoubleTopic("Distance To Hub").publish()
        self._target_rpm_pub = table.getDoubleTopic("Target RPM").publish()
        self._current_rpm_pub = table.getDoubleTopic("Current RPM").publish()
        self._rpm_error_pub = table.getDoubleTopic("RPM Error").publish()
        self._target_hood_pub = table.getDoubleTopic("Target Hood Angle").publish()
        self._feeding_pub = table.getBooleanTopic("Feeding").publish()

    def initialize(self):
        self.feeding = False

        distance = self.calculate_distance_to_hub()
        self._distance_pub.set(distance)

        self.target_rpm, self.target_hood_angle = self.compute_ballistics(distance)

        self.shooter.set_target_speed(self.target_rpm)
        self.hood.set_target_position(self.target_hood_angle)

        self._target_rpm_pub.set(self.target_rpm)
        self._target_hood_pub.set(self.target_hood_angle)
        self._feeding_pub.set(False)

    def execute(self):
        current_rpm = self.shooter.get_current_speed()
        rpm_error = abs(current_rpm - self.target_rpm)

        self._current_rpm_pub.set(current_rpm)
        self._rpm_error_pub.set(rpm_error)

        if rpm_error < self.SHOOTER_TOLERANCE_RPM and not self.feeding:
            self.feeding = True
            self._feeding_pub.set(True)

            self.indexer.set_target_output(0.5)
            self.kicker.set_target_speed(3000)

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
        robot_translation = robot_pose.translation()
        return robot_translation.distance(self.HUB_POSITION)

    def compute_ballistics(self, distance: float) -> tuple[float, float]:
        if distance <= self.SHOT_TABLE[0][0]:
            return self.SHOT_TABLE[0][2], self.SHOT_TABLE[0][1]

        if distance >= self.SHOT_TABLE[-1][0]:
            return self.SHOT_TABLE[-1][2], self.SHOT_TABLE[-1][1]

        for i in range(len(self.SHOT_TABLE) - 1):
            dist1, angle1, rpm1 = self.SHOT_TABLE[i]
            dist2, angle2, rpm2 = self.SHOT_TABLE[i + 1]

            if dist1 <= distance <= dist2:
                t = (distance - dist1) / (dist2 - dist1)
                hood_angle = angle1 + t * (angle2 - angle1)
                shooter_rpm = rpm1 + t * (rpm2 - rpm1)
                return shooter_rpm, hood_angle

        return self.SHOOTER_TARGET_RPM, 30.0
