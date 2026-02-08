from commands2 import Command
from wpimath.geometry import Pose2d, Translation2d
from wpilib import SmartDashboard
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

    # Hub position on field (center of hub) - adjust these for your field
    # For 2022 Rapid React: hub is at center of field
    HUB_POSITION = Translation2d(8.23, 4.115)  # meters (center of field for 2022)

    # Ballistics constants - TUNE THESE FOR YOUR ROBOT
    SHOOTER_TARGET_RPM = 4000  # Target shooter speed
    SHOOTER_TOLERANCE_RPM = 200  # Acceptable velocity error

    # Lookup table for distance -> hood angle (degrees)
    # Format: (distance_meters, hood_angle_degrees, shooter_rpm)
    SHOT_TABLE = [
        (1.0, 20.0, 3500),   # Close shot
        (2.0, 25.0, 3800),   # Medium shot
        (3.0, 30.0, 4200),   # Far shot
        (4.0, 35.0, 4500),   # Very far shot
        (5.0, 40.0, 4800),   # Max range shot
    ]

    def __init__(self, shooter, indexer, hood, vision, drivetrain):
        """
        Initialize the shoot at hub command.

        :param shooter: ShooterSubSystem instance
        :param indexer: IndexerSubSystem instance
        :param hood: HoodSubSystem instance
        :param vision: VisionSubsystem instance
        :param drivetrain: CommandSwerveDrivetrain instance
        """
        super().__init__()
        self.shooter = shooter
        self.indexer = indexer
        self.hood = hood
        self.vision = vision
        self.drivetrain = drivetrain

        self.addRequirements(shooter, indexer, hood)

        self.target_rpm = self.SHOOTER_TARGET_RPM
        self.target_hood_angle = 0.0
        self.feeding = False

    def initialize(self):
        """Called when command starts."""
        self.feeding = False

        # Calculate distance to hub
        distance = self.calculate_distance_to_hub()
        SmartDashboard.putNumber("Shoot/DistanceToHub", distance)

        # Compute ballistics
        self.target_rpm, self.target_hood_angle = self.compute_ballistics(distance)

        # Set shooter to target speed
        self.shooter.set_target_speed(self.target_rpm)

        # Set hood to target angle
        self.hood.set_target_position(self.target_hood_angle)

        SmartDashboard.putNumber("Shoot/TargetRPM", self.target_rpm)
        SmartDashboard.putNumber("Shoot/TargetHoodAngle", self.target_hood_angle)
        SmartDashboard.putBoolean("Shoot/Feeding", False)

    def execute(self):
        """Called repeatedly while command is scheduled."""
        # Check if shooter is at target speed
        current_rpm = self.shooter.get_current_speed()
        rpm_error = abs(current_rpm - self.target_rpm)

        SmartDashboard.putNumber("Shoot/CurrentRPM", current_rpm)
        SmartDashboard.putNumber("Shoot/RPMError", rpm_error)

        # If shooter is ready and we haven't started feeding yet
        if rpm_error < self.SHOOTER_TOLERANCE_RPM and not self.feeding:
            self.feeding = True
            SmartDashboard.putBoolean("Shoot/Feeding", True)

            # Start feeding with conveyor and kickers
            self.indexer.set_conveyor_target_speed(2000)  # RPM for conveyor
            self.indexer.set_left_kicker_target_speed(3000)  # RPM for left kicker
            self.indexer.set_right_kicker_target_speed(3000)  # RPM for right kicker

    def end(self, interrupted: bool):
        """Called when command ends."""
        # Stop shooter
        self.shooter.set_target_speed(0)

        # Stop indexer
        self.indexer.set_conveyor_target_speed(0)
        self.indexer.set_left_kicker_target_speed(0)
        self.indexer.set_right_kicker_target_speed(0)

        SmartDashboard.putBoolean("Shoot/Feeding", False)

    def isFinished(self) -> bool:
        """
        Command finishes when ball has been fed for sufficient time.
        You might want to add a sensor or timer here.
        For now, it runs until interrupted (whileTrue).
        """
        # Could add a timer or sensor check here
        # For example: return self.feeding and self.timer.hasElapsed(0.5)
        return False

    def calculate_distance_to_hub(self) -> float:
        """
        Calculate distance from robot to hub using odometry.

        :return: Distance in meters
        """
        # Get robot pose from drivetrain
        robot_pose = self.drivetrain.get_state().pose
        robot_translation = robot_pose.translation()

        # Calculate distance to hub
        distance = robot_translation.distance(self.HUB_POSITION)

        return distance

    def compute_ballistics(self, distance: float) -> tuple[float, float]:
        """
        Compute shooter RPM and hood angle based on distance to target.
        Uses interpolation from shot lookup table.

        :param distance: Distance to hub in meters
        :return: Tuple of (shooter_rpm, hood_angle_degrees)
        """
        # If distance is less than minimum in table
        if distance <= self.SHOT_TABLE[0][0]:
            return self.SHOT_TABLE[0][2], self.SHOT_TABLE[0][1]

        # If distance is greater than maximum in table
        if distance >= self.SHOT_TABLE[-1][0]:
            return self.SHOT_TABLE[-1][2], self.SHOT_TABLE[-1][1]

        # Find the two points to interpolate between
        for i in range(len(self.SHOT_TABLE) - 1):
            dist1, angle1, rpm1 = self.SHOT_TABLE[i]
            dist2, angle2, rpm2 = self.SHOT_TABLE[i + 1]

            if dist1 <= distance <= dist2:
                # Linear interpolation
                t = (distance - dist1) / (dist2 - dist1)
                hood_angle = angle1 + t * (angle2 - angle1)
                shooter_rpm = rpm1 + t * (rpm2 - rpm1)

                return shooter_rpm, hood_angle

        # Fallback (shouldn't reach here)
        return self.SHOOTER_TARGET_RPM, 30.0
