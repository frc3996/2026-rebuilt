#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from commands2 import ParallelCommandGroup, cmd
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry

from pathplannerlib.auto import AutoBuilder, PathPlannerPath
from phoenix6 import swerve
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians
from subsystems.vision import VisionSubsystem
from subsystems.climb import ClimbSubsystem
from commands.climb_commands import ExtendClimb, RetractClimb
from commands.shoot_at_hub import ShootAtHub
from subsystems.hood import HoodSubSystem
from subsystems.indexer import IndexerSubSystem
from subsystems.intake import IntakeSubSystem
from subsystems.shooter import ShooterSubSystem


def joystick_filter(value):
    if abs(value) < 0.05:
        return 0

    value = (value * value) * (value / abs(value))
    return value

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            1.0 * TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = (
            swerve.requests.FieldCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )
        # Snap angle requests - maintain heading while driving
        self._snap_angle = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.1)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
            .with_heading_pid(1, 0, 0)
        )

        self._logger = Telemetry(self._max_speed)

        self._joystick_1 = CommandXboxController(0)
        self._joystick_2 = CommandXboxController(1)

        self.drivetrain = TunerConstants.create_drivetrain()

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # Add vision
        self.limelight = VisionSubsystem(
            swerve=self.drivetrain,
            camera="limelight-back"
        )

        self.climber = ClimbSubsystem()
        self.shooter = ShooterSubSystem()
        self.hood = HoodSubSystem()
        self.intake = IntakeSubSystem()
        self.indexer = IndexerSubSystem()

        self.climb_left_path = PathPlannerPath.fromPathFile("climb_left")
        self.climb_right_path = PathPlannerPath.fromPathFile("climb_right")

        # self._do_pigeon_zero = self.drivetrain.seed_field_centric
        # Configure the button bindings
        self.configureSwerveButtonBindings()
        self.configureOtherButtonBindings()


    def configureSwerveButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        joystick_filter(-self._joystick_1.getLeftY()) * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        joystick_filter(-self._joystick_1.getLeftX()) * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        joystick_filter(-self._joystick_1.getRightX()) * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        self._joystick_1.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick_1.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(joystick_filter(-self._joystick_1.getLeftY()), joystick_filter(-self._joystick_1.getLeftX()))
                )
            )
        )

        self._joystick_1.povUp().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(0)
            )
        )
        self._joystick_1.povDown().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(-0.5).with_velocity_y(0)
            )
        )

        self._joystick_1.povLeft().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0).with_velocity_y(-0.5)
            )
        )
        self._joystick_1.povRight().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0).with_velocity_y(0.5)
            )
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        # (self._joystick.back() & self._joystick.y()).whileTrue(
        #     self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        # )
        # (self._joystick.back() & self._joystick.x()).whileTrue(
        #     self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        # )
        # (self._joystick.start() & self._joystick.y()).whileTrue(
        #     self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        # )
        # (self._joystick.start() & self._joystick.x()).whileTrue(
        #     self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        # )

        # reset the field-centric heading on left bumper press
        self._joystick_1.leftBumper().onTrue(
            self.drivetrain.runOnce(self.drivetrain.seed_field_centric)
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def configureOtherButtonBindings(self):
        # Right trigger: Shoot at hub while aiming
        self._joystick_1.rightTrigger().whileTrue(ParallelCommandGroup(
            # Aim robot at hub
            self.drivetrain.apply_request(
                lambda: self._snap_angle
                .with_target_direction(self.calculate_angle_to_hub())
                .with_velocity_x(joystick_filter(-self._joystick_1.getLeftY()) * self._max_speed)
                .with_velocity_y(joystick_filter(-self._joystick_1.getLeftX()) * self._max_speed)
            ),
            # Shoot with ballistics calculation
            ShootAtHub(self.shooter, self.indexer, self.hood, self.limelight, self.drivetrain)
        ))




        # Climb function
        self._joystick_2.povUp().onTrue(ExtendClimb(self.climber))
        self._joystick_2.povDown().onTrue(RetractClimb(self.climber))

        # Snap angle commands - maintain heading while driving
        # Left POV: Face left (-90 degrees)
        self._joystick_2.povLeft().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._snap_angle
                .with_target_direction(Rotation2d.fromDegrees(-90))
                .with_velocity_x(joystick_filter(-self._joystick_1.getLeftY()) * self._max_speed)
                .with_velocity_y(joystick_filter(-self._joystick_1.getLeftX()) * self._max_speed)
            )
        )
        # Right POV: Face right (90 degrees)
        self._joystick_2.povRight().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._snap_angle
                .with_target_direction(Rotation2d.fromDegrees(90))
                .with_velocity_x(joystick_filter(-self._joystick_1.getLeftY()) * self._max_speed)
                .with_velocity_y(joystick_filter(-self._joystick_1.getLeftX()) * self._max_speed)
            )
        )

    def calculate_angle_to_hub(self) -> Rotation2d:
        """
        Calculate the angle the robot should face to aim at the hub.

        :return: Rotation2d representing the angle to the hub
        """
        from wpimath.geometry import Translation2d
        import math

        # Hub position (center of field for 2022 Rapid React)
        # Adjust these coordinates for your specific field/game
        HUB_POSITION = Translation2d(8.23, 4.115)  # meters

        # Get current robot pose
        robot_pose = self.drivetrain.get_state().pose
        robot_translation = robot_pose.translation()

        # Calculate vector from robot to hub
        dx = HUB_POSITION.X() - robot_translation.X()
        dy = HUB_POSITION.Y() - robot_translation.Y()

        # Calculate angle (atan2 handles quadrants correctly)
        angle_radians = math.atan2(dy, dx)

        return Rotation2d(angle_radians)

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """

        return self._auto_chooser.getSelected()

        # # Simple drive forward auton
        # idle = swerve.requests.Idle()
        # return cmd.sequence(
        #     # Reset our field centric heading to match the robot
        #     # facing away from our alliance station wall (0 deg).
        #     self.drivetrain.runOnce(
        #         lambda: self.drivetrain.seed_field_centric(Rotation2d.fromDegrees(0))
        #     ),
        #     # Then slowly drive forward (away from us) for 5 seconds.
        #     self.drivetrain.apply_request(
        #         lambda: (
        #             self._drive.with_velocity_x(0.5)
        #             .with_velocity_y(0)
        #             .with_rotational_rate(0)
        #         )
        #     )
        #     .withTimeout(5.0),
        #     # Finally idle for the rest of auton
        #     self.drivetrain.apply_request(lambda: idle)
        # )
