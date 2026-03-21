#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
from commands2 import ParallelCommandGroup, cmd
from commands2.button import CommandXboxController, Trigger
from ntcore import NetworkTableInstance
from pathplannerlib.auto import AutoBuilder, PathPlannerPath
from phoenix6 import swerve
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

# from subsystems.climb import ClimbSubsystem
# from commands.climb_commands import ExtendClimb, RetractClimb
from commands.auto_home import AutoHome
from commands.auto_tune_hood import AutoTuneHoodCommand
from commands.auto_tune_intake import AutoTuneIntakeCommand
from commands.home_hood import HomeHood
from commands.home_intake import HomeIntake
from commands.shoot_at_hub import ShootAtHub
from constants import LIMELIGHT_CAMERA_NAME
from generated.tuner_constants import TunerConstants
from subsystems.hood import HOMING_TIMEOUT_SECONDS, HoodSubSystem
from subsystems.indexer import IndexerSubSystem
from subsystems.intake import IntakeSubSystem
from subsystems.kicker import KickerSubSystem
from subsystems.shooter import ShooterSubSystem
from subsystems.vision import VisionSubsystem
from telemetry import Telemetry


def joystick_filter(value):
    """Squared response curve with 5% deadband. Preserves sign for finer control near center."""
    if abs(value) < 0.05:
        return 0
    return value * value * (1 if value > 0 else -1)


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = 1.0 * TunerConstants.speed_at_12_volts  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(0.75)  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = swerve.requests.FieldCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )
        # Snap angle requests - maintain heading while driving
        self._snap_angle = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.1)
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE)
            .with_heading_pid(1, 0, 0)
        )

        self._logger = Telemetry(self._max_speed)

        self._joystick_1 = CommandXboxController(0)

        self.drivetrain = TunerConstants.create_drivetrain()

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # Add vision
        self.limelight = VisionSubsystem(swerve=self.drivetrain, camera=LIMELIGHT_CAMERA_NAME)

        # self.climber = ClimbSubsystem()  # Disabled — PCM not on CAN bus yet
        self.shooter = ShooterSubSystem()
        self.hood = HoodSubSystem()
        self.intake = IntakeSubSystem()
        self.kicker = KickerSubSystem()
        self.indexer = IndexerSubSystem()

        # Default commands — ensure motors stop when no command is running
        self.shooter.setDefaultCommand(self.shooter.run(self.shooter.stop))
        self.kicker.setDefaultCommand(self.kicker.run(self.kicker.stop))
        self.indexer.setDefaultCommand(self.indexer.run(self.indexer.stop))
        self.intake.setDefaultCommand(self.intake.run(self.intake.hold))
        self.hood.setDefaultCommand(self.hood.run(self.hood.stow))

        self.climb_left_path = PathPlannerPath.fromPathFile("climb_left")
        self.climb_right_path = PathPlannerPath.fromPathFile("climb_right")

        # self._do_pigeon_zero = self.drivetrain.seed_field_centric
        # Configure the button bindings — uncomment ONE group at a time:
        self.configureHardwareTestBindings()
        # self.configureTuningTestBindings()
        # self.configureSwerveButtonBindings()
        # self.configureCompetitionBindings()
        # self.configureManualBindings()

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
        Trigger(DriverStation.isDisabled).whileTrue(self.drivetrain.apply_request(lambda: idle).ignoringDisable(True))

        self._joystick_1.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick_1.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(
                        joystick_filter(-self._joystick_1.getLeftY()), joystick_filter(-self._joystick_1.getLeftX())
                    )
                )
            )
        )

        self._joystick_1.povUp().whileTrue(
            self.drivetrain.apply_request(lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(0))
        )
        self._joystick_1.povDown().whileTrue(
            self.drivetrain.apply_request(lambda: self._forward_straight.with_velocity_x(-0.5).with_velocity_y(0))
        )

        self._joystick_1.povLeft().whileTrue(
            self.drivetrain.apply_request(lambda: self._forward_straight.with_velocity_x(0).with_velocity_y(-0.5))
        )
        self._joystick_1.povRight().whileTrue(
            self.drivetrain.apply_request(lambda: self._forward_straight.with_velocity_x(0).with_velocity_y(0.5))
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
        self._joystick_1.leftBumper().onTrue(self.drivetrain.runOnce(self.drivetrain.seed_field_centric))

        self.drivetrain.register_telemetry(lambda state: self._logger.telemeterize(state))

    def configureHardwareTestBindings(self):
        """
        Hardware bring-up bindings — all hold-to-run for safety, NT-tunable speeds.
        Single controller. No swerve.

        Button layout:
          Y  (hold)    = Shooter flywheel (velocity RPM)
          B  (hold)    = Kicker (velocity RPM)
          RB (hold)    = Conveyor (duty cycle)
          RT (hold)    = Intake roller (velocity RPM)
          LT (hold)    = Intake arm (current amps)
          A  (hold)    = Hood up (duty cycle)
          X  (hold)    = Hood down (duty cycle)
          Start        = Home hood
          Back         = Home intake arm
          LB           = (reserved)
          POV Up       = Set hood max limit
          POV Down     = Set hood min limit
          POV Right    = Set intake max limit
          POV Left     = Set intake min limit

        Hood and intake arm track HWTest/ NT positions as default commands.
        """
        table = NetworkTableInstance.getDefault().getTable("HWTest")

        shooter_rpm_sub = table.getDoubleTopic("Shooter RPM").subscribe(5767.0)
        table.getDoubleTopic("Shooter RPM").publish().set(5767.0)

        kicker_rpm_sub = table.getDoubleTopic("Kicker RPM").subscribe(5767.0)
        table.getDoubleTopic("Kicker RPM").publish().set(5767.0)

        conveyor_output_sub = table.getDoubleTopic("Conveyor Output").subscribe(0.3)
        table.getDoubleTopic("Conveyor Output").publish().set(0.3)

        roller_rpm_sub = table.getDoubleTopic("Roller RPM").subscribe(1000.0)
        table.getDoubleTopic("Roller RPM").publish().set(1000.0)

        arm_amps_sub = table.getDoubleTopic("Arm Amps").subscribe(5.0)
        table.getDoubleTopic("Arm Amps").publish().set(5.0)

        hood_output_sub = table.getDoubleTopic("Hood Output").subscribe(0.15)
        table.getDoubleTopic("Hood Output").publish().set(0.15)

        hood_pos_sub = table.getDoubleTopic("Hood Position").subscribe(0.0)
        table.getDoubleTopic("Hood Position").publish().set(0.0)

        arm_pos_sub = table.getDoubleTopic("Arm Position").subscribe(0.0)
        table.getDoubleTopic("Arm Position").publish().set(0.0)

        # ── Motor testing (hold-to-run) ────────────────────────────

        # Y: Hold to run shooter flywheel at NT-tunable RPM
        self._joystick_1.y().whileTrue(
            cmd.runEnd(
                lambda: self.shooter.set_target_speed(shooter_rpm_sub.get()),
                self.shooter.stop,
                self.shooter,
            )
        )

        # B: Hold to run kicker at NT-tunable RPM
        self._joystick_1.b().whileTrue(
            cmd.runEnd(
                lambda: self.kicker.set_target_speed(kicker_rpm_sub.get()),
                self.kicker.stop,
                self.kicker,
            )
        )

        # RB: Hold to run conveyor at NT-tunable duty cycle
        self._joystick_1.rightBumper().whileTrue(
            cmd.runEnd(
                lambda: self.indexer.set_target_output(conveyor_output_sub.get()),
                self.indexer.stop,
                self.indexer,
            )
        )

        # RT: Hold to run intake roller at NT-tunable RPM
        self._joystick_1.rightTrigger().whileTrue(
            cmd.runEnd(
                lambda: self.intake.set_roller_target_speed(roller_rpm_sub.get()),
                lambda: self.intake.set_roller_target_speed(0),
                self.intake,
            )
        )

        # LT: Hold to run intake arm at NT-tunable current (amps)
        self._joystick_1.leftTrigger().whileTrue(
            cmd.runEnd(
                lambda: self.intake.set_arm_target_amp(arm_amps_sub.get()),
                lambda: self.intake.set_arm_target_amp(0),
                self.intake,
            )
        )

        # A: Hold to drive hood up (positive duty cycle)
        self._joystick_1.a().whileTrue(
            cmd.runEnd(
                lambda: self.hood.set_duty_cycle(hood_output_sub.get()),
                self.hood.stop,
                self.hood,
            )
        )

        # X: Hold to drive hood down (negative duty cycle)
        self._joystick_1.x().whileTrue(
            cmd.runEnd(
                lambda: self.hood.set_duty_cycle(-hood_output_sub.get()),
                self.hood.stop,
                self.hood,
            )
        )

        # ── Homing ────────────────────────────────────────────────

        # Start: Home hood
        self._joystick_1.start().onTrue(HomeHood(self.hood).withTimeout(HOMING_TIMEOUT_SECONDS))

        # Back: Home intake arm
        self._joystick_1.back().onTrue(HomeIntake(self.intake).withTimeout(HOMING_TIMEOUT_SECONDS))

        # ── Limit calibration (POV — one-shot) ────────────────────

        # POV Up: Set hood max limit at current position
        self._joystick_1.povUp().onTrue(cmd.runOnce(self.hood.set_max_limit, self.hood))

        # POV Down: Set hood min limit at current position
        self._joystick_1.povDown().onTrue(cmd.runOnce(self.hood.set_min_limit, self.hood))

        # POV Right: Set intake max limit at current position
        self._joystick_1.povRight().onTrue(cmd.runOnce(self.intake.set_max_limit, self.intake))

        # POV Left: Set intake min limit at current position
        self._joystick_1.povLeft().onTrue(cmd.runOnce(self.intake.set_min_limit, self.intake))

        # ── Default commands — track NT positions after homing ─────

        self.hood.setDefaultCommand(self.hood.run(lambda: self.hood.set_target_position(hood_pos_sub.get())))

        self.intake.setDefaultCommand(self.intake.run(lambda: self.intake.set_arm_target_position(arm_pos_sub.get())))

    def configureTuningTestBindings(self):
        """
        PID auto-tuning bindings. Requires homing and limits set first.

        Button layout:
          Start = Auto-tune hood PID (Z-N relay method)
          Back  = Auto-tune intake arm PID (Z-N relay method)
        """
        self._joystick_1.start().onTrue(AutoTuneHoodCommand(self.hood))
        self._joystick_1.back().onTrue(AutoTuneIntakeCommand(self.intake))

    def configureCompetitionBindings(self):
        # Right trigger: Shoot at hub while aiming
        self._joystick_1.rightTrigger().whileTrue(
            ParallelCommandGroup(
                # Aim robot at hub
                self.drivetrain.apply_request(
                    lambda: (
                        self._snap_angle.with_target_direction(self.calculate_angle_to_hub())
                        .with_velocity_x(joystick_filter(-self._joystick_1.getLeftY()) * self._max_speed)
                        .with_velocity_y(joystick_filter(-self._joystick_1.getLeftX()) * self._max_speed)
                    )
                ),
                # Shoot with ballistics calculation
                ShootAtHub(self.shooter, self.kicker, self.indexer, self.hood, self.limelight, self.drivetrain),
            )
        )

        # Climb function — disabled until PCM is on CAN bus
        # self._joystick_1.povUp().onTrue(ExtendClimb(self.climber))
        # self._joystick_1.povDown().onTrue(RetractClimb(self.climber))

    def configureManualBindings(self):
        """
        Manual driving + shooting without vision/auto-aim.
        Use when limelight is unavailable or for practice.

        Button layout:
          Left stick   = Translation (field-centric)
          Right stick X = Rotation
          RT (hold)    = Aim hood + spin up shooter/kicker, auto-feed when up to speed
          LT (hold)    = Deploy intake arm + spin rollers
          Start        = Home hood
          Back         = Home intake arm
          POV          = Cardinal drive at 0.5 m/s
        Auto-brake when sticks are idle.
        """

        # --- Swerve: sticks + auto-brake when idle ---
        def _drive_or_brake():
            vx = joystick_filter(-self._joystick_1.getLeftY())
            vy = joystick_filter(-self._joystick_1.getLeftX())
            vr = joystick_filter(-self._joystick_1.getRightX())
            if vx == 0 and vy == 0 and vr == 0:
                return self._brake
            return (
                self._drive.with_velocity_x(vx * self._max_speed)
                .with_velocity_y(vy * self._max_speed)
                .with_rotational_rate(vr * self._max_angular_rate)
            )

        self.drivetrain.setDefaultCommand(self.drivetrain.apply_request(_drive_or_brake))

        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(self.drivetrain.apply_request(lambda: idle).ignoringDisable(True))

        self._joystick_1.povUp().whileTrue(
            self.drivetrain.apply_request(lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(0))
        )
        self._joystick_1.povDown().whileTrue(
            self.drivetrain.apply_request(lambda: self._forward_straight.with_velocity_x(-0.5).with_velocity_y(0))
        )
        self._joystick_1.povLeft().whileTrue(
            self.drivetrain.apply_request(lambda: self._forward_straight.with_velocity_x(0).with_velocity_y(-0.5))
        )
        self._joystick_1.povRight().whileTrue(
            self.drivetrain.apply_request(lambda: self._forward_straight.with_velocity_x(0).with_velocity_y(0.5))
        )

        self.drivetrain.register_telemetry(lambda state: self._logger.telemeterize(state))

        # --- Mechanism bindings ---
        table = NetworkTableInstance.getDefault().getTable("Manual")

        shooter_rpm_sub = table.getDoubleTopic("Shooter RPM").subscribe(4000.0)
        table.getDoubleTopic("Shooter RPM").publish().set(4000.0)

        hood_pos_sub = table.getDoubleTopic("Hood Position").subscribe(2.0)
        table.getDoubleTopic("Hood Position").publish().set(2.0)

        SHOOTER_TOLERANCE_RPM = 200
        KICKER_TOLERANCE_RPM = 500

        def _shoot_execute():
            target_rpm = shooter_rpm_sub.get()
            self.hood.set_target_position(hood_pos_sub.get())
            self.shooter.set_target_speed(target_rpm)

            shooter_ready = abs(self.shooter.get_current_speed() - target_rpm) < SHOOTER_TOLERANCE_RPM
            if shooter_ready:
                self.kicker.set_target_speed(target_rpm)
            else:
                self.kicker.stop()

            kicker_ready = abs(self.kicker.get_current_speed() - target_rpm) < KICKER_TOLERANCE_RPM
            if shooter_ready and kicker_ready:
                self.indexer.set_target_output(0.5)
            else:
                self.indexer.stop()

        # RT: Hold to shoot — staged: shooter → kicker → conveyor
        self._joystick_1.rightTrigger().whileTrue(
            cmd.runEnd(
                _shoot_execute,
                lambda: (self.shooter.stop(), self.kicker.stop(), self.indexer.stop()),
                self.shooter,
                self.kicker,
                self.hood,
                self.indexer,
            )
        )

        # LT: Hold to deploy intake arm + spin rollers (does not stow on release)
        self._joystick_1.leftTrigger().whileTrue(
            cmd.runEnd(
                lambda: (
                    self.intake.deploy(),
                    self.intake.set_roller_target_speed(3000),
                ),
                lambda: self.intake.set_roller_target_speed(0),
                self.intake,
            )
        )

        # Start: Home hood
        self._joystick_1.start().onTrue(HomeHood(self.hood).withTimeout(HOMING_TIMEOUT_SECONDS))

        # Back: Home intake arm
        self._joystick_1.back().onTrue(HomeIntake(self.intake).withTimeout(HOMING_TIMEOUT_SECONDS))

    def calculate_angle_to_hub(self) -> Rotation2d:
        """
        Calculate the angle the robot should face to aim at the hub.

        :return: Rotation2d representing the angle to the hub
        """
        import math

        from constants import HUB_POSITION

        robot_pose = self.drivetrain.get_state().pose
        robot_translation = robot_pose.translation()

        dx = HUB_POSITION.X() - robot_translation.X()
        dy = HUB_POSITION.Y() - robot_translation.Y()

        return Rotation2d(math.atan2(dy, dx))

    def getAutoHomeCommand(self) -> commands2.Command:
        """Returns a command that homes the hood and intake arm sequentially."""
        return AutoHome(self.hood, self.intake)

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
