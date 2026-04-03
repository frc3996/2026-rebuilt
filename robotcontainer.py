#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

import commands2
from commands2 import ParallelCommandGroup, cmd
from commands2.button import CommandXboxController, Trigger
from ntcore import NetworkTableInstance
from pathplannerlib.auto import (AutoBuilder, NamedCommands, PathPlannerAuto,
                                 PathPlannerPath)
from pathplannerlib.events import EventTrigger
from phoenix6 import swerve
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

# from subsystems.climb import ClimbSubsystem
# from commands.climb_commands import ExtendClimb, RetractClimb
from commands.auto_home import AutoHome
from commands.auto_tune_hood import AutoTuneHoodCommand
from commands.auto_tune_intake import AutoTuneIntakeCommand
from commands.auto_tune_kicker import AutoTuneKickerCommand
from commands.auto_tune_shooter import AutoTuneShooterCommand
from commands.calibrate_ff import CalibrateFF
from commands.home_hood import HomeHood
from commands.home_intake import HomeIntake
from commands.hub_shot import HubShot, VirtualGoal
from commands.safe_retract_intake import SafeRetractIntake
from commands.tune_shot import TuneShot
from generated.tuner_constants import TunerConstants
from subsystems.hood import HOMING_TIMEOUT_SECONDS, HoodSubSystem
from subsystems.indexer import IndexerSubSystem
from subsystems.intake import IntakeSubSystem
from subsystems.kicker import KickerSubSystem
from subsystems.shooter import ShooterSubSystem
from subsystems.vision import CAMERAS, VisionSubsystem
from telemetry import Telemetry


def joystick_filter(value):
    """Squared response curve with 5% deadband. Rescales so output ramps smoothly from 0."""
    if abs(value) < 0.05:
        return 0
    scaled = (abs(value) - 0.05) / 0.95
    return scaled * scaled * (1 if value > 0 else -1)


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
        self._forward_straight = swerve.requests.FieldCentric().with_drive_request_type(
            swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
        )
        # Snap angle requests - maintain heading while driving
        self._snap_angle = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.1)
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
            .with_heading_pid(8, 0, 0.2)
        )

        self._logger = Telemetry(self._max_speed)

        self._joystick_1 = CommandXboxController(0)

        self.drivetrain = TunerConstants.create_drivetrain()
        self._virtual_goal = VirtualGoal(self.drivetrain)

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # Add vision
        self.limelight = VisionSubsystem(swerve=self.drivetrain, cameras=CAMERAS)

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

        self.climb_left_path = PathPlannerPath.fromPathFile("test move")
        self.climb_right_path = PathPlannerPath.fromPathFile("test move")

        # ── Named Commands (for sequential auto structure) ──
        NamedCommands.registerCommand(
            "retract-intake",
            SafeRetractIntake(self.intake),
        )

        # ── Event Triggers (for zoned event markers on paths) ──
        # In PathPlanner GUI, create event markers with these trigger names.
        # Use zones (start + end position) for whileTrue behavior.
        hubshot_trigger = EventTrigger("hubshot")
        hubshot_trigger.whileTrue(
            HubShot(
                self.shooter, self.kicker, self.indexer, self.hood, self._virtual_goal
            )
        )
        hubshot_trigger.onFalse(self._clearout_command())

        EventTrigger("intake").whileTrue(
            cmd.startEnd(
                lambda: (
                    self.intake.deploy(),
                    self.intake.set_roller_duty_cycle(1.0),
                ),
                lambda: (self.intake.set_roller_duty_cycle(0),),
                self.intake,
            )
        )

        # Configure the button bindings — uncomment ONE group at a time:
        # self.configureSwerveButtonBindings()
        # self.configureHardwareTestBindings()
        # self.configureTuningTestBindings()
        # self.configureManualBindings()
        self.configureCompetitionBindings()

    def _clearout_command(self):
        """Reverse conveyor while shooter+kicker keep running, then stop."""
        return cmd.run(
            lambda: (
                self.shooter.set_target_speed(self._virtual_goal.last_rpm),
                self.kicker.set_duty_cycle(1.0),
                self.indexer.set_target_output(-1.0),
            ),
            self.shooter,
            self.kicker,
            self.indexer,
        ).withTimeout(0.5)

    def _drive_or_brake(self):
        """Swerve request: drive from joystick input, or brake when sticks are idle."""
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

    def configureSwerveButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(self._drive_or_brake)
        )

        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        self._joystick_1.a().whileTrue(
            self.drivetrain.apply_request(lambda: self._brake)
        )
        self._joystick_1.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(
                        joystick_filter(-self._joystick_1.getLeftY()),
                        joystick_filter(-self._joystick_1.getLeftX()),
                    )
                )
            )
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def configureHardwareTestBindings(self):
        """
        Hardware bring-up bindings — all hold-to-run, duty cycle only (no PID).
        Single controller. No swerve.

        Button layout:
          Y  (hold)    = Shooter flywheel (duty cycle)
          B  (hold)    = Kicker (duty cycle)
          RB (hold)    = Conveyor (duty cycle)
          RT (hold)    = Intake roller (duty cycle)
          LT (hold)    = Intake arm (duty cycle)
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

        shooter_output_sub = table.getDoubleTopic("Shooter Output").subscribe(0.3)
        table.getDoubleTopic("Shooter Output").publish().set(0.3)

        kicker_output_sub = table.getDoubleTopic("Kicker Output").subscribe(0.3)
        table.getDoubleTopic("Kicker Output").publish().set(0.3)

        conveyor_output_sub = table.getDoubleTopic("Conveyor Output").subscribe(1.0)
        table.getDoubleTopic("Conveyor Output").publish().set(1.0)

        roller_output_sub = table.getDoubleTopic("Roller Output").subscribe(1.0)
        table.getDoubleTopic("Roller Output").publish().set(1.0)

        arm_output_sub = table.getDoubleTopic("Arm Output").subscribe(0.15)
        table.getDoubleTopic("Arm Output").publish().set(0.15)

        hood_output_sub = table.getDoubleTopic("Hood Output").subscribe(0.15)
        table.getDoubleTopic("Hood Output").publish().set(0.15)

        hood_pos_sub = table.getDoubleTopic("Hood Position").subscribe(0.0)
        hood_pos_pub = table.getDoubleTopic("Hood Position").publish()
        hood_pos_pub.set(0.0)

        arm_pos_sub = table.getDoubleTopic("Arm Position").subscribe(0.0)
        arm_pos_pub = table.getDoubleTopic("Arm Position").publish()
        arm_pos_pub.set(0.0)

        # ── Motor testing (hold-to-run, duty cycle only) ──────────

        # Y: Hold to run shooter flywheel at NT-tunable duty cycle
        self._joystick_1.y().whileTrue(
            cmd.runEnd(
                lambda: self.shooter.set_duty_cycle(shooter_output_sub.get()),
                self.shooter.stop,
                self.shooter,
            )
        )

        # B: Hold to run kicker at NT-tunable duty cycle
        self._joystick_1.b().whileTrue(
            cmd.runEnd(
                lambda: self.kicker.set_duty_cycle(kicker_output_sub.get()),
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

        # RT: Hold to run intake roller at NT-tunable duty cycle
        self._joystick_1.rightTrigger().whileTrue(
            cmd.runEnd(
                lambda: self.intake.set_roller_duty_cycle(roller_output_sub.get()),
                lambda: self.intake.set_roller_duty_cycle(0),
                self.intake,
            )
        )

        # LT: Hold to run intake arm at NT-tunable duty cycle
        self._joystick_1.leftTrigger().whileTrue(
            cmd.runEnd(
                lambda: self.intake.set_arm_duty_cycle(arm_output_sub.get()),
                lambda: self.intake.set_arm_duty_cycle(0),
                self.intake,
            )
        )

        # A: Hold to drive hood up (positive duty cycle), hold position on release
        self._joystick_1.a().whileTrue(
            cmd.runEnd(
                lambda: self.hood.set_duty_cycle(hood_output_sub.get()),
                lambda: hood_pos_pub.set(self.hood.get_current_position()),
                self.hood,
            )
        )

        # X: Hold to drive hood down (negative duty cycle), hold position on release
        self._joystick_1.x().whileTrue(
            cmd.runEnd(
                lambda: self.hood.set_duty_cycle(-hood_output_sub.get()),
                lambda: hood_pos_pub.set(self.hood.get_current_position()),
                self.hood,
            )
        )

        # ── Homing ────────────────────────────────────────────────

        # Start: Home hood, then move to min soft limit
        self._joystick_1.start().onTrue(
            HomeHood(self.hood)
            .withTimeout(HOMING_TIMEOUT_SECONDS)
            .finallyDo(lambda interrupted: hood_pos_pub.set(self.hood.min_rotations))
        )

        # Back: Home intake arm, reset NT position regardless of outcome
        self._joystick_1.back().onTrue(
            HomeIntake(self.intake)
            .withTimeout(HOMING_TIMEOUT_SECONDS)
            .finallyDo(
                lambda interrupted: arm_pos_pub.set(self.intake.get_arm_position())
            )
        )

        # ── Limit calibration (POV — one-shot) ────────────────────

        # POV Up: Set hood max limit at current position
        self._joystick_1.povUp().onTrue(cmd.runOnce(self.hood.set_max_limit, self.hood))

        # POV Down: Set hood min limit at current position
        self._joystick_1.povDown().onTrue(
            cmd.runOnce(self.hood.set_min_limit, self.hood)
        )

        # POV Right: Set intake max limit at current position
        self._joystick_1.povRight().onTrue(
            cmd.runOnce(self.intake.set_max_limit, self.intake)
        )

        # POV Left: Set intake min limit at current position
        self._joystick_1.povLeft().onTrue(
            cmd.runOnce(self.intake.set_min_limit, self.intake)
        )

        # ── Default commands — track NT positions after homing, stop if not homed ─

        self.hood.setDefaultCommand(
            self.hood.run(
                lambda: (
                    self.hood.set_target_position(hood_pos_sub.get())
                    if self.hood.is_homed
                    else self.hood.stop()
                )
            )
        )
        self.intake.setDefaultCommand(
            self.intake.run(
                lambda: (
                    self.intake.set_arm_target_position(arm_pos_sub.get())
                    if self.intake.homed
                    else self.intake.stop_arm()
                )
            )
        )

    def configureTuningTestBindings(self):
        """
        PID auto-tuning + stick position control. Requires homing first.

        Button layout:
          Left stick Y  = Hood position (center=min, up=max soft limit)
          Right stick Y = Intake arm position (center=home, down=min soft limit)
          LB            = Home hood
          RB            = Home intake arm
          Start         = Auto-tune hood PID (Z-N relay method)
          Back          = Auto-tune intake arm PID (Z-N relay method)
          Y             = Calibrate shooter FF (voltage ramp)
          B             = Calibrate kicker right FF (voltage ramp)
          A             = Calibrate kicker left FF (voltage ramp)
        """

        # Left stick Y: hood position — center=min, full up=max
        # getLeftY() returns -1 (up) to +1 (down), clamp negative half only
        def _hood_from_stick():
            if not self.hood.is_homed:
                return
            raw = -self._joystick_1.getLeftY()  # 0..1 when pushed up
            t = max(0.0, raw)
            pos = self.hood.min_rotations + t * (
                self.hood.max_rotations - self.hood.min_rotations
            )
            self.hood.set_target_position(pos)

        self.hood.setDefaultCommand(self.hood.run(_hood_from_stick))

        # Right stick Y: intake arm — stick up=deployed (min), stick down=stowed (max)
        def _intake_from_stick():
            if not self.intake.homed:
                return
            raw = -self._joystick_1.getRightY()  # 0..1 when pushed up
            t = max(0.0, raw)
            if t < 0.05:
                self.intake.hold()
                return
            pos = self.intake.max_rotations + t * (
                self.intake.min_rotations - self.intake.max_rotations
            )
            self.intake.set_arm_target_position(pos)

        self.intake.setDefaultCommand(self.intake.run(_intake_from_stick))

        # Homing
        self._joystick_1.leftBumper().onTrue(
            HomeHood(self.hood).withTimeout(HOMING_TIMEOUT_SECONDS)
        )
        self._joystick_1.rightBumper().onTrue(
            HomeIntake(self.intake).withTimeout(HOMING_TIMEOUT_SECONDS)
        )

        # Auto-tune (hood/intake use Z-N for position PID)
        self._joystick_1.start().onTrue(AutoTuneHoodCommand(self.hood))
        self._joystick_1.back().onTrue(AutoTuneIntakeCommand(self.intake))
        # FF calibration (shooter/kicker use voltage ramp for feedforward)
        self._joystick_1.y().onTrue(CalibrateFF(self.shooter, "CalFF Shooter"))
        self._joystick_1.b().onTrue(
            CalibrateFF(
                self.kicker,
                "CalFF Kicker Right",
                set_output=self.kicker.set_right_duty_cycle,
                get_speed=self.kicker.get_right_speed,
            )
        )
        self._joystick_1.a().onTrue(
            CalibrateFF(
                self.kicker,
                "CalFF Kicker Left",
                set_output=self.kicker.set_left_duty_cycle,
                get_speed=self.kicker.get_left_speed,
            )
        )

    def configureCompetitionBindings(self):
        """
        Full competition bindings — auto-aim shooting with virtual goal compensation.

        Button layout:
          Left stick   = Translation (field-centric)
          Right stick X = Rotation (free drive) / auto-aim (while RT held)
          RT (hold)    = Aim at hub + shoot (auto RPM + hood from lookup table)
          LT (hold)    = Deploy intake arm + spin rollers
          Y            = Stow intake arm
          RB           = Home hood
          LB           = Home intake arm
          Back         = Re-home hood + intake
        Auto-brake when sticks are idle.
        """
        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(self._drive_or_brake)
        )

        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

        # RT: Hold to aim at hub + shoot (auto RPM + hood from lookup table)
        self._shoot_at_hub = HubShot(
            self.shooter, self.kicker, self.indexer, self.hood, self._virtual_goal
        )
        _SHOOT_DRIVE_SCALE = (
            0.25  # Limit swerve to 25% while shooting to prevent brownouts
        )

        def _hub_shot_request():
            aim, ff = self._virtual_goal.calculate()
            return (
                self._snap_angle.with_target_direction(aim)
                .with_target_rate_feedforward(ff)
                .with_velocity_x(
                    joystick_filter(-self._joystick_1.getLeftY())
                    * self._max_speed
                    * _SHOOT_DRIVE_SCALE
                )
                .with_velocity_y(
                    joystick_filter(-self._joystick_1.getLeftX())
                    * self._max_speed
                    * _SHOOT_DRIVE_SCALE
                )
            )

        self._joystick_1.rightTrigger().whileTrue(
            ParallelCommandGroup(
                self._shoot_at_hub,
                self.drivetrain.apply_request(_hub_shot_request),
            )
        )

        # RT release: clearout — reverse conveyor, keep shooter+kicker for 1s
        self._joystick_1.rightTrigger().onFalse(self._clearout_command())

        # LT: Hold to deploy intake arm + spin rollers (speed limited)
        _INTAKE_DRIVE_SCALE = 0.25  # Limit swerve to 50% while intaking  # TUNE

        def _intake_drive_request():
            return (
                self._drive.with_velocity_x(
                    joystick_filter(-self._joystick_1.getLeftY())
                    * self._max_speed
                    * _INTAKE_DRIVE_SCALE
                )
                .with_velocity_y(
                    joystick_filter(-self._joystick_1.getLeftX())
                    * self._max_speed
                    * _INTAKE_DRIVE_SCALE
                )
                .with_rotational_rate(
                    joystick_filter(-self._joystick_1.getRightX())
                    * self._max_angular_rate
                )
            )

        self._joystick_1.leftTrigger().whileTrue(
            ParallelCommandGroup(
                cmd.runEnd(
                    lambda: (
                        self.intake.deploy(),
                        self.intake.set_roller_duty_cycle(1.0),
                    ),
                    lambda: self.intake.set_roller_duty_cycle(0),
                    self.intake,
                ),
                self.drivetrain.apply_request(_intake_drive_request),
            )
        )

        # Y: Stow intake arm
        self._joystick_1.y().onTrue(SafeRetractIntake(self.intake))

        # RB: Home hood
        self._joystick_1.rightBumper().onTrue(
            HomeHood(self.hood).withTimeout(HOMING_TIMEOUT_SECONDS)
        )

        # LB: Home intake arm
        self._joystick_1.leftBumper().onTrue(
            HomeIntake(self.intake).withTimeout(HOMING_TIMEOUT_SECONDS)
        )

        # Back: Re-home hood + intake
        self._joystick_1.back().onTrue(AutoHome(self.hood, self.intake))

        # Climb function — disabled until PCM is on CAN bus
        # self._joystick_1.povUp().onTrue(ExtendClimb(self.climber))
        # self._joystick_1.povDown().onTrue(RetractClimb(self.climber))

    def configureManualBindings(self):
        """
        Manual driving + shooting with vision-corrected aim at hub.
        Hood and RPM set via NT / POV controls.

        Button layout:
          Left stick   = Translation (field-centric)
          Right stick X = Rotation
          RT (hold)    = Aim hood + spin up shooter/kicker, auto-feed when up to speed
          LT (hold)    = Deploy intake arm + spin rollers
          RB           = Home hood
          LB           = Home intake arm
          Start        = Record shot data point (distance, hood, RPM)
          POV Up/Down  = Adjust hood position (slow nudge)
          POV Left/Right = Adjust shooter RPM
        Auto-brake when sticks are idle.
        """

        self.drivetrain.setDefaultCommand(
            self.drivetrain.apply_request(self._drive_or_brake)
        )

        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True)
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

        # --- Mechanism bindings ---
        table = NetworkTableInstance.getDefault().getTable("Manual")

        hood_step = 0.025  # turns per loop (~0.25 turns/sec)
        rpm_step = 20.0  # RPM per loop (~250 RPM/sec)

        shooter_rpm_pub = table.getDoubleTopic("Shooter RPM").publish()
        shooter_rpm_pub.set(2000.0)
        shooter_rpm_sub = table.getDoubleTopic("Shooter RPM").subscribe(2000.0)

        hood_mid = (self.hood.min_rotations + self.hood.max_rotations) / 2
        hood_pos_pub = table.getDoubleTopic("Hood Position").publish()
        hood_pos_pub.set(hood_mid)
        hood_pos_sub = table.getDoubleTopic("Hood Position").subscribe(hood_mid)

        # POV Up: Nudge hood position up (hold to repeat) — applied live
        def _hood_up():
            pos = min(hood_pos_sub.get() + hood_step, self.hood.max_rotations)
            hood_pos_pub.set(pos)
            self.hood.set_target_position(pos)

        self._joystick_1.povUp().whileTrue(cmd.run(_hood_up, self.hood))

        # POV Down: Nudge hood position down (hold to repeat) — applied live
        def _hood_down():
            pos = max(hood_pos_sub.get() - hood_step, self.hood.min_rotations)
            hood_pos_pub.set(pos)
            self.hood.set_target_position(pos)

        self._joystick_1.povDown().whileTrue(cmd.run(_hood_down, self.hood))

        # POV Right: Increase shooter RPM (hold to repeat)
        self._joystick_1.povRight().whileTrue(
            cmd.run(lambda: shooter_rpm_pub.set(shooter_rpm_sub.get() + rpm_step))
        )

        # POV Left: Decrease shooter RPM (hold to repeat)
        self._joystick_1.povLeft().whileTrue(
            cmd.run(
                lambda: shooter_rpm_pub.set(max(0, shooter_rpm_sub.get() - rpm_step))
            )
        )

        # RT: Hold to aim at hub + shoot
        self._tune_shot = TuneShot(
            self.shooter, self.kicker, self.indexer, self.hood, self.drivetrain
        )

        def _tune_shot_request():
            hub = self._virtual_goal._get_hub()
            pose = self.drivetrain.get_state().pose
            dx = hub.X() - pose.X()
            dy = hub.Y() - pose.Y()
            return (
                self._snap_angle.with_target_direction(Rotation2d(math.atan2(dy, dx)))
                .with_velocity_x(
                    joystick_filter(-self._joystick_1.getLeftY()) * self._max_speed
                )
                .with_velocity_y(
                    joystick_filter(-self._joystick_1.getLeftX()) * self._max_speed
                )
            )

        self._joystick_1.rightTrigger().whileTrue(
            ParallelCommandGroup(
                self._tune_shot,
                self.drivetrain.apply_request(_tune_shot_request),
            )
        )

        # Start: Record shot data point (distance, hood_turns, RPM) to CSV + NT
        self._joystick_1.start().onTrue(cmd.runOnce(self._tune_shot.record_entry))

        # LT: Hold to deploy intake arm + spin rollers (does not stow on release)
        self._joystick_1.leftTrigger().whileTrue(
            cmd.runEnd(
                lambda: (
                    self.intake.deploy(),
                    self.intake.set_roller_duty_cycle(1.0),
                ),
                lambda: self.intake.set_roller_duty_cycle(0),
                self.intake,
            )
        )

        # Y: Stow intake arm
        self._joystick_1.y().onTrue(SafeRetractIntake(self.intake))

        # RB: Home hood
        self._joystick_1.rightBumper().onTrue(
            HomeHood(self.hood).withTimeout(HOMING_TIMEOUT_SECONDS)
        )

        # LB: Home intake arm
        self._joystick_1.leftBumper().onTrue(
            HomeIntake(self.intake).withTimeout(HOMING_TIMEOUT_SECONDS)
        )

        # Back: Re-home hood + intake
        self._joystick_1.back().onTrue(AutoHome(self.hood, self.intake))

    def getAutoHomeCommand(self) -> commands2.Command:
        """Returns a command that homes the hood and intake arm sequentially."""
        return AutoHome(self.hood, self.intake)

    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """

        return self._auto_chooser.getSelected()
