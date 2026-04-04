import csv
from pathlib import Path

import ntcore
import rev
from commands2 import Command
from wpilib import DriverStation, Timer

from commands.hub_shot import BLUE_HUB, RED_HUB
from subsystems.hood import HoodSubSystem
from subsystems.indexer import IndexerSubSystem
from subsystems.kicker import KickerSubSystem
from subsystems.shooter import (
    KP_HIGH,
    KP_LOW,
     KP_MID,
    ShooterSubSystem,
)

SHOOTER_TOLERANCE_RPM = 75
SHOT_LOG_PATH = Path("/home/lvuser/shot_tuning.csv")


class TuneShot(Command):
    """
    Calibration command for building the shot lookup table.

    Reads target shooter RPM and hood position from NetworkTables.
    Stages shooter → kicker → indexer and auto-feeds when up to speed.
    Call record_entry() to log a (distance, hood_turns, shooter_rpm) data point.
    Recorded entries are published to NT as a string array for display.
    """

    def __init__(
        self,
        shooter: ShooterSubSystem,
        kicker: KickerSubSystem,
        indexer: IndexerSubSystem,
        hood: HoodSubSystem,
        drivetrain,
    ) -> None:
        super().__init__()
        self.shooter = shooter
        self.kicker = kicker
        self.indexer = indexer
        self.hood = hood
        self._drivetrain = drivetrain
        self.addRequirements(shooter, kicker, indexer, hood)

        table = ntcore.NetworkTableInstance.getDefault().getTable("Manual")

        # Tunable inputs (shared with configureManualBindings POV controls)
        self._rpm_sub = table.getDoubleTopic("Shooter RPM").subscribe(2000.0)
        self._hood_sub = table.getDoubleTopic("Hood Position").subscribe(0.1)

        # Telemetry
        self._distance_pub = table.getDoubleTopic("Distance To Hub").publish()
        self._feeding_pub = table.getBooleanTopic("Feeding").publish()
        self._shooter_ready_pub = table.getBooleanTopic("Shooter Ready").publish()
        self._current_rpm_pub = table.getDoubleTopic("Current RPM").publish()
        self._rpm_error_pub = table.getDoubleTopic("RPM Error").publish()

        # Recorded calibration entries
        self._entries_pub = table.getStringArrayTopic("Recorded Entries").publish()
        self._entry_count_pub = table.getIntegerTopic("Entry Count").publish()
        self._entries: list[tuple[float, float, float]] = []
        self._feed_timer = Timer()

        # NT-tunable P gains per slot (shooter + kicker share same slots)
        pid_table = ntcore.NetworkTableInstance.getDefault().getTable("PID Tuning")

        self._shooter_p_low_sub = pid_table.getDoubleTopic("Shooter kP Low").subscribe(KP_LOW)
        self._shooter_p_mid_sub = pid_table.getDoubleTopic("Shooter kP Mid").subscribe(KP_MID)
        self._shooter_p_high_sub = pid_table.getDoubleTopic("Shooter kP High").subscribe(KP_HIGH)
        pid_table.getDoubleTopic("Shooter kP Low").publish().set(KP_LOW)
        pid_table.getDoubleTopic("Shooter kP Mid").publish().set(KP_MID)
        pid_table.getDoubleTopic("Shooter kP High").publish().set(KP_HIGH)

        # Track last applied values to avoid unnecessary CAN updates
        self._last_shooter_p = [KP_LOW, KP_MID, KP_HIGH]

    FEED_DELAY_S = 2.0

    def initialize(self) -> None:
        self._feed_timer.restart()

    def execute(self) -> None:
        self._check_pid_updates()

        target_rpm = self._rpm_sub.get()
        hood_pos = self._hood_sub.get()

        self.hood.set_target_position(hood_pos)
        self.shooter.set_target_speed(target_rpm)
        self.kicker.set_duty_cycle(1.0)

        feeding = self._feed_timer.hasElapsed(self.FEED_DELAY_S)
        if feeding:
            self.indexer.set_target_output(1.0)
        else:
            self.indexer.stop()

        current_rpm = self.shooter.get_current_speed()
        distance = self._get_hub_distance()
        self._distance_pub.set(distance)
        self._feeding_pub.set(feeding)
        self._shooter_ready_pub.set(abs(current_rpm - target_rpm) < SHOOTER_TOLERANCE_RPM)
        self._current_rpm_pub.set(current_rpm)
        self._rpm_error_pub.set(abs(current_rpm - target_rpm))

        # Publish entries + current live point
        entries = [f"({d:.2f}, {h:.1f}, {int(r)})" for d, h, r in self._entries]
        entries.append(f"({distance:.2f}, {hood_pos:.1f}, {int(target_rpm)})")
        self._entries_pub.set(entries)

    def end(self, interrupted: bool) -> None:
        self.shooter.stop()
        self.kicker.stop()
        self.indexer.stop()
        self.hood.stow()
        self._feeding_pub.set(False)
        self._shooter_ready_pub.set(False)

    def isFinished(self) -> bool:
        return False

    def _check_pid_updates(self) -> None:
        """Apply P gains from NT to motor controllers when they change."""
        shooter_p = [
            self._shooter_p_low_sub.get(),
            self._shooter_p_mid_sub.get(),
            self._shooter_p_high_sub.get(),
        ]
        slots = [rev.ClosedLoopSlot.kSlot0, rev.ClosedLoopSlot.kSlot1, rev.ClosedLoopSlot.kSlot2]
        for i, (new, old) in enumerate(zip(shooter_p, self._last_shooter_p)):
            if abs(new - old) > 1e-8:
                self.shooter.set_slot_p(slots[i], new)
                self._last_shooter_p[i] = new

    def _get_hub_distance(self) -> float:
        robot_pos = self._drivetrain.get_state().pose.translation()
        alliance = DriverStation.getAlliance()
        hub = RED_HUB if alliance == DriverStation.Alliance.kRed else BLUE_HUB
        return robot_pos.distance(hub)

    def record_entry(self) -> None:
        """Log current (distance, hood_turns, shooter_rpm) as a calibration point."""
        entry = (self._get_hub_distance(), self._hood_sub.get(), self._rpm_sub.get())
        self._entries.append(entry)
        self._entries_pub.set(
            [f"({d:.2f}, {h:.1f}, {int(r)})" for d, h, r in self._entries]
        )
        self._entry_count_pub.set(len(self._entries))

        # Append to CSV on roboRIO (creates with header if missing)
        write_header = not SHOT_LOG_PATH.exists()
        with open(SHOT_LOG_PATH, "a", newline="") as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(["distance_m", "hood_turns", "shooter_rpm"])
            writer.writerow([f"{entry[0]:.2f}", f"{entry[1]:.1f}", f"{int(entry[2])}"])
