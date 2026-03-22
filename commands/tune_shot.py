import csv
from pathlib import Path

import ntcore
from commands2 import Command
from wpilib import DriverStation

from commands.shoot_at_hub import BLUE_HUB, RED_HUB
from subsystems.hood import HoodSubSystem
from subsystems.indexer import IndexerSubSystem
from subsystems.kicker import KickerSubSystem
from subsystems.shooter import ShooterSubSystem

SHOOTER_TOLERANCE_RPM = 200
KICKER_TOLERANCE_RPM = 500
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
        self.drivetrain = drivetrain
        self.addRequirements(shooter, kicker, indexer, hood)

        table = ntcore.NetworkTableInstance.getDefault().getTable("Manual")

        # Tunable inputs (shared with configureManualBindings POV controls)
        self._rpm_sub = table.getDoubleTopic("Shooter RPM").subscribe(4000.0)
        self._hood_sub = table.getDoubleTopic("Hood Position").subscribe(2.0)

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

    def execute(self) -> None:
        target_rpm = self._rpm_sub.get()
        hood_pos = self._hood_sub.get()

        self.hood.set_target_position(hood_pos)
        self.shooter.set_target_speed(target_rpm)

        current_rpm = self.shooter.get_current_speed()
        shooter_ready = abs(current_rpm - target_rpm) < SHOOTER_TOLERANCE_RPM

        if shooter_ready:
            self.kicker.set_target_speed(target_rpm)
        else:
            self.kicker.stop()

        kicker_ready = abs(self.kicker.get_current_speed() - target_rpm) < KICKER_TOLERANCE_RPM
        feeding = shooter_ready and kicker_ready
        if feeding:
            self.indexer.set_target_output(1.0)
        else:
            self.indexer.stop()

        self._distance_pub.set(self._get_hub_distance())
        self._feeding_pub.set(feeding)
        self._shooter_ready_pub.set(shooter_ready)
        self._current_rpm_pub.set(current_rpm)
        self._rpm_error_pub.set(abs(current_rpm - target_rpm))

    def end(self, interrupted: bool) -> None:
        self.shooter.stop()
        self.kicker.stop()
        self.indexer.stop()
        self.hood.stow()
        self._feeding_pub.set(False)
        self._shooter_ready_pub.set(False)

    def isFinished(self) -> bool:
        return False

    def _get_hub_distance(self) -> float:
        """Distance from robot to the correct alliance hub."""
        robot_pos = self.drivetrain.get_state().pose.translation()
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
