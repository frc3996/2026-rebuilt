import csv
from pathlib import Path

import ntcore
import rev
from commands2 import Command
from wpilib import DriverStation

from commands.shoot_at_hub import BLUE_HUB, RED_HUB
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
KICKER_TOLERANCE_RPM = 150
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
        self._rpm_sub = table.getDoubleTopic("Shooter RPM").subscribe(2000.0)
        self._hood_sub = table.getDoubleTopic("Hood Position").subscribe(3.45)

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
        # 0 = spinning up shooter, 1 = spinning up kicker, 2 = feeding
        self._stage: int = 0

        # NT-tunable P gains per slot (shooter + kicker share same slots)
        pid_table = ntcore.NetworkTableInstance.getDefault().getTable("PID Tuning")

        self._shooter_p_low_sub = pid_table.getDoubleTopic("Shooter kP Low").subscribe(KP_LOW)
        self._shooter_p_mid_sub = pid_table.getDoubleTopic("Shooter kP Mid").subscribe(KP_MID)
        self._shooter_p_high_sub = pid_table.getDoubleTopic("Shooter kP High").subscribe(KP_HIGH)
        pid_table.getDoubleTopic("Shooter kP Low").publish().set(KP_LOW)
        pid_table.getDoubleTopic("Shooter kP Mid").publish().set(KP_MID)
        pid_table.getDoubleTopic("Shooter kP High").publish().set(KP_HIGH)

        from subsystems.kicker import (
            LEFT_KP_HIGH,
            LEFT_KP_LOW,
            LEFT_KP_MID,
            RIGHT_KP_HIGH,
            RIGHT_KP_LOW,
            RIGHT_KP_MID,
        )
        self._kicker_r_low_sub = pid_table.getDoubleTopic("Kicker R kP Low").subscribe(RIGHT_KP_LOW)
        self._kicker_r_mid_sub = pid_table.getDoubleTopic("Kicker R kP Mid").subscribe(RIGHT_KP_MID)
        self._kicker_r_high_sub = pid_table.getDoubleTopic("Kicker R kP High").subscribe(RIGHT_KP_HIGH)
        pid_table.getDoubleTopic("Kicker R kP Low").publish().set(RIGHT_KP_LOW)
        pid_table.getDoubleTopic("Kicker R kP Mid").publish().set(RIGHT_KP_MID)
        pid_table.getDoubleTopic("Kicker R kP High").publish().set(RIGHT_KP_HIGH)

        self._kicker_l_low_sub = pid_table.getDoubleTopic("Kicker L kP Low").subscribe(LEFT_KP_LOW)
        self._kicker_l_mid_sub = pid_table.getDoubleTopic("Kicker L kP Mid").subscribe(LEFT_KP_MID)
        self._kicker_l_high_sub = pid_table.getDoubleTopic("Kicker L kP High").subscribe(LEFT_KP_HIGH)
        pid_table.getDoubleTopic("Kicker L kP Low").publish().set(LEFT_KP_LOW)
        pid_table.getDoubleTopic("Kicker L kP Mid").publish().set(LEFT_KP_MID)
        pid_table.getDoubleTopic("Kicker L kP High").publish().set(LEFT_KP_HIGH)

        # Track last applied values to avoid unnecessary CAN updates
        self._last_shooter_p = [KP_LOW, KP_MID, KP_HIGH]
        self._last_kicker_r_p = [RIGHT_KP_LOW, RIGHT_KP_MID, RIGHT_KP_HIGH]
        self._last_kicker_l_p = [LEFT_KP_LOW, LEFT_KP_MID, LEFT_KP_HIGH]

    def initialize(self) -> None:
        self._stage = 0

    def execute(self) -> None:
        self._check_pid_updates()

        target_rpm = self._rpm_sub.get()
        hood_pos = self._hood_sub.get()

        self.hood.set_target_position(hood_pos)
        self.shooter.set_target_speed(target_rpm)

        current_rpm = self.shooter.get_current_speed()
        shooter_ready = (
            current_rpm > target_rpm * 0.9
            and abs(current_rpm - target_rpm) < SHOOTER_TOLERANCE_RPM
        )

        # Stage 0 → 1: shooter at speed, start kicker
        if self._stage == 0 and shooter_ready:
            self._stage = 1

        # Stage 1 → 2: kicker at speed, start feeding
        if self._stage >= 1:
            self.kicker.set_target_speed(target_rpm)
            kicker_speed = self.kicker.get_current_speed()
            kicker_ready = abs(kicker_speed - target_rpm) < KICKER_TOLERANCE_RPM
            if self._stage == 1 and kicker_ready:
                self._stage = 2
        else:
            self.kicker.stop()

        if self._stage >= 2:
            self.indexer.set_target_output(1.0)
        else:
            self.indexer.stop()

        distance = self._get_hub_distance()
        self._distance_pub.set(distance)
        self._feeding_pub.set(self._stage >= 2)
        self._shooter_ready_pub.set(shooter_ready)
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
        self._stage = 0

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

        kicker_r_p = [
            self._kicker_r_low_sub.get(),
            self._kicker_r_mid_sub.get(),
            self._kicker_r_high_sub.get(),
        ]
        for i, (new, old) in enumerate(zip(kicker_r_p, self._last_kicker_r_p)):
            if abs(new - old) > 1e-8:
                self.kicker.set_right_slot_p(slots[i], new)
                self._last_kicker_r_p[i] = new

        kicker_l_p = [
            self._kicker_l_low_sub.get(),
            self._kicker_l_mid_sub.get(),
            self._kicker_l_high_sub.get(),
        ]
        for i, (new, old) in enumerate(zip(kicker_l_p, self._last_kicker_l_p)):
            if abs(new - old) > 1e-8:
                self.kicker.set_left_slot_p(slots[i], new)
                self._last_kicker_l_p[i] = new

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
