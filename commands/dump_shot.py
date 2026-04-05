"""
Dump shot command - shoots without auto-aiming, hood lifted high for close-range shots.
Unlike HubShot, this does not change robot orientation and uses fixed shooter settings.
"""

import ntcore
from commands2 import Command
from wpilib import Timer


class DumpShot(Command):
    """
    Command that shoots at a fixed RPM and hood angle without changing robot orientation.
    Useful for close-range shots or dumping into the hub when already aligned.

    Does NOT use VirtualGoal or auto-aiming - just spins up and feeds.
    """

    # Fixed shooting parameters for dump shot
    DUMP_RPM = 1800.0  # Lower RPM for close range
    DUMP_HOOD_POSITION = 1.0  # High hood angle (close to max)
    FEED_DELAY_S = 1.0  # Shorter delay since no need to wait for precise aim

    def __init__(self, shooter, kicker, indexer, hood):
        super().__init__()
        self.shooter = shooter
        self.kicker = kicker
        self.indexer = indexer
        self.hood = hood

        self.addRequirements(shooter, kicker, indexer, hood)

        self._feed_timer = Timer()

        # NetworkTables for monitoring
        table = ntcore.NetworkTableInstance.getDefault().getTable("DumpShot")
        self._target_rpm_pub = table.getDoubleTopic("Target RPM").publish()
        self._current_rpm_pub = table.getDoubleTopic("Current RPM").publish()
        self._rpm_error_pub = table.getDoubleTopic("RPM Error").publish()
        self._target_hood_pub = table.getDoubleTopic("Target Hood").publish()
        self._feeding_pub = table.getBooleanTopic("Feeding").publish()

        # NT-tunable parameters
        self._rpm_pub = table.getDoubleTopic("RPM").publish()
        self._rpm_pub.set(self.DUMP_RPM)
        self._rpm_sub = table.getDoubleTopic("RPM").subscribe(self.DUMP_RPM)

        self._hood_pub = table.getDoubleTopic("Hood Position").publish()
        self._hood_pub.set(self.DUMP_HOOD_POSITION)
        self._hood_sub = table.getDoubleTopic("Hood Position").subscribe(self.DUMP_HOOD_POSITION)

        self._kicker_full_pub = table.getBooleanTopic("Kicker Full Speed").publish()
        self._kicker_full_pub.set(True)
        self._kicker_full_sub = table.getBooleanTopic("Kicker Full Speed").subscribe(True)

    def initialize(self):
        """Start the feed timer when command begins."""
        self._feed_timer.restart()

    def execute(self):
        """Run shooter, kicker, hood at fixed settings, then feed after delay."""
        # Get NT-tunable parameters
        target_rpm = self._rpm_sub.get()
        target_hood = self._hood_sub.get()

        # Set shooter and hood to fixed positions
        self.shooter.set_target_speed(target_rpm)
        self.hood.set_target_position(target_hood)

        # Kicker can run at full speed or match shooter
        if self._kicker_full_sub.get():
            self.kicker.set_duty_cycle(1.0)
        else:
            self.kicker.set_target_speed(target_rpm)

        # Start feeding after delay
        if self._feed_timer.hasElapsed(self.FEED_DELAY_S):
            self.indexer.set_target_output(1.0)
        else:
            self.indexer.stop()

        # Telemetry
        current_rpm = self.shooter.get_current_speed()
        feeding = self._feed_timer.hasElapsed(self.FEED_DELAY_S)
        self._target_rpm_pub.set(target_rpm)
        self._current_rpm_pub.set(current_rpm)
        self._rpm_error_pub.set(abs(current_rpm - target_rpm))
        self._target_hood_pub.set(target_hood)
        self._feeding_pub.set(feeding)

    def end(self, interrupted: bool):
        """Stop all motors when command ends."""
        self.shooter.stop()
        self.kicker.stop()
        self.indexer.stop()
        self.hood.stow()
        self._feeding_pub.set(False)

    def isFinished(self) -> bool:
        """Run until manually interrupted."""
        return False
