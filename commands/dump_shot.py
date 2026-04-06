from commands2 import Command
from wpilib import Timer

from subsystems.hood import HoodSubSystem
from subsystems.indexer import IndexerSubSystem
from subsystems.kicker import KickerSubSystem
from subsystems.shooter import ShooterSubSystem

DUMP_RPM = 2800
FEED_DELAY_S = 2.0


class DumpShot(Command):
    """Fixed-parameter shot: hood at max position, shooter at 2800 RPM.

    On end, schedules a clearout that reverses the conveyor
    while keeping shooter+kicker running.
    """

    def __init__(
        self,
        shooter: ShooterSubSystem,
        kicker: KickerSubSystem,
        indexer: IndexerSubSystem,
        hood: HoodSubSystem,
    ) -> None:
        super().__init__()
        self.shooter = shooter
        self.kicker = kicker
        self.indexer = indexer
        self.hood = hood
        self.addRequirements(shooter, kicker, indexer, hood)
        self._feed_timer = Timer()

    def initialize(self):
        self._feed_timer.restart()

    def execute(self):
        self.hood.set_target_position(self.hood.max_rotations)
        self.shooter.set_target_speed(DUMP_RPM)
        self.kicker.set_duty_cycle(1.0)

        if self._feed_timer.hasElapsed(FEED_DELAY_S):
            self.indexer.set_target_output(1.0)
        else:
            self.indexer.stop()

    def end(self, interrupted: bool):
        self.hood.stow()

    def isFinished(self) -> bool:
        return False
