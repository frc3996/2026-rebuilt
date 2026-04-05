from commands2 import Command
from wpilib import Timer

from subsystems.indexer import IndexerSubSystem
from subsystems.kicker import KickerSubSystem
from subsystems.shooter import ShooterSubSystem

CLEAROUT_S = 0.25


class Clearout(Command):
    """Reverse conveyor while keeping shooter+kicker running, then stop."""

    def __init__(
        self,
        shooter: ShooterSubSystem,
        kicker: KickerSubSystem,
        indexer: IndexerSubSystem,
        shooter_rpm: float = 0,
    ) -> None:
        super().__init__()
        self.shooter = shooter
        self.kicker = kicker
        self.indexer = indexer
        self._rpm = shooter_rpm
        self.addRequirements(shooter, kicker, indexer)
        self._timer = Timer()

    def initialize(self):
        self._timer.restart()

    def execute(self):
        self.shooter.set_target_speed(self._rpm)
        self.kicker.set_duty_cycle(1.0)
        self.indexer.set_target_output(-1.0)

    def end(self, interrupted: bool):
        self.shooter.stop()
        self.kicker.stop()
        self.indexer.stop()

    def isFinished(self) -> bool:
        return self._timer.hasElapsed(CLEAROUT_S)
