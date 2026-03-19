
import commands2
from subsystems.shooter import ShooterSubSystem
from subsystems.kicker import KickerSubSystem
from subsystems.indexer import IndexerSubSystem

class Shoot(commands2.Command):
    """
    Basic shoot command for testing — runs shooter, kicker, and conveyor
    at fixed speeds without ballistics or auto-aim.
    """
    TEST_SHOOTER_RPM = 3000
    TEST_KICKER_RPM = 3000
    TEST_CONVEYOR_OUTPUT = 0.5  # Duty cycle (-1.0 to 1.0)

    def __init__(self, shooter: ShooterSubSystem, kicker: KickerSubSystem, indexer: IndexerSubSystem) -> None:
        self.shooter = shooter
        self.kicker = kicker
        self.indexer = indexer
        self.addRequirements(self.shooter, self.kicker, self.indexer)

    def initialize(self):
        self.shooter.set_target_speed(self.TEST_SHOOTER_RPM)
        self.kicker.set_target_speed(self.TEST_KICKER_RPM)
        self.indexer.set_target_output(self.TEST_CONVEYOR_OUTPUT)

    def execute(self):
        pass

    def end(self, interrupted: bool):
        self.shooter.stop()
        self.kicker.stop()
        self.indexer.stop()

    def isFinished(self) -> bool:
        return False
