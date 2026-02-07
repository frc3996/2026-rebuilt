
import commands2
from subsystems.shooter import ShooterSubSystem
from subsystems.hood import HoodSubSystem
from subsystems.indexer import IndexerSubSystem

class Shoot(commands2.Command):
    # Example command    
    def __init__(self, shooter_subsystem: ShooterSubSystem, ) -> None:
        # Initilization
        self.shooter_subsystem = shooter_subsystem
        self.addRequirements(self.shooter_subsystem)

    def initialize(self):
        print("START CLIMBING")
        self.climb_subsystem.setState(True)
    
    def execute(self):
        # Called every loop when the command is active
        pass

    def end(self, interrupted: bool):
        # Called when the command is done executing or interrupted
        self.climb_subsystem.setState(False)
        print("DONE CLIMBING")
        
    def isFinished(self) -> bool:
        # Should return True when the command is done 
        return False
