
import commands2
from subsystems.climb import ClimbSubsystem

class ExtendClimb(commands2.Command):
    # Example command    
    def __init__(self, climb_subsystem: ClimbSubsystem) -> None:
        # Initilization
        self.climb_subsystem = climb_subsystem
        self.addRequirements(self.climb_subsystem)

    def initialize(self):
        self.climb_subsystem.setState(True)
    
    def execute(self):
        # Called every loop when the command is active
        pass

    def end(self, interrupted: bool):
        # Called when the command is done executing or interrupted
        pass
        
    def isFinished(self) -> bool:
        # Should return True when the command is done 
        return True

class RetractClimb(commands2.Command):
    # Example command    
    def __init__(self, climb_subsystem: ClimbSubsystem) -> None:
        # Initilization
        self.climb_subsystem = climb_subsystem
        self.addRequirements(self.climb_subsystem)

    def initialize(self):
        self.climb_subsystem.setState(False)
    
    def execute(self):
        # Called every loop when the command is active
        pass

    def end(self, interrupted: bool):
        # Called when the command is done executing or interrupted
        pass
        
    def isFinished(self) -> bool:
        # Should return True when the command is done 
        return True

