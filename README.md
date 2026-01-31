# 2026 Rebuild
Code basé sur CTRE Swerve with Pathplannerlib


# CommandV2 Example


## ./subsystems/sub_system_name.py
```python

from commands2 import Subsystem, Command
from wpilib.shuffleboard import Shuffleboard

class SubSystemName(Subsystem):
    """
    Subsystem Demo
    """
    def __init__(self):
        # Initilization
        tab = Shuffleboard.getTab("SubSystemName")
        tab.add("Demo Component", "can be a wpilib component, such as wpilib.solenoid")
        tab.addDouble("Variable name [Unit]", lambda: self.blabla())
        pass

    def periodic(self):
        # Called on every loop
        pass

    def simulationPeriodic(self):
        # Called on every simulation loop

```


## ./commands/command_name.py
```python

import commands2

class RunClimb(commands2.Command):
    # Example command    
    def __init__(self, speed) -> None:
        # Initilization
        pass
    
    def initialize(self):
        # Called once when the command is started.

    def execute(self):
        # Called every loop when the command is active
        pass

    def end(self, interrupted: bool):
        # Called once when the command is done executing or interrupted
        pass
        
    def isFinished(self) -> bool:
        # Should return True when the command is done 
        return False

```


## robotcontainer.py
```python
class RobotContainer:
    def __init__(self) -> None:
        # Declare the sybsystem here. The subsystems can then be passed
        # to the commands

    def configureButtonBindings(self) -> None:
        self._joystick.a().whileTrue(COMMAND_TO_CALL())
        # OR
        commands2.button.JoystickButton(self._joystick, 1).whileTrue(COMMAND_TO_CALL)

```
