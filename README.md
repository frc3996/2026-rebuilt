# 2026 Rebuild
Code basé sur CTRE Swerve with Pathplannerlib


# CommandV2 Example


## ./subsystems/sub_system_name.py
```python

from commands2 import Subsystem, Command
from wpilib.shuffleboard import Shuffleboard

class SubSystemName(Subsystem):
    """
    Demo Subsystem
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
        pass

```


## ./commands/command_name.py
```python

import commands2

class MyCommand(commands2.Command):
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



# Components

## Swerve
Finnish

## Vision
Finnish

## Shooter
- moteur_gauche_1
- moteur_gauche_2
- moteur_droite_1
- moteur_droite_2

## Hood
- moteur

## Indexeur
- moteur_gauche
- moteur_droite
- moteur_convoyeur

## Climb
- Solenoide

## Intake
- moteur_descendre
- moteur_intake



# Commandes
## Intake
- Sort l'intake
- Intake
- Stop quand on lache le bouton (remonte pas)

## Climb [Gauche, Droite]
- Sort piston
- Run path
- Stop quand on lache le bouton et piston remonte

## Drive Intake Forward
- Robot roule avec l'intake pointe dans sa direction de deplacement

## PreSpin
- Accelere le shooter a la bonne vitesse
- Stop quand bouton lache

## Shoot [Gauche, Droite, Hub]
- Robot vise la cible
- Ajuste hood
- Quand vitesse bonne, shoot
- Stop quand on lache boutton
