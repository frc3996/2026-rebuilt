import commands2


class RunClimb(commands2.RepeatCommand):
    
    def __init__(self) -> None:
        print("STARTING CLIMB SEQUENCE")
        super().__init__()

    def periodic(self):
        print("bob")
        
    def isFinished(self) -> bool:
        print("DONE CLIMB SEQUENCE")
        # End when the controller is at the reference.
        return True