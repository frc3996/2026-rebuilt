
from commands2 import Subsystem, Command

class Climb(Subsystem):
    """
    Vision subsystem optimized for a single Limelight.
    No state-based tag filtering — always accepts all visible tags.
    """

    def __init__(self):
        super().__init__()

    def periodic(self):
        pass

    def demo_idle(self) -> Command:
        print("idling")
        return False

    def run_climb(self) -> Command:
        print("Climbing!")

    def climb_done(self) -> bool:
        return False
