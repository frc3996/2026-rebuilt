import commands2
from subsystems.climb import ClimbSubsystem


class ExtendClimb(commands2.InstantCommand):
    def __init__(self, climb_subsystem: ClimbSubsystem) -> None:
        super().__init__(lambda: climb_subsystem.setState(True), climb_subsystem)


class RetractClimb(commands2.InstantCommand):
    def __init__(self, climb_subsystem: ClimbSubsystem) -> None:
        super().__init__(lambda: climb_subsystem.setState(False), climb_subsystem)
