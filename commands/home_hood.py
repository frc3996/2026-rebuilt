import commands2
from subsystems.hood import HoodSubSystem


class HomeHood(commands2.Command):
    """
    Drives the hood toward a hard stop using current control.
    Once the motor stalls (velocity near zero), resets the encoder to 0
    and finishes. After this command completes, position control is valid.
    """

    def __init__(self, hood: HoodSubSystem):
        super().__init__()
        self.hood = hood
        self.addRequirements(self.hood)

    def initialize(self):
        self.hood.disable_soft_limits()
        self.hood.set_target_amps(HoodSubSystem.HOMING_AMPS)

    def execute(self):
        pass

    def end(self, interrupted: bool):
        self.hood.stop()
        if not interrupted:
            self.hood.reset_encoder()
            self.hood.enable_soft_limits()

    def isFinished(self) -> bool:
        return self.hood.is_stalled()
