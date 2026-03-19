import commands2
from subsystems.intake import IntakeSubSystem


class HomeIntake(commands2.Command):
    """
    Drives the intake arm toward a hard stop using current control.
    Once the motor stalls (velocity near zero), resets the encoder to 0
    and finishes. After this command completes, position control is valid.
    """

    def __init__(self, intake: IntakeSubSystem):
        super().__init__()
        self.intake = intake
        self.addRequirements(self.intake)

    def initialize(self):
        self.intake.disable_soft_limits()
        self.intake.set_up_down_target_amp(IntakeSubSystem.HOMING_AMPS)

    def execute(self):
        pass

    def end(self, interrupted: bool):
        self.intake.stop()
        if not interrupted:
            self.intake.reset_encoder()
            self.intake.enable_soft_limits()

    def isFinished(self) -> bool:
        return self.intake.is_stalled()
