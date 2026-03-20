import commands2
from wpilib import Timer
from subsystems.intake import (
    IntakeSubSystem,
    HOMING_VOLTAGE,
    STALL_CURRENT_THRESHOLD,
    STALL_VELOCITY_THRESHOLD,
    STALL_CONFIRM_CYCLES,
    HOMING_TIMEOUT_SECONDS,
)


class HomeIntake(commands2.Command):
    """
    Drives the intake arm toward its deployed hard stop using constant voltage.
    Detects stall via current + velocity thresholds held for multiple
    consecutive loops. On confirmed stall, zeros the encoder and marks homed.
    """

    def __init__(self, intake: IntakeSubSystem) -> None:
        super().__init__()
        self.intake = intake
        self.addRequirements(self.intake)
        self._stall_counter: int = 0
        self._timer = Timer()
        self._timed_out: bool = False

    def initialize(self) -> None:
        self._stall_counter = 0
        self._timed_out = False
        self._timer.restart()
        self.intake.disable_soft_limits()
        self.intake.up_down_motor.setVoltage(HOMING_VOLTAGE)
        print("[Intake] Homing started")

    def execute(self) -> None:
        current = self.intake.up_down_motor.getOutputCurrent()
        velocity = abs(self.intake.up_down_encoder.getVelocity())

        if current > STALL_CURRENT_THRESHOLD and velocity < STALL_VELOCITY_THRESHOLD:
            self._stall_counter += 1
        else:
            self._stall_counter = 0

        if self._timer.hasElapsed(HOMING_TIMEOUT_SECONDS):
            self._timed_out = True
            print("[Intake] Homing timed out — aborting")

    def end(self, interrupted: bool) -> None:
        self.intake.up_down_motor.set(0)
        if not interrupted and not self._timed_out:
            self.intake.up_down_encoder.setPosition(0.0)
            self.intake.homed = True
            self.intake.enable_soft_limits()
            print("[Intake] Stall confirmed — encoder zeroed, homing complete")
        else:
            print("[Intake] Homing did not complete — encoder NOT zeroed")

    def isFinished(self) -> bool:
        if self._timed_out:
            return True
        return self._stall_counter >= STALL_CONFIRM_CYCLES
