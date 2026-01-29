import math
from typing import override

from phoenix6 import utils
from wpilib import DataLogManager, SmartDashboard

from modules.limelight import PoseEstimate, LimelightHelpers
from commands2 import Subsystem
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain

class VisionSubsystem(Subsystem):
    """
    Vision subsystem optimized for a single Limelight.
    No state-based tag filtering — always accepts all visible tags.
    """

    def __init__(self, swerve: CommandSwerveDrivetrain, camera: str):
        self._swerve: CommandSwerveDrivetrain = swerve
        self._camera:str   = camera
        SmartDashboard.putBoolean("Vision/UseMegaTag2", False)
        super().__init__()

    @override
    def periodic(self):
        super().periodic()

        # Reject vision during extreme rotation
        angular_velocity = self._swerve.pigeon2.get_angular_velocity_z_world().value
        if abs(angular_velocity) > 720:
            return

        try:
            state = self._swerve.get_state_copy()

            # Provide robot orientation for MegaTag2
            LimelightHelpers.set_robot_orientation(
                self._camera,
                state.pose.rotation().degrees(),
                0, 0, 0, 0, 0
            )

            # MT1
            estimate = None
            use_mt2 = SmartDashboard.getBoolean("Vision/UseMegaTag2", True)
            if use_mt2:
                # MT2
                estimate = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
                    self._camera
                )
            else:
                estimate = LimelightHelpers.get_botpose_estimate_wpiblue(
                    self._camera
                )

            # Rejection and Update
            if estimate is None or estimate.tag_count == 0:
                return

            # Optional distance sanity check
            if estimate.avg_tag_dist > 4.125:
                SmartDashboard.putBoolean("Vision/TooFar", True)
                return

            SmartDashboard.putBoolean("Vision/TooFar", False)
            
            if not use_mt2:
                self._swerve.seed_field_centric(estimate.pose.rotation())

            self._swerve.add_vision_measurement(
                estimate.pose,
                estimate.timestamp_seconds,
                self._get_dynamic_std_devs(estimate),
            )

        except Exception as e:
            DataLogManager.log(f"Vision processing failed: {e}")

    def set_throttle(self, throttle: int) -> None:
        LimelightHelpers.set_throttle(self._camera, throttle)

    @staticmethod
    def _get_dynamic_std_devs(estimate: PoseEstimate) -> tuple[float, float, float]:
        """
        Computes dynamic standard deviations based on tag count and distance.
        """
        if estimate.tag_count == 0:
            return 0.5, 0.5, 0.5

        avg_dist = (
            sum(f.dist_to_camera for f in estimate.raw_fiducials)
            / estimate.tag_count
        )

        factor = 0.9 + (avg_dist ** 2 / 30)

        return (
            0.5 * factor,
            0.5 * factor,
            math.inf if estimate.is_megatag_2 else 0.5 * factor,
        )
