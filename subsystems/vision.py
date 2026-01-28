import math

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
        if not isinstance(camera, str):
            raise TypeError("Camera name must be a string")

        self._swerve = swerve
        self._camera = camera

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

            # ## MT1
            # estimate_mt1 = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
            #     self._camera
            # )
            # if estimate is None or estimate.tag_count == 0:
            #     return
            # # Optional distance sanity check
            # if estimate.avg_tag_dist > 4.125:
            #     return

            # stddev_x, stddev_y, stddev_rot = self._get_dynamic_std_devs(estimate)

            # self._swerve.add_vision_measurement(
            #     estimate.pose,
            #     utils.fpga_to_current_time(estimate.timestamp_seconds),
            #     (99999, 99999, stddev_rot),
            # )

            # MT2
            estimate = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
                self._camera
            )

            if estimate is None or estimate.tag_count == 0:
                return

            # Optional distance sanity check
            if estimate.avg_tag_dist > 4.125:
                SmartDashboard.putBoolean("Vision/TooFar", True)
                return

            SmartDashboard.putBoolean("Vision/TooFar", False)
            
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
