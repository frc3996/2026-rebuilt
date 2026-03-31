import math
from typing import override

import ntcore
from commands2 import Subsystem
from wpilib import DataLogManager

from modules.limelight import LimelightHelpers, PoseEstimate
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain

CAMERAS = ["limelight-front", "limelight-back"]
VISION_MAX_TAG_DISTANCE = 4.125  # meters — reject estimates beyond this
VISION_MAX_ANGULAR_VELOCITY = 720  # deg/s — reject estimates during fast rotation


class VisionSubsystem(Subsystem):
    """
    Dual-Limelight vision subsystem for AprilTag pose estimation.

    Uses MT1 while disabled (full 6-DOF solve for initial localization),
    switches to MT2 during auto/teleop (gyro-constrained, more stable while moving).
    Each camera's estimate is fed independently to the pose estimator with
    distance-scaled standard deviations.
    """

    def __init__(self, swerve: CommandSwerveDrivetrain, cameras: list[str]):
        self._swerve: CommandSwerveDrivetrain = swerve
        self._cameras = cameras
        self._use_megatag2 = False

        table = ntcore.NetworkTableInstance.getDefault().getTable("Vision")
        self._too_far_pubs = {
            cam: table.getBooleanTopic(f"{cam}/TooFar").publish() for cam in cameras
        }
        self._tag_count_pubs = {
            cam: table.getIntegerTopic(f"{cam}/TagCount").publish() for cam in cameras
        }

        super().__init__()

    @override
    def periodic(self):
        angular_velocity = self._swerve.pigeon2.get_angular_velocity_z_world().value
        if abs(angular_velocity) > VISION_MAX_ANGULAR_VELOCITY:
            return

        try:
            state = self._swerve.get_state_copy()
            heading_deg = state.pose.rotation().degrees()

            for cam in self._cameras:
                self._process_camera(cam, heading_deg)

        except Exception as e:
            DataLogManager.log(f"Vision processing failed: {e}")

    def _process_camera(self, cam: str, heading_deg: float) -> None:
        if self._use_megatag2:
            LimelightHelpers.set_robot_orientation_no_flush(
                cam, heading_deg, 0, 0, 0, 0, 0
            )
            estimate = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(cam)
        else:
            estimate = LimelightHelpers.get_botpose_estimate_wpiblue(cam)

        if estimate is None or estimate.tag_count == 0:
            self._tag_count_pubs[cam].set(0)
            return

        self._tag_count_pubs[cam].set(estimate.tag_count)

        if estimate.avg_tag_dist > VISION_MAX_TAG_DISTANCE:
            self._too_far_pubs[cam].set(True)
            return

        self._too_far_pubs[cam].set(False)

        self._swerve.add_vision_measurement(
            estimate.pose,
            estimate.timestamp_seconds,
            self._get_dynamic_std_devs(estimate),
        )

    def set_throttle(self, throttle: int) -> None:
        for cam in self._cameras:
            LimelightHelpers.set_throttle(cam, throttle)

    @staticmethod
    def _get_dynamic_std_devs(estimate: PoseEstimate) -> tuple[float, float, float]:
        """Scale trust by tag count and distance. More tags / closer = tighter std devs."""
        avg_dist = (
            sum(f.dist_to_camera for f in estimate.raw_fiducials) / estimate.tag_count
        )

        if estimate.tag_count >= 2:
            factor = 0.5 + (avg_dist**2 / 40)
        else:
            factor = 0.9 + (avg_dist**2 / 30)

        return (
            0.5 * factor,
            0.5 * factor,
            math.inf if estimate.is_megatag_2 else 0.5 * factor,
        )
