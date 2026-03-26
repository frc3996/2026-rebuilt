import math
from typing import override

import ntcore
from commands2 import Subsystem
from wpilib import DataLogManager

from modules.limelight import LimelightHelpers, PoseEstimate
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain

LIMELIGHT_CAMERA_NAME = "limelight-front"
VISION_MAX_TAG_DISTANCE = 4.125  # meters — reject estimates beyond this
VISION_MAX_ANGULAR_VELOCITY = 720  # deg/s — reject estimates during fast rotation


class VisionSubsystem(Subsystem):
    """
    Vision subsystem optimized for a single Limelight.
    No state-based tag filtering — always accepts all visible tags.
    """

    def __init__(self, swerve: CommandSwerveDrivetrain, camera: str):
        self._swerve: CommandSwerveDrivetrain = swerve
        self._camera: str = camera
        self._use_megatag2 = False

        table = ntcore.NetworkTableInstance.getDefault().getTable("Vision")
        self._use_mt2_sub = table.getBooleanTopic("UseMegaTag2").subscribe(False)
        self._use_mt2_pub = table.getBooleanTopic("UseMegaTag2").publish()
        self._use_mt2_pub.set(False)
        self._too_far_pub = table.getBooleanTopic("TooFar").publish()

        super().__init__()

    @override
    def periodic(self):
        # Reject vision during extreme rotation
        angular_velocity = self._swerve.pigeon2.get_angular_velocity_z_world().value
        if abs(angular_velocity) > VISION_MAX_ANGULAR_VELOCITY:
            return

        try:
            state = self._swerve.get_state_copy()

            # Provide robot orientation for MegaTag2 — no_flush avoids blocking NT round-trip
            LimelightHelpers.set_robot_orientation_no_flush(
                self._camera, state.pose.rotation().degrees(), 0, 0, 0, 0, 0
            )

            if self._use_megatag2:
                estimate = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
                    self._camera
                )
            else:
                estimate = LimelightHelpers.get_botpose_estimate_wpiblue(self._camera)

            # Rejection and Update
            if estimate is None or estimate.tag_count == 0:
                return

            if estimate.avg_tag_dist > VISION_MAX_TAG_DISTANCE:
                self._too_far_pub.set(True)
                return

            self._too_far_pub.set(False)

            if not self._use_megatag2:
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
            sum(f.dist_to_camera for f in estimate.raw_fiducials) / estimate.tag_count
        )

        factor = 0.9 + (avg_dist**2 / 30)

        return (
            0.5 * factor,
            0.5 * factor,
            math.inf if estimate.is_megatag_2 else 0.5 * factor,
        )
