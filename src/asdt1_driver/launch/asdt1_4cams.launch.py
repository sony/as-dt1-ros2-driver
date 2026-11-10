"""Launch all AS-DT1 cameras listed in a calibration file.

One driver is started per camera entry in the calibration file, all in a
single component container together with a point cloud combiner that
merges them into /combiner/point_cloud and /combiner/imu. The default
calibration file, calibration_example.yml, lists four cameras.

Each camera's namespace is derived from its ``description`` (e.g.
"Top Left" -> asdt1_top_left). Its ``sync_mode`` is taken from the file,
defaulting to slave when an entry does not set it.

Examples:
  ros2 launch asdt1_ros2_driver asdt1_4cams.launch.py
  ros2 launch asdt1_ros2_driver asdt1_4cams.launch.py calibration_file:=/path/to/calibration.yml
  ros2 launch asdt1_ros2_driver asdt1_4cams.launch.py output_format:=XYZITR
"""

import re

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def _namespace_from_description(description):
    desc = description.strip().lower()
    return re.sub(r"[^a-z0-9]+", "_", desc).strip("_")


def _load_calibrated_cameras(calibration_file):
    with open(calibration_file, "r", encoding="utf-8") as file:
        calibration = yaml.safe_load(file) or {}

    cameras = calibration.get("cameras", [])
    if not isinstance(cameras, list):
        raise RuntimeError(f"Calibration file '{calibration_file}' does not contain a valid cameras list")

    calibrated_cameras = []
    for idx, camera in enumerate(cameras):
        serial_value = camera.get("id")
        description = str(camera.get("description", "")).strip()
        if serial_value is None or not description:
            raise RuntimeError(
                f"Camera entry {idx} in '{calibration_file}' must contain both id and description"
            )
        try:
            serial = int(serial_value)
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                f"Camera entry {idx} in '{calibration_file}' must contain an integer id"
            ) from exc

        calibrated_cameras.append(
            {
                "serial": serial,
                "description": description,
                "namespace": f"asdt1_{_namespace_from_description(description)}",
                "sync_mode": camera.get("sync_mode", "slave"),
            }
        )

    if not calibrated_cameras:
        raise RuntimeError(f"Calibration file '{calibration_file}' does not contain any cameras")

    return calibrated_cameras


def _launch_setup(context, *args, **kwargs):
    calibration_file = LaunchConfiguration("calibration_file").perform(context)
    output_format = LaunchConfiguration("output_format").perform(context)
    calibrated_cameras = _load_calibrated_cameras(calibration_file)

    driver_components = []
    for camera in calibrated_cameras:
        driver_components.append(
            ComposableNode(
                package="asdt1_ros2_driver",
                plugin="ASDT1_Driver",
                name="asdt1_driver",
                namespace=camera["namespace"],
                parameters=[
                    {
                        "serial_number": camera["serial"],
                        "calibration_file": calibration_file,
                        "output_format": output_format,
                        "sync_mode": camera["sync_mode"],
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

    driver_components.append(
        ComposableNode(
            package="asdt1_ros2_driver",
            plugin="PointCloudCombiner",
            name="point_cloud_combiner",
            namespace="combiner",
            parameters=[
                {
                    "input_topics": [
                        f"/{camera['namespace']}/asdt1_driver/point_cloud"
                        for camera in calibrated_cameras
                    ],
                    "imu_topics": [
                        f"/{camera['namespace']}/asdt1_driver/imu"
                        for camera in calibrated_cameras
                    ],
                    "output_topic": "point_cloud",
                    "output_imu_topic": "imu",
                    "target_frame": "camera_optical",
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    )

    return [
        ComposableNodeContainer(
            name="asdt1_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=driver_components,
            output="both",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "calibration_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("asdt1_ros2_driver"), "config", "calibration_example.yml"]
                ),
                description="Path to camera calibration YAML file",
            ),
            DeclareLaunchArgument(
                "output_format",
                default_value="XYZ",
                description="Point cloud format: XYZ or XYZITR",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
