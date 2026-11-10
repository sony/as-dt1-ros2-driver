# AS-DT1 ROS2 User Manual

This document describes how to use the `asdt1_driver` ROS 2 node in a ROS2 environment.
It assumes the code has been built and ROS2 is installed on the PC.
The driver has been tested on ROS2 Humble, Jazzy and Lyrical.

## Overview

The package, named asdt1_ros2_driver, contains two ROS 2 nodes:

- `asdt1_driver`: Main driver node for one AS-DT1 device.
- `point_cloud_combiner`: Utility node for combining multiple point clouds into a single point cloud.

This manual focuses on `asdt1_driver`, since that is the node that talks to the AS-DT1 over serial and publishes the sensor data.


## Prerequisites

Give permission to open the TTY port of the AS-DT1 by adding user to the dialout group.

```bash
sudo usermod -a -G dialout $USER
```

Restart for the change to take effect.

Source the workspace.

```bash
source /opt/ros/<distro>/setup.bash  # e.g. humble, jazzy or lyrical
source install/setup.bash
```

## Starting the Node

If an AS-DT1 has not been connected to host when starting the driver, it will continue looking for an AS-DT1 until it is connected.

### Start without specifying a serial number

If `serial_number` is not set (or set to `0` or less), the node will scan serial ports and connect to the first AS-DT1 it finds.

```bash
ros2 run asdt1_ros2_driver asdt1_driver
```

Equivalent explicit form:

```bash
ros2 run asdt1_ros2_driver asdt1_driver --ros-args -p serial_number:=-1
```

### Start with a specific serial number

Use this when more than one AS-DT1 may be connected and a specific AS-DT1 is wanted.

```bash
ros2 run asdt1_ros2_driver asdt1_driver --ros-args -p serial_number:=4200059
```

### Start with a specific calibration file

```bash
ros2 run asdt1_ros2_driver asdt1_driver --ros-args \
  -p serial_number:=4200059 \
  -p calibration_file:=/absolute/path/to/calibration.yml
```


### Start multiple cameras with the provided launch file

The package includes an example launch file [`asdt1_4cams.launch.py`](../src/asdt1_driver/launch/asdt1_4cams.launch.py), which starts one driver instance per camera in the calibration file, plus a point cloud combiner. With the default `calibration_example.yml` this gives four cameras with these namespaces:

- `asdt1_top_left`
- `asdt1_top_right`
- `asdt1_bottom_left`
- `asdt1_bottom_right`

Run it with:

```bash
ros2 launch asdt1_ros2_driver asdt1_4cams.launch.py
```

Optionally override the calibration file:

```bash
ros2 launch asdt1_ros2_driver asdt1_4cams.launch.py \
  calibration_file:=/absolute/path/to/calibration.yml
```
<div class="page"></div>

## Parameters

### Startup parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `calibration_file` | string | `config/calibration.yml` | YAML file with camera extrinsics and optional frame IDs. |
| `serial_number` | integer | `-1` | Hardware serial to connect to. `0` or less means "connect to any AS-DT1 found". |
| `output_format` | string | `XYZ` | Point cloud format published on the `point_cloud` topic: `XYZ` or `XYZITR`. |
| `mode` | string | `30MSTD` | Sensor mode. |
| `framerate` | integer | `30` | Frame rate in Hz, from `1` to `30`. |
| `sync_mode` | string | `none` | Synchronization mode: `none`, `master`, or `slave`. |


### Select point cloud output format

Use `output_format` to choose which point cloud payload the driver publishes:

- `XYZ`: `x`, `y`, `z` (default)
 - xyz: The coordinates in m.
- `XYZITR`: `x`, `y`, `z`, `intensity`, `time`, `ring`
 - xyz: The coordinates in m.
 - intensity: Intensity value for every dot.
 - time: Time offset from start of frame timestamp in ms for every dot.
 - ring: Horizontal line number for the dot.

Note that setting XYZITR mode means the camera is set in histogram output mode, which increases output data rate and power consumption and should hence only be used if the additional output data is needed.

Example:

```bash
ros2 run asdt1_ros2_driver asdt1_driver --ros-args -p output_format:=XYZITR
```
<div class="page"></div>

### Supported `mode` values

The driver accepts the following mode strings:

| Mode | Meaning |
| --- | --- |
| `20M` | Short-range measurements |
| `30MSTD` | General-purpose measurements |
| `30M15F` | Outdoor distance measurements, less affected by sunlight |
| `30M30F` | Outdoor distance measurements with reduced points for faster operation |
| `40M` | Indoor distance measurements, more affected by sunlight |

Mode matching is case-insensitive in the driver.

### Runtime parameter changes

The node installs a parameter callback at startup.

- `calibration_file` cannot be changed at runtime.
- `serial_number` cannot be changed at runtime.
- `mode` can be changed at runtime. A successful change triggers a device reboot.
- `framerate` can be changed at runtime. A successful change triggers a device reboot.
- `sync_mode` can be changed at runtime. A successful change triggers a device reboot.

Example:

```bash
ros2 param set /asdt1_driver mode 20M
ros2 param set /asdt1_driver framerate 15
ros2 param set /asdt1_driver sync_mode master
```

If the node is running in a namespace, use that fully qualified node path instead, for example:

```bash
ros2 param set /asdt1_top_left/asdt1_driver mode 20M
```

<div class="page"></div>

## Published Topics

The driver does not subscribe to any topics. It publishes the following:

| Topic | Type | Default frame ID | Notes |
| --- | --- | --- | --- |
| `asdt1_driver/point_cloud` | `sensor_msgs/msg/PointCloud2` | `asdt1_optical` | Point cloud fields depend on `output_format`: `XYZ` publishes `x`, `y`, `z`; `XYZITR` publishes `x`, `y`, `z`, `intensity`, `time`, `ring`. |
| `asdt1_driver/imu` | `sensor_msgs/msg/Imu` | `asdt1_optical` | IMU samples with angular velocity and linear acceleration. Orientation is not populated. |

Important naming note:

- These topic names are relative topic names.
- Without a namespace, they resolve to `/asdt1_driver/point_cloud` and `/asdt1_driver/imu`.
- With namespace `asdt1_top_left`, they resolve to `/asdt1_top_left/asdt1_driver/point_cloud` and `/asdt1_top_left/asdt1_driver/imu`.

### Point cloud details

- Message type: `sensor_msgs/msg/PointCloud2`
- For `output_format:=XYZ`, the fields are `x`, `y`, `z`
- For `output_format:=XYZITR`, the fields are `x`, `y`, `z`, `intensity`, `time`, `ring`
- Coordinates are published as `FLOAT32`
- `intensity`, `time` are published as `UINT32`
- `ring` is published as `UINT16`
- Frame ID comes from calibration file if available; otherwise defaults to `asdt1_optical`
- The cloud may contain invalid points, so `is_dense` is set to `false`

### IMU details

- Message type: `sensor_msgs/msg/Imu`
- Frame ID matches the depth frame
- `orientation` is not estimated by the driver
- `orientation_covariance[0]` is set to `-1.0` to indicate that orientation is not available
- `angular_velocity_covariance` and `linear_acceleration_covariance` are diagonal matrices with fixed variances (`6.43051e-06` and `3.25895e-06`), derived from the noise densities in `config/imu.yml`

<div class="page"></div>

## Transforms

On connection, the node broadcasts a static transform on `/tf_static`:

| Parent frame | Child frame | Source |
| --- | --- | --- |
| `camera_system_optical` | `asdt1_optical` | Default values when no per-camera frame IDs are provided in the calibration file |

If the calibration YAML contains `frame_id` and `parent_frame_id` for the connected hardware ID, those values are used instead of the defaults.

The translation from the calibration file is converted from millimeters to meters before broadcasting.

## Services

The node exposes these services, all using `std_srvs/srv/Trigger`:

| Service | Type | Description |
| --- | --- | --- |
| `reboot` | `std_srvs/srv/Trigger` | Reboots the device |
| `get_driver_version` | `std_srvs/srv/Trigger` | Returns the ROS driver version in the response message |
| `get_device_info` | `std_srvs/srv/Trigger` | Returns `hw_id`, device version, and current sync mode in the response message |
| `start_streaming` | `std_srvs/srv/Trigger` | Starts streaming |
| `stop_streaming` | `std_srvs/srv/Trigger` | Stops streaming |
| `get_licenses` | `std_srvs/srv/Trigger` | Returns license information for the driver and its third-party components |

Naming note:

- Without a namespace, these resolve to `/reboot`, `/get_driver_version`, `/get_device_info`, `/start_streaming`, `/stop_streaming`, and `/get_licenses`.
- With namespace `asdt1_top_left`, they resolve to `/asdt1_top_left/reboot`, `/asdt1_top_left/get_driver_version`, `/asdt1_top_left/get_device_info`, `/asdt1_top_left/start_streaming`, `/asdt1_top_left/stop_streaming`, and `/asdt1_top_left/get_licenses`.

### Service examples

Single camera without namespace:

```bash
ros2 service call /get_driver_version std_srvs/srv/Trigger
ros2 service call /get_device_info std_srvs/srv/Trigger
ros2 service call /stop_streaming std_srvs/srv/Trigger
ros2 service call /start_streaming std_srvs/srv/Trigger
ros2 service call /reboot std_srvs/srv/Trigger
```

<div class="page"></div>

Namespaced camera example:

```bash
ros2 service call /asdt1_top_left/get_driver_version std_srvs/srv/Trigger
ros2 service call /asdt1_top_left/get_device_info std_srvs/srv/Trigger
```

## Calibration File Behavior

The node loads the calibration file at startup and looks for a `cameras` list. Each entry has these fields:

| Field | Required | Description |
| --- | --- | --- |
| `id` | yes | Hardware serial number of the camera |
| `extrinsics.rotation_matrix` | yes | 3x3 rotation matrix |
| `extrinsics.translation_vector` | yes | Translation vector with 3 values, in millimeters |
| `description` | no | Human-readable name of the camera |
| `frame_id` | no | Child frame ID; can also be given as `extrinsics.frame_id` |
| `parent_frame_id` | no | Parent frame ID; can also be given as `extrinsics.parent_frame_id` |

An entry with a missing or malformed required field is skipped with a warning, and the camera then falls back to the defaults below.

If no matching calibration entry is found for the hardware ID:

- identity extrinsics are used
- the child frame defaults to `asdt1_optical`
- the parent frame defaults to `camera_system_optical`

## Useful ROS 2 Commands

Inspect node information:

```bash
ros2 node info /asdt1_driver
```

List topics and services:

```bash
ros2 topic list
ros2 service list
```

Inspect the point cloud stream:

```bash
ros2 topic echo /asdt1_driver/point_cloud --once
```

Inspect the IMU stream:

```bash
ros2 topic echo /asdt1_driver/imu --once
```
<div class="page"></div>

## Viewing Data in RViz2

To view the point cloud in RViz2:

1. Start the node.
2. Run `rviz2`.
3. Add a `PointCloud2` display.
4. Select the point cloud topic, for example `/asdt1_driver/point_cloud`.
5. Set the fixed frame to the depth frame, usually `asdt1_optical`, unless your calibration file provides another frame ID.
6. Change the QoS/Reliability Policy setting for the topic to `Best effort`.

## Related Utility Node

The package also contains `point_cloud_combiner`, which can combine several point cloud and IMU topics into one output cloud and one IMU stream. Its parameters are:

| Parameter | Default | Description |
| --- | --- | --- |
| `input_topics` | `[]` | Point cloud topics to combine. Must be set; the node logs an error and creates no subscriptions if it is empty. |
| `imu_topics` | `[]` | IMU topics to combine. |
| `output_topic` | `combined_point_cloud` | Combined point cloud topic. |
| `output_imu_topic` | `combined_imu` | Combined IMU topic. |
| `target_frame` | `base_link` | Frame the clouds are transformed into before combining. |

This node is separate from the AS-DT1 hardware driver and is optional.

The resulting combined point cloud will be best aligned if the cameras are HW synchronized and master/slave configuration has been set up in the calibration file.

Timestamp for the combined point cloud will be set to the timestamp from the first topic in `input_topics`. The provided launch file lists the topics in the same order as the cameras in the calibration file.
