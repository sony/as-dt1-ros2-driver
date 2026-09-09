# as-dt1-ros2-driver

ROS 2 driver for the **AS-DT1** LiDAR camera by Sony.

## Overview

This package provides a ROS 2 node that interfaces with the AS-DT1 LiDAR camera over USB. It handles device discovery, streaming, and publishes point cloud and IMU data to ROS 2 topics. Multiple cameras can be operated simultaneously in master/slave synchronisation mode.

The driver has been built and tested on ROS 2 Humble and Jazzy on both x86 and ARM64 hosts. It currently requires AS-DT1 firmware version 1.01 or later.

### Key features

- Point cloud publishing (`sensor_msgs/PointCloud2`) in XYZ or XYZITR format
- IMU data publishing (`sensor_msgs/Imu`)
- Multi-camera synchronisation via calibration file
- Runtime control via ROS 2 services (start/stop streaming, device info etc)
- Configurable sensor mode, frame rate, and sync role

## Packages

| Package | Node | Description |
|---|---|---|
| `asdt1_ros2_driver` | `asdt1_driver` | Per-camera driver node — device communication and point cloud publishing |
| `asdt1_ros2_driver` | `point_cloud_combiner` | Combines point clouds from multiple cameras into a single stream |

## Code outline

The source is split into a ROS-agnostic device layer and a ROS 2 node layer.

### Device layer (`src/asdt1_driver/src/device/`)

The device layer has no ROS dependency and can be used or tested independently. Logging is injected via a `log_callback` at construction time, keeping the layer ROS-agnostic.

Two implementations of the driver exist, with AS-DT1 set up in CDC or UVC mode. UVC mode is used when XYZITR data output is requested.

| File | Description |
|---|---|
| `dt1_device.cpp` | Abstract base class. Manages the serial connection lifecycle — port discovery, connect/reconnect loop, mode negotiation, frame rate and sync configuration. Spawns a serial thread that drives the state machine. |
| `dt1_device_cdc.cpp` | CDC (serial) mode implementation. Receives ASCII binary frames over the serial port and publishes XYZ point clouds. |
| `dt1_device_hist.cpp` | UVC (V4L2/histogram) mode implementation. Required for full XYZITR output. Manages V4L2 buffer streaming, media device enumeration, and histogram frame unpacking into XYZITR point clouds including ring, intensity, and per-point timestamps. |
| `dt1_modes.cpp` | Sensor mode table — maps mode names (e.g. `30MSTD`, `20m`) to frame geometry and bank counts. |
| `calibration_parser.cpp` | Parses the YAML calibration file. Singleton with mutex-protected init. Provides per-camera extrinsics, frame IDs, and serial-number-to-description mapping. |

### ROS 2 node layer (`src/asdt1_driver/src/`)

| File | Description |
|---|---|
| `asdt1_driver_node.cpp` | ROS 2 composable node wrapping `dt1_device`. Handles parameter declaration, publisher creation on camera connect, TF broadcasting, and conversion of device callbacks to `PointCloud2` and `Imu` messages. |
| `point_cloud_combiner_node.cpp` | ROS 2 composable node that subscribes to N per-camera point cloud topics, applies extrinsic transforms to a common target frame, and publishes a single combined `PointCloud2`. Uses approximate-time synchronisation with a configurable tolerance. Also fuses per-camera IMU streams into a single `Imu` message by averaging angular velocity and linear acceleration after transforming each to the target frame. This is a simple implementation that does not correctly handle the case where IMUs are mounted in a non-symmetric fashion — the averaged linear acceleration will be incorrect if the sensors are at different offsets from the target frame origin. |

### Key types (`src/asdt1_driver/src/device/include/asdt1_types.h`)

- `points_data` — `std::variant` holding either a `const std::vector<Vector3f>&` (XYZ) or `const std::vector<Vector6f>&` (XYZITR), passed zero-copy via `std::reference_wrapper`.
- `point_cloud_callback`, `imu_callback`, `log_callback` — function types injected into the device layer at construction.

## Documentation

See the [User Manual](doc/user_manual.md) for setup and usage instructions.

For detailed information about the AS-DT1 communication protocol and data layout, which the driver adheres to, see the AS-DT1 API Manual from the official AS-DT1 SDK package. The latest SDK can be found under the resources [here](https://www.image-sensing-solutions.eu/LiDAR-as-dt1.html).

## Dependencies

Non-ROS dependencies (install before building):

```bash
# Eigen3
sudo apt install libeigen3-dev
```

The `serial` library is bundled as a vendored dependency and requires no separate installation.

## License

Copyright 2026 Sony Depthsensing Solutions. See [LICENSE](LICENSE) for details.
