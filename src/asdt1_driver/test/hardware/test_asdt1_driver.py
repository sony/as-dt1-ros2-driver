"""
Hardware-in-the-loop tests for the asdt1_driver node.

Requires a single AS-DT1 camera connected. Set environment variables:
  ASDT1_CALIBRATION_FILE  path to calibration YAML (required)
  ASDT1_MODE              camera mode: 20m, 30mstd, 30m15f, 30m30f, 40m (default: 30m30f)
  ASDT1_OUTPUT_FORMAT     XYZ or XYZITR (default: XYZ)
  ASDT1_FRAMERATE         frame rate in Hz, 1-30 (default: 30)

Run all mode/format combinations:
  pytest test_asdt1_driver.py --hw

Run all frame rate tests:
  pytest test_asdt1_driver.py::test_all_framerates --hw

Run a specific combination:
  ASDT1_MODE=20m ASDT1_OUTPUT_FORMAT=XYZITR ASDT1_FRAMERATE=15 pytest test_asdt1_driver.py --hw
"""

import os
import time
import math
import itertools
import unittest

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger
import sensor_msgs_py.point_cloud2 as pc2

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

import launch
from launch import LaunchDescription
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers

# ---------------------------------------------------------------------------
# Mode definitions — must match dt1_modes_list in dt1_modes.cpp
# ---------------------------------------------------------------------------
MODE_POINT_COUNTS = {
    "20m":    576,   # 24 x 24
    "30mstd": 576,   # 24 x 24
    "30m15f": 576,   # 24 x 24
    "30m30f": 288,   # 24 x 12
    "40m":    576,   # 24 x 24
}

MODE_MAX_RANGE = {
    "20m":    20.0,
    "30mstd": 30.0,
    "30m15f": 30.0,
    "30m30f": 30.0,
    "40m":    40.0,
}

XYZ_FIELDS    = {"x", "y", "z"}
XYZITR_FIELDS = {"x", "y", "z", "intensity", "time", "ring"}

ALL_MODES      = list(MODE_POINT_COUNTS.keys())
ALL_FORMATS    = ["XYZ", "XYZITR"]
ALL_FRAMERATES = [1, 5, 10, 15, 20, 25, 30]
ALL_COMBINATIONS = list(itertools.product(ALL_MODES, ALL_FORMATS))

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _calibration_file():
    path = os.environ.get("ASDT1_CALIBRATION_FILE")
    if not path:
        pytest.skip("ASDT1_CALIBRATION_FILE not set — skipping hardware test")
    return path


def _collect_messages(node, topic, msg_type, count, timeout_sec=10.0):
    """Spin node until `count` messages arrive on `topic` or timeout."""
    msgs = []
    sub = node.create_subscription(msg_type, topic, lambda m: msgs.append(m), SENSOR_QOS)
    deadline = time.monotonic() + timeout_sec
    while len(msgs) < count and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_subscription(sub)
    return msgs


def _point_cloud_fields(msg):
    return {f.name for f in msg.fields}


def _call_service(node, service_name, timeout_sec=5.0):
    """Call a Trigger service and return the response, or None on timeout."""
    client = node.create_client(Trigger, service_name)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        client.destroy()
        return None
    future = client.call_async(Trigger.Request())
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    client.destroy()
    return future.result() if future.done() else None


def _iter_xyz(msg):
    """Yield (x, y, z) tuples from a PointCloud2 message."""
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False):
        yield p[0], p[1], p[2]


# ---------------------------------------------------------------------------
# Parametrised launch description
# ---------------------------------------------------------------------------

@launch_testing.markers.keep_alive
def generate_test_description():
    calibration_file = os.environ.get("ASDT1_CALIBRATION_FILE", "")
    mode             = os.environ.get("ASDT1_MODE", "30m30f")
    output_format    = os.environ.get("ASDT1_OUTPUT_FORMAT", "XYZ")
    framerate        = int(os.environ.get("ASDT1_FRAMERATE", "30"))

    driver_node = launch_ros.actions.Node(
        package="asdt1_ros2_driver",
        executable="asdt1_driver",
        name="asdt1_driver",
        parameters=[{
            "calibration_file": calibration_file,
            "mode":             mode,
            "output_format":    output_format,
            "framerate":        framerate,
        }],
        output="screen",
    )

    return LaunchDescription([
        driver_node,
        launch_testing.actions.ReadyToTest(),
    ]), {"driver_node": driver_node}


# ---------------------------------------------------------------------------
# Base test class — shared setup/teardown
# ---------------------------------------------------------------------------

class DriverTestBase(unittest.TestCase):
    POINT_CLOUD_TOPIC = "/asdt1_driver/point_cloud"
    WARMUP_FRAMES     = 5     # discard before asserting (device settling)
    TEST_FRAMES       = 30

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("asdt1_hw_test_node")
        cls.mode          = os.environ.get("ASDT1_MODE", "30m30f")
        cls.output_format = os.environ.get("ASDT1_OUTPUT_FORMAT", "XYZ")
        cls.framerate     = int(os.environ.get("ASDT1_FRAMERATE", "30"))
        cls.max_points    = MODE_POINT_COUNTS[cls.mode]
        cls.max_range     = MODE_MAX_RANGE[cls.mode]

        # Wait for node to come up and discard warmup frames
        _collect_messages(cls.node, cls.POINT_CLOUD_TOPIC, PointCloud2,
                          cls.WARMUP_FRAMES, timeout_sec=15.0)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _get_frames(self, count=None):
        n = count or self.TEST_FRAMES
        # Discard 2 frames first to ensure the subscription is live and
        # the first collected frame is not separated by a large gap from
        # when the subscriber was last listening.
        _collect_messages(self.node, self.POINT_CLOUD_TOPIC, PointCloud2,
                          2, timeout_sec=5.0)
        msgs = _collect_messages(self.node, self.POINT_CLOUD_TOPIC,
                                 PointCloud2, n, timeout_sec=15.0)
        self.assertGreaterEqual(
            len(msgs), n,
            f"Only received {len(msgs)}/{n} frames within timeout — "
            "is the device connected and streaming?")
        return msgs


# ---------------------------------------------------------------------------
# Test cases
# ---------------------------------------------------------------------------

class TestDriverPublishes(DriverTestBase):
    """Basic liveness — node comes up and publishes frames."""

    def test_publishes_within_timeout(self):
        msgs = _collect_messages(self.node, self.POINT_CLOUD_TOPIC,
                                 PointCloud2, 1, timeout_sec=15.0)
        self.assertGreaterEqual(len(msgs), 1,
            "Driver did not publish any point cloud within 15 seconds")

    def test_frame_id_not_empty(self):
        msgs = self._get_frames(5)
        for msg in msgs:
            self.assertTrue(len(msg.header.frame_id) > 0,
                "frame_id is empty")


class TestOutputFormat(DriverTestBase):
    """PointCloud2 field schema matches the configured output format."""

    def test_fields_match_output_format(self):
        msgs = self._get_frames(5)
        expected = XYZITR_FIELDS if self.output_format == "XYZITR" else XYZ_FIELDS
        for msg in msgs:
            actual = _point_cloud_fields(msg)
            self.assertEqual(actual, expected,
                f"Field mismatch for format {self.output_format}: "
                f"got {actual}, expected {expected}")

    def test_point_step_matches_format(self):
        msgs = self._get_frames(5)
        # XYZ: 3 × float32 = 12 bytes
        # XYZITR: x,y,z (12) + intensity uint32 (4) + time uint32 (4) + ring uint16 (2) = 22 bytes
        expected_step = 22 if self.output_format == "XYZITR" else 12
        for msg in msgs:
            self.assertEqual(msg.point_step, expected_step,
                f"point_step={msg.point_step}, expected {expected_step} "
                f"for format {self.output_format}")


class TestPointCount(DriverTestBase):
    """Point count per frame is within expected bounds."""

    def test_point_count_not_zero(self):
        msgs = self._get_frames()
        for i, msg in enumerate(msgs):
            count = msg.width * msg.height
            self.assertGreater(count, 0, f"Frame {i} has zero points")

    def test_point_count_within_mode_maximum(self):
        msgs = self._get_frames()
        for i, msg in enumerate(msgs):
            count = msg.width * msg.height
            self.assertLessEqual(count, self.max_points,
                f"Frame {i}: point count {count} exceeds mode maximum "
                f"{self.max_points} for mode {self.mode}")

    def test_point_count_above_minimum_threshold(self):
        """At least 10% of maximum points should be valid (scene dependent but catches dead sensors)."""
        msgs = self._get_frames()
        min_expected = max(1, self.max_points // 10)
        for i, msg in enumerate(msgs):
            count = msg.width * msg.height
            self.assertGreaterEqual(count, min_expected,
                f"Frame {i}: only {count} points, expected at least {min_expected} "
                f"(10% of mode maximum {self.max_points})")


class TestPointValidity(DriverTestBase):
    """Individual point values are physically plausible."""

    def test_no_nan_or_inf(self):
        msgs = self._get_frames()
        for i, msg in enumerate(msgs):
            for x, y, z in _iter_xyz(msg):
                self.assertFalse(math.isnan(x) or math.isnan(y) or math.isnan(z),
                    f"Frame {i}: NaN in point ({x}, {y}, {z})")
                self.assertFalse(math.isinf(x) or math.isinf(y) or math.isinf(z),
                    f"Frame {i}: Inf in point ({x}, {y}, {z})")

    def test_z_above_minimum(self):
        """Z must be above the 0.2m filter threshold — lower values are filtered in driver."""
        msgs = self._get_frames()
        for i, msg in enumerate(msgs):
            for x, y, z in _iter_xyz(msg):
                self.assertGreaterEqual(z, 0.2,
                    f"Frame {i}: z={z:.4f} below minimum 0.2m — "
                    "garbage point passed filter")

    def test_z_below_mode_max_range(self):
        """Z must not exceed the mode's maximum range."""
        msgs = self._get_frames()
        for i, msg in enumerate(msgs):
            for x, y, z in _iter_xyz(msg):
                self.assertLessEqual(z, self.max_range,
                    f"Frame {i}: z={z:.4f} exceeds mode max range "
                    f"{self.max_range}m for mode {self.mode}")

    def test_xyzitr_intensity_not_zero(self):
        """Intensity field should be non-zero for valid returns (XYZITR only)."""
        if self.output_format != "XYZITR":
            self.skipTest("Only applies to XYZITR format")
        msgs = self._get_frames()
        zero_intensity_count = 0
        total_points = 0
        for msg in msgs:
            for p in pc2.read_points(msg, field_names=("intensity",), skip_nans=False):
                total_points += 1
                if p[0] == 0:
                    zero_intensity_count += 1
        zero_ratio = zero_intensity_count / max(total_points, 1)
        self.assertLess(zero_ratio, 0.5,
            f"More than 50% of points have zero intensity ({zero_intensity_count}/{total_points}) — "
            "suggests unpacking error in XYZITR mode")

    def test_xyzitr_timestamps_within_frame_window(self):
        """Per-point timestamps (rolling shutter) should be within 30ms of each other (XYZITR only)."""
        if self.output_format != "XYZITR":
            self.skipTest("Only applies to XYZITR format")
        msgs = self._get_frames()
        for i, msg in enumerate(msgs):
            times = [p[0] for p in pc2.read_points(msg, field_names=("time",), skip_nans=False)]
            if len(times) < 2:
                continue
            spread_ms = (max(times) - min(times)) / 1e6  # assuming microseconds
            self.assertLessEqual(spread_ms, 30.0,
                f"Frame {i}: per-point timestamp spread {spread_ms:.1f}ms exceeds 30ms — "
                "possible timestamp underflow or overflow")


class TestTimestamps(DriverTestBase):
    """Frame-level header timestamps are well-behaved."""

    def test_timestamps_monotonically_increasing(self):
        msgs = self._get_frames()
        for i in range(1, len(msgs)):
            prev = msgs[i - 1].header.stamp
            curr = msgs[i].header.stamp
            prev_ns = prev.sec * 1_000_000_000 + prev.nanosec
            curr_ns = curr.sec * 1_000_000_000 + curr.nanosec
            self.assertGreater(curr_ns, prev_ns,
                f"Timestamp went backwards between frame {i-1} and {i}: "
                f"{prev_ns} → {curr_ns}")

    def test_frame_interval_within_bounds(self):
        """Inter-frame interval should not exceed 2× the configured period."""
        msgs = self._get_frames()
        max_interval_ms = (1000.0 / self.framerate) * 2.0
        for i in range(1, len(msgs)):
            prev = msgs[i - 1].header.stamp
            curr = msgs[i].header.stamp
            prev_ns = prev.sec * 1_000_000_000 + prev.nanosec
            curr_ns = curr.sec * 1_000_000_000 + curr.nanosec
            interval_ms = (curr_ns - prev_ns) / 1e6
            self.assertLess(interval_ms, max_interval_ms,
                f"Frame interval {interval_ms:.1f}ms between frame {i-1} and {i} "
                f"exceeds 2× expected period ({max_interval_ms:.1f}ms) "
                f"for framerate={self.framerate}Hz")


class TestFrameRate(DriverTestBase):
    """Achieved frame rate matches the configured framerate parameter."""

    # Collect enough frames to measure rate accurately; at 1 Hz this takes ~15s.
    # Timeout is set generously to accommodate low framerates.
    MEASURE_FRAMES = 10

    def _measure_achieved_fps(self):
        """Return (achieved_fps, intervals_ms) over MEASURE_FRAMES frames."""
        timeout = max(15.0, (self.MEASURE_FRAMES / self.framerate) * 3.0)
        msgs = _collect_messages(self.node, self.POINT_CLOUD_TOPIC,
                                 PointCloud2, self.MEASURE_FRAMES, timeout_sec=timeout)
        self.assertGreaterEqual(len(msgs), self.MEASURE_FRAMES,
            f"Only received {len(msgs)}/{self.MEASURE_FRAMES} frames at "
            f"{self.framerate}Hz within {timeout:.0f}s timeout")

        def stamp_ns(msg):
            return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        intervals_ms = [
            (stamp_ns(msgs[i]) - stamp_ns(msgs[i - 1])) / 1e6
            for i in range(1, len(msgs))
        ]
        elapsed_sec = (stamp_ns(msgs[-1]) - stamp_ns(msgs[0])) / 1e9
        achieved_fps = (len(msgs) - 1) / elapsed_sec if elapsed_sec > 0 else 0.0
        return achieved_fps, intervals_ms

    def test_achieved_fps_within_tolerance(self):
        """Achieved FPS must be within ±20% of the configured framerate."""
        achieved_fps, _ = self._measure_achieved_fps()
        low  = self.framerate * 0.80
        high = self.framerate * 1.20
        self.assertGreaterEqual(achieved_fps, low,
            f"Achieved {achieved_fps:.2f}fps is more than 20% below "
            f"configured {self.framerate}Hz")
        self.assertLessEqual(achieved_fps, high,
            f"Achieved {achieved_fps:.2f}fps is more than 20% above "
            f"configured {self.framerate}Hz")

    def test_no_frame_stalls(self):
        """No single inter-frame gap should exceed 3× the expected period."""
        _, intervals_ms = self._measure_achieved_fps()
        max_allowed_ms = (1000.0 / self.framerate) * 3.0
        for i, interval in enumerate(intervals_ms):
            self.assertLess(interval, max_allowed_ms,
                f"Frame gap {i}→{i+1} was {interval:.1f}ms — "
                f"exceeds 3× expected period ({max_allowed_ms:.1f}ms) "
                f"at {self.framerate}Hz")

    def test_no_frame_bursts(self):
        """No single inter-frame gap should be less than 10% of the expected period.
        Bursts (frames arriving much faster than configured) suggest the framerate
        parameter is not being applied correctly."""
        _, intervals_ms = self._measure_achieved_fps()
        min_allowed_ms = (1000.0 / self.framerate) * 0.10
        for i, interval in enumerate(intervals_ms):
            self.assertGreater(interval, min_allowed_ms,
                f"Frame gap {i}→{i+1} was {interval:.1f}ms — "
                f"suspiciously short for {self.framerate}Hz "
                f"(minimum expected {min_allowed_ms:.1f}ms)")

    def test_interval_jitter_acceptable(self):
        """Standard deviation of inter-frame intervals should be <25% of the expected period."""
        _, intervals_ms = self._measure_achieved_fps()
        if len(intervals_ms) < 3:
            self.skipTest("Not enough intervals to measure jitter")
        mean = sum(intervals_ms) / len(intervals_ms)
        variance = sum((x - mean) ** 2 for x in intervals_ms) / len(intervals_ms)
        stddev_ms = variance ** 0.5
        expected_period_ms = 1000.0 / self.framerate
        self.assertLess(stddev_ms, expected_period_ms * 0.25,
            f"Frame interval jitter stddev={stddev_ms:.1f}ms exceeds 25% of "
            f"expected period {expected_period_ms:.1f}ms at {self.framerate}Hz")


class TestServices(DriverTestBase):
    """ROS service interface — availability, response validity, and streaming control."""

    # get_driver_version and get_licenses are registered at node startup (before camera connects).
    # get_device_info, start_streaming, stop_streaming are registered only after camera connects,
    # so the base class warmup (which waits for frames) guarantees they are available.

    def tearDown(self):
        """Ensure streaming is always restarted after each test so subsequent tests are not affected."""
        _call_service(self.node, "start_streaming")

    def test_get_driver_version_available(self):
        resp = _call_service(self.node, "get_driver_version")
        self.assertIsNotNone(resp, "get_driver_version service not available")

    def test_get_driver_version_succeeds(self):
        resp = _call_service(self.node, "get_driver_version")
        self.assertIsNotNone(resp, "get_driver_version service not available")
        self.assertTrue(resp.success, f"get_driver_version returned success=False: {resp.message}")

    def test_get_driver_version_message_not_empty(self):
        resp = _call_service(self.node, "get_driver_version")
        self.assertIsNotNone(resp, "get_driver_version service not available")
        self.assertTrue(len(resp.message) > 0, "get_driver_version returned empty message")

    def test_get_licenses_available(self):
        resp = _call_service(self.node, "get_licenses")
        self.assertIsNotNone(resp, "get_licenses service not available")

    def test_get_licenses_succeeds(self):
        resp = _call_service(self.node, "get_licenses")
        self.assertIsNotNone(resp, "get_licenses service not available")
        self.assertTrue(resp.success, f"get_licenses returned success=False: {resp.message}")

    def test_get_licenses_message_not_empty(self):
        resp = _call_service(self.node, "get_licenses")
        self.assertIsNotNone(resp, "get_licenses service not available")
        self.assertTrue(len(resp.message) > 0, "get_licenses returned empty message")

    def test_get_device_info_available(self):
        resp = _call_service(self.node, "get_device_info")
        self.assertIsNotNone(resp, "get_device_info service not available — camera may not be connected")

    def test_get_device_info_succeeds(self):
        resp = _call_service(self.node, "get_device_info")
        self.assertIsNotNone(resp, "get_device_info service not available")
        self.assertTrue(resp.success, f"get_device_info returned success=False: {resp.message}")

    def test_get_device_info_contains_expected_fields(self):
        """Response message should contain hw_id, version, and sync_mode fields."""
        resp = _call_service(self.node, "get_device_info")
        self.assertIsNotNone(resp, "get_device_info service not available")
        self.assertIn("hw_id", resp.message,
            f"get_device_info response missing 'hw_id': {resp.message}")
        self.assertIn("version", resp.message,
            f"get_device_info response missing 'version': {resp.message}")
        self.assertIn("sync_mode", resp.message,
            f"get_device_info response missing 'sync_mode': {resp.message}")

    def test_stop_streaming_succeeds(self):
        resp = _call_service(self.node, "stop_streaming")
        self.assertIsNotNone(resp, "stop_streaming service not available")
        self.assertTrue(resp.success, f"stop_streaming returned success=False: {resp.message}")

    def test_stop_streaming_halts_frames(self):
        """After stop_streaming, no new frames should arrive within 2 seconds."""
        resp = _call_service(self.node, "stop_streaming")
        self.assertIsNotNone(resp, "stop_streaming service not available")
        self.assertTrue(resp.success, f"stop_streaming failed: {resp.message}")

        msgs = _collect_messages(self.node, self.POINT_CLOUD_TOPIC,
                                 PointCloud2, 1, timeout_sec=2.0)
        self.assertEqual(len(msgs), 0,
            f"Received {len(msgs)} frames after stop_streaming — streaming did not stop")

    def test_start_streaming_succeeds(self):
        # Ensure we are stopped first
        _call_service(self.node, "stop_streaming")
        resp = _call_service(self.node, "start_streaming")
        self.assertIsNotNone(resp, "start_streaming service not available")
        self.assertTrue(resp.success, f"start_streaming returned success=False: {resp.message}")

    def test_start_streaming_resumes_frames(self):
        """After stop then start, frames should resume within 5 seconds."""
        _call_service(self.node, "stop_streaming")
        time.sleep(0.5)
        resp = _call_service(self.node, "start_streaming")
        self.assertIsNotNone(resp, "start_streaming service not available")
        self.assertTrue(resp.success, f"start_streaming failed: {resp.message}")

        msgs = _collect_messages(self.node, self.POINT_CLOUD_TOPIC,
                                 PointCloud2, 3, timeout_sec=5.0)
        self.assertGreaterEqual(len(msgs), 3,
            "Frames did not resume after start_streaming")

    def test_stop_start_cycle_repeatable(self):
        """stop → start cycle should work reliably three times in a row."""
        for cycle in range(3):
            stop = _call_service(self.node, "stop_streaming")
            self.assertIsNotNone(stop, f"stop_streaming unavailable on cycle {cycle}")
            self.assertTrue(stop.success, f"stop_streaming failed on cycle {cycle}: {stop.message}")
            time.sleep(0.2)

            start = _call_service(self.node, "start_streaming")
            self.assertIsNotNone(start, f"start_streaming unavailable on cycle {cycle}")
            self.assertTrue(start.success, f"start_streaming failed on cycle {cycle}: {start.message}")

            msgs = _collect_messages(self.node, self.POINT_CLOUD_TOPIC,
                                     PointCloud2, 2, timeout_sec=5.0)
            self.assertGreaterEqual(len(msgs), 2,
                f"Frames did not resume after start_streaming on cycle {cycle}")


# ---------------------------------------------------------------------------
# Parametrised runners
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("mode,output_format", ALL_COMBINATIONS,
    ids=[f"{m}_{f}" for m, f in ALL_COMBINATIONS])
def test_all_combinations(mode, output_format):
    """Smoke test: driver publishes valid frames for every mode+format combination."""
    _calibration_file()

    import subprocess, sys
    env = os.environ.copy()
    env["ASDT1_MODE"]          = mode
    env["ASDT1_OUTPUT_FORMAT"] = output_format

    result = subprocess.run(
        [sys.executable, "-m", "pytest", __file__, "--hw",
         "-k", "TestDriverPublishes or TestOutputFormat or TestPointCount",
         "--tb=short", "-q"],
        env=env,
        capture_output=True, text=True
    )
    assert result.returncode == 0, (
        f"Combination mode={mode} format={output_format} failed:\n"
        f"{result.stdout}\n{result.stderr}"
    )


@pytest.mark.parametrize("framerate", ALL_FRAMERATES,
    ids=[f"{r}hz" for r in ALL_FRAMERATES])
def test_all_framerates(framerate):
    """Verify the driver honours the framerate parameter across all supported values."""
    _calibration_file()

    import subprocess, sys
    env = os.environ.copy()
    env["ASDT1_FRAMERATE"] = str(framerate)

    result = subprocess.run(
        [sys.executable, "-m", "pytest", __file__, "--hw",
         "-k", "TestFrameRate",
         "--tb=short", "-q"],
        env=env,
        capture_output=True, text=True
    )
    assert result.returncode == 0, (
        f"Framerate {framerate}Hz failed:\n"
        f"{result.stdout}\n{result.stderr}"
    )
