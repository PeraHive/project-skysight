#!/usr/bin/env python3
import time
import math
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from simple_pid import PID  # pip install simple-pid


def ang_norm(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def pose_xyz_yaw(p: PoseStamped) -> Tuple[float, float, float, float]:
    q = p.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return float(p.pose.position.x), float(p.pose.position.y), float(p.pose.position.z), float(yaw)


class YawFollowSimplePIDNS(Node):
    """
    Takeoff + hold altitude; yaw-rate tracks /detected/offset.x via PID (simple-pid).
    All MAVROS topics are namespaced via 'uav_ns'.
    """

    def __init__(self) -> None:
        super().__init__('yaw_follow_simplepid_ns')

        # --- QoS similar to MAVROS sensor topics ---
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.durability  = QoSDurabilityPolicy.VOLATILE

        # ---------- Parameters ----------
        # Vehicle / mission
        self.uav_ns      = self.declare_parameter('uav_ns', '/uav1').get_parameter_value().string_value
        self.guided_mode = self.declare_parameter('guided_mode', 'GUIDED').get_parameter_value().string_value  # 'OFFBOARD' for PX4
        self.target_alt  = float(self.declare_parameter('target_alt', 3.0).get_parameter_value().double_value)
        self.alt_thresh  = float(self.declare_parameter('alt_air_thresh', 0.5).get_parameter_value().double_value)
        self.sp_rate_hz  = float(self.declare_parameter('sp_rate_hz', 5.0).get_parameter_value().double_value)

        # Offset source
        self.offset_topic = self.declare_parameter('offset_topic', '/detected/offset').get_parameter_value().string_value

        # Error normalization (optional)
        self.normalize_error = bool(self.declare_parameter('normalize_error', False).get_parameter_value().bool_value)
        self.image_width_px  = float(self.declare_parameter('image_width_px', 640.0).get_parameter_value().double_value)

        # PID gains + limits (live tunable)
        self.Kp = float(self.declare_parameter('Kp', 0.002).get_parameter_value().double_value)
        self.Ki = float(self.declare_parameter('Ki', 0.0000).get_parameter_value().double_value)
        self.Kd = float(self.declare_parameter('Kd', 0.0000).get_parameter_value().double_value)
        self.yaw_rate_limit = float(self.declare_parameter('yaw_rate_limit', 0.7).get_parameter_value().double_value)  # rad/s
        self.sample_time = float(self.declare_parameter('pid_sample_time', 0.0).get_parameter_value().double_value)     # 0.0 = run every call

        # Build PID (target = 0 error)
        self.pid = PID(self.Kp, self.Ki, self.Kd, setpoint=0.0, sample_time=self.sample_time)
        self.pid.output_limits = (-abs(self.yaw_rate_limit), abs(self.yaw_rate_limit))

        # Watch for live param updates
        self.add_on_set_parameters_callback(self._on_params)

        # ---------- State ----------
        self.state = State()
        self.rel_alt = 0.0
        self.pose = PoseStamped()
        self.offset = Point()
        self.offset_received = False  # set True when first offset arrives

        # ---------- I/O ----------
        self.create_subscription(State,       f'{self.uav_ns}/state',                    self._state_cb, qos)
        self.create_subscription(Float64,     f'{self.uav_ns}/global_position/rel_alt',  self._alt_cb,   qos)
        self.create_subscription(PoseStamped, f'{self.uav_ns}/local_position/pose',      self._pose_cb,  qos)
        self.create_subscription(Point,       self.offset_topic,                         self._offset_cb, 10)

        self.sp_pub = self.create_publisher(PositionTarget, f'{self.uav_ns}/setpoint_raw/local', 10)

        self.arm_cli     = self.create_client(CommandBool, f'{self.uav_ns}/cmd/arming')
        self.tko_cli     = self.create_client(CommandTOL,  f'{self.uav_ns}/cmd/takeoff')
        self.setmode_cli = self.create_client(SetMode,     f'{self.uav_ns}/set_mode')

        # Reset integrator service
        self.reset_srv = self.create_service(Empty, 'reset_pid_integrator', self._handle_reset_pid)

        self.get_logger().info(f"MAVROS NS: {self.uav_ns} | Offset topic: {self.offset_topic}")
        self.get_logger().info('Waiting for MAVROS services...')
        for cli in (self.arm_cli, self.tko_cli, self.setmode_cli):
            cli.wait_for_service()
        self.get_logger().info('MAVROS services are available.')

    # ---------- Param live update ----------
    def _on_params(self, params):
        for p in params:
            if p.name == 'Kp': self.Kp = float(p.value)
            elif p.name == 'Ki': self.Ki = float(p.value)
            elif p.name == 'Kd': self.Kd = float(p.value)
            elif p.name == 'yaw_rate_limit': self.yaw_rate_limit = float(p.value)
            elif p.name == 'pid_sample_time': self.sample_time = float(p.value)
            elif p.name == 'normalize_error': self.normalize_error = bool(p.value)
            elif p.name == 'image_width_px': self.image_width_px = float(p.value)
        # Push into PID
        self.pid.tunings = (self.Kp, self.Ki, self.Kd)
        self.pid.sample_time = self.sample_time
        self.pid.output_limits = (-abs(self.yaw_rate_limit), abs(self.yaw_rate_limit))
        return SetParametersResult(successful=True)

    # ---------- Callbacks ----------
    def _handle_reset_pid(self, req, resp):
        self.pid.reset()
        self.get_logger().info("PID integrator reset.")
        return resp

    def _state_cb(self, msg: State): self.state = msg
    def _alt_cb(self, msg: Float64): self.rel_alt = float(msg.data)
    def _pose_cb(self, msg: PoseStamped): self.pose = msg
    def _offset_cb(self, msg: Point):
        self.offset = msg
        self.offset_received = True

    # ---------- MAVROS helpers ----------
    def _set_mode(self, mode: str) -> bool:
        req = SetMode.Request(); req.base_mode = 0; req.custom_mode = mode
        fut = self.setmode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return bool(fut.done() and fut.result() and getattr(fut.result(), 'mode_sent', False))

    def _arm(self) -> bool:
        req = CommandBool.Request(); req.value = True
        fut = self.arm_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))

    def _takeoff(self, alt: float) -> bool:
        req = CommandTOL.Request()
        req.altitude = float(alt); req.latitude=0.0; req.longitude=0.0; req.min_pitch=0.0; req.yaw=float('nan')
        fut = self.tko_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return bool(fut.done() and fut.result() and getattr(fut.result(), 'success', False))

    # ---------- Setpoint publisher ----------
    def _publish_xyz_yawrate(self, x: float, y: float, z: float, yaw_rate: float):
        sp = PositionTarget()
        sp.header.stamp = self.get_clock().now().to_msg()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        sp.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW     # we ignore absolute yaw, we DO send yaw_rate (so DO NOT ignore yaw_rate)
        )
        sp.position.x = float(x); sp.position.y = float(y); sp.position.z = float(z)
        sp.yaw_rate = float(yaw_rate)  # rad/s
        self.sp_pub.publish(sp)

    # ---------- Main run loop ----------
    def run(self):
        # 1) Wait FCU connection
        while rclpy.ok() and not self.state.connected:
            self.get_logger().info('Waiting for FCU…')
            rclpy.spin_once(self, timeout_sec=0.2)

        # 2) Mode + arm + takeoff
        self._set_mode(self.guided_mode)
        while rclpy.ok() and not self.state.armed:
            if not self._arm():
                self.get_logger().warn('Arming failed, retrying…')
            rclpy.spin_once(self, timeout_sec=0.2)

        if not self._takeoff(self.target_alt):
            self.get_logger().error('Takeoff failed.')
            return

        while rclpy.ok() and self.rel_alt < (self.target_alt - self.alt_thresh):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info(f'UAV airborne at {self.rel_alt:.1f} m')

        # 3) Wait for offset messages
        self.get_logger().info('Waiting for offset messages…')
        while rclpy.ok() and not self.offset_received:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info('Offset messages received.')

        # 4) Control loop
        dt = 1.0 / max(1e-3, self.sp_rate_hz)  # proper loop period
        self.get_logger().info('Tracking target yaw…')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)

            # Current pose
            x, y, _, _ = pose_xyz_yaw(self.pose)

            # Pixel error → optionally normalize to [-1..1]
            err = float(self.offset.x)
            if self.normalize_error and self.image_width_px > 1.0:
                half_w = self.image_width_px / 2.0
                err = max(-half_w, min(half_w, err)) / half_w

            # PID → yaw rate (rad/s). Using -err means “turn toward the target”.
            yaw_rate_cmd = float(self.pid(-err))

            self._publish_xyz_yawrate(x, y, self.target_alt, yaw_rate_cmd)
            time.sleep(dt)


def main():
    rclpy.init()
    node = YawFollowSimplePIDNS()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
