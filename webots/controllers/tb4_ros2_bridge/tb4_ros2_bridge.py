#!/usr/bin/env python3
"""Webots <-> ROS 2 bridge for the TurtleBot 4 proto (extern controller).

Run it with the system ROS 2 Python (see ../../run_sim.sh):
  publishes  /clock, /scan, /odom, /imu, /tf (odom->base_link), /tf_static,
             /ground_truth/pose (start-relative, i.e. in Hector's map frame;
             z is the height above the start floor)
  subscribes /cmd_vel_unstamped
With --patrol the robot ignores /cmd_vel_unstamped and drives the waypoint loop from
the layout module (--layout, default tools/layout.py) so SLAM runs are
repeatable.
"""
import argparse
import importlib
import math
import os
import sys

import rclpy
from controller import Robot
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', 'tools'))

WHEEL_RADIUS = 0.03575
WHEEL_SEPARATION = 0.233
MAX_WHEEL_SPEED = 10.0  # rad/s, maxVelocity of the proto wheel motors
# rplidar_link pose in base_link, taken from the proto
LIDAR_XYZ = (-0.04, 0.0, 0.1929)
LIDAR_YAW = math.pi / 2


def yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Bridge:
    def __init__(self, patrol, lidar_period_ms, reverse_scan):
        self.robot = Robot()
        self.dt = int(self.robot.getBasicTimeStep())
        self.patrol = patrol  # list of (x, y) waypoints, or None
        self.reverse_scan = reverse_scan

        self.left = self.robot.getDevice('left_wheel_joint')
        self.right = self.robot.getDevice('right_wheel_joint')
        for m in (self.left, self.right):
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
        self.left_s = self.robot.getDevice('left_wheel_joint_sensor')
        self.right_s = self.robot.getDevice('right_wheel_joint_sensor')
        self.left_s.enable(self.dt)
        self.right_s.enable(self.dt)
        self.lidar = self.robot.getDevice('rplidar')
        self.lidar.enable(lidar_period_ms)
        self.gps = self.robot.getDevice('p3d gps')
        self.imu = self.robot.getDevice('p3d inertial')
        self.gyro = self.robot.getDevice('p3d gyro')
        self.gps.enable(self.dt)
        self.imu.enable(self.dt)
        self.gyro.enable(self.dt)

        rclpy.init()
        self.node = rclpy.create_node('tb4_webots_bridge')
        self.node.create_subscription(Twist, 'cmd_vel_unstamped', self.on_cmd, 10)
        self.clock_pub = self.node.create_publisher(Clock, '/clock', 10)
        self.scan_pub = self.node.create_publisher(
            LaserScan, 'scan', qos_profile_sensor_data)
        self.odom_pub = self.node.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.node.create_publisher(
            Imu, 'imu', qos_profile_sensor_data)
        self.gt_pub = self.node.create_publisher(
            PoseStamped, 'ground_truth/pose', 10)
        self.tf = TransformBroadcaster(self.node)
        self.static_tf = StaticTransformBroadcaster(self.node)
        self.publish_static()

        self.cmd = (0.0, 0.0)
        self.x = self.y = self.th = 0.0
        self.prev = None
        self.origin = None  # (x, y, yaw) of the robot at start, world frame
        self.wp = 0
        self.slam_since = None
        self.waiting_logged = False
        self.stamp = None

    # -- ROS side ---------------------------------------------------------
    def on_cmd(self, msg):
        if not self.patrol:
            self.cmd = (msg.linear.x, msg.angular.z)

    def now(self):
        t = self.robot.getTime()
        return rclpy.time.Time(seconds=int(t),
                               nanoseconds=int((t % 1) * 1e9)).to_msg()

    def publish_static(self):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'rplidar_link'
        t.transform.translation.x, t.transform.translation.y, \
            t.transform.translation.z = LIDAR_XYZ
        q = yaw_quat(LIDAR_YAW)
        t.transform.rotation.x, t.transform.rotation.y, \
            t.transform.rotation.z, t.transform.rotation.w = q
        self.static_tf.sendTransform(t)

    # -- sensing ----------------------------------------------------------
    def ground_truth(self):
        gx, gy, gz = self.gps.getValues()
        yaw = self.imu.getRollPitchYaw()[2]
        if self.origin is None:
            self.origin = (gx, gy, gz, yaw)
        ox, oy, oz, oyaw = self.origin
        dx, dy = gx - ox, gy - oy
        c, s = math.cos(-oyaw), math.sin(-oyaw)
        return (c * dx - s * dy, s * dx + c * dy, gz - oz, wrap(yaw - oyaw),
                (gx, gy, yaw))

    def publish_imu(self, stamp):
        # Orientation in the world frame (like the Create 3 IMU), angular
        # velocity in the body frame; no accelerometer on the proto.
        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = 'base_link'
        qx, qy, qz, qw = self.imu.getQuaternion()
        m.orientation.x, m.orientation.y = qx, qy
        m.orientation.z, m.orientation.w = qz, qw
        wx, wy, wz = self.gyro.getValues()
        m.angular_velocity.x, m.angular_velocity.y = wx, wy
        m.angular_velocity.z = wz
        m.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(m)

    def update_odom(self):
        l, r = self.left_s.getValue(), self.right_s.getValue()
        if self.prev is not None:
            dl = (l - self.prev[0]) * WHEEL_RADIUS
            dr = (r - self.prev[1]) * WHEEL_RADIUS
            ds, dth = (dl + dr) / 2, (dr - dl) / WHEEL_SEPARATION
            self.x += ds * math.cos(self.th + dth / 2)
            self.y += ds * math.sin(self.th + dth / 2)
            self.th = wrap(self.th + dth)
        self.prev = (l, r)

        stamp = self.now()
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        q = yaw_quat(self.th)
        t.transform.rotation.x, t.transform.rotation.y, \
            t.transform.rotation.z, t.transform.rotation.w = q
        self.tf.sendTransform(t)

        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = 'odom'
        o.child_frame_id = 'base_link'
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = q[2], q[3]
        o.twist.twist.linear.x = self.cmd[0]
        o.twist.twist.angular.z = self.cmd[1]
        self.odom_pub.publish(o)

    def publish_scan(self):
        try:
            ranges = list(self.lidar.getRangeImage())
        except (ValueError, SystemError):
            return  # no measurement available yet
        if self.reverse_scan:
            # Webots orders lidar rays clockwise; LaserScan is counter-clockwise.
            ranges.reverse()
        n = len(ranges)
        fov = self.lidar.getFov()
        s = LaserScan()
        s.header.stamp = self.now()
        s.header.frame_id = 'rplidar_link'
        s.angle_min = -fov / 2
        s.angle_increment = fov / n
        s.angle_max = s.angle_min + s.angle_increment * (n - 1)
        s.range_min = float(self.lidar.getMinRange())
        s.range_max = float(self.lidar.getMaxRange())
        s.scan_time = self.lidar.getSamplingPeriod() / 1000.0
        s.time_increment = s.scan_time / n
        s.ranges = [r if math.isfinite(r) else float('inf') for r in ranges]
        self.scan_pub.publish(s)

    # -- control ----------------------------------------------------------
    def patrol_cmd(self, gx, gy, gyaw):
        """Go-to-goal on the ground-truth pose (world frame)."""
        if self.wp >= len(self.patrol):
            return 0.0, 0.0
        tx, ty = self.patrol[self.wp]
        dx, dy = tx - gx, ty - gy
        dist = math.hypot(dx, dy)
        if dist < 0.15:
            self.wp += 1
            if self.wp >= len(self.patrol):
                self.node.get_logger().info('Patrol finished')
            return 0.0, 0.0
        err = wrap(math.atan2(dy, dx) - gyaw)
        if abs(err) > 0.3:
            return 0.0, max(-0.6, min(0.6, 1.5 * err))
        return 0.25 * max(0.3, math.cos(err)), max(-0.5, min(0.5, 1.5 * err))

    def slam_ready(self):
        """Hold the patrol until SLAM publishes /map (+3 s): odometry that
        drifted before SLAM starts would tilt the whole map frame."""
        if self.slam_since is None:
            if self.node.count_publishers('/map') == 0:
                if not self.waiting_logged:
                    self.node.get_logger().info('Patrol waits for SLAM (/map)...')
                    self.waiting_logged = True
                return False
            self.slam_since = self.robot.getTime()
        return self.robot.getTime() - self.slam_since > 3.0

    def drive(self, v, w):
        vl = (v - w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
        vr = (v + w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
        self.left.setVelocity(max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, vl)))
        self.right.setVelocity(max(-MAX_WHEEL_SPEED, min(MAX_WHEEL_SPEED, vr)))

    def run(self):
        last_scan = -1
        while self.robot.step(self.dt) != -1:
            clock = Clock()
            clock.clock = self.now()
            self.clock_pub.publish(clock)

            rx, ry, rz, ryaw, (gx, gy, gyaw) = self.ground_truth()
            if self.patrol and self.slam_ready():
                self.cmd = self.patrol_cmd(gx, gy, gyaw)
            self.drive(*self.cmd)

            self.update_odom()
            gt = PoseStamped()
            gt.header.stamp = clock.clock
            gt.header.frame_id = 'map'
            gt.pose.position.x, gt.pose.position.y = rx, ry
            gt.pose.position.z = rz
            q = yaw_quat(ryaw)
            gt.pose.orientation.z, gt.pose.orientation.w = q[2], q[3]
            self.gt_pub.publish(gt)
            self.publish_imu(clock.clock)

            t = self.robot.getTime()
            period = self.lidar.getSamplingPeriod() / 1000.0
            if t - last_scan >= period - 1e-6:
                last_scan = t
                self.publish_scan()
            rclpy.spin_once(self.node, timeout_sec=0)
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--patrol', action='store_true',
                    help='drive the built-in waypoint loop, ignore /cmd_vel_unstamped')
    ap.add_argument('--layout', default='layout',
                    help='module in tools/ with the PATROL waypoints '
                         '(layout, layout_2floor)')
    ap.add_argument('--lidar-period-ms', type=int, default=100)
    ap.add_argument('--no-reverse-scan', dest='reverse_scan',
                    action='store_false',
                    help='publish Webots ranges in their native (clockwise) '
                         'order; this mirrors the scan, debugging only')
    a = ap.parse_args()
    patrol = importlib.import_module(a.layout).PATROL if a.patrol else None
    Bridge(patrol, a.lidar_period_ms, a.reverse_scan).run()
