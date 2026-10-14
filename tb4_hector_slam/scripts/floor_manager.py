#!/usr/bin/env python3
"""Multi-floor mapping on top of 2D Hector SLAM.

Hector maps one plane. To map a building with several floors, one
hector_mapping instance runs per floor (namespaces floor1, floor2, ...), all
in the same `map` frame, and this node decides which one gets the scans:

  floor detection  IMU pitch vs a learned flat-ground baseline says "on a
                   ramp"; while on it, height is integrated from odometry
                   (dz = ds * sin(slope)). Leaving the ramp with |dz| > half
                   a floor height means the robot changed floor.
  scan routing     /scan -> /floor<N>/scan for the current floor only, and
                   -> /scan_flat (for the Nav2 costmaps) only off-ramp: a
                   tilted scan plane sees the ramp, floor or ceiling.
  map relay        /floor<N>/map of the current floor -> /map (Nav2, RViz,
                   frontier explorer).
  localisation     map -> odom from the current floor's slam_out_pose. Held
                   while on a ramp, so odometry carries the pose across; the
                   next floor's Hector starts from that pose (it uses the TF
                   start estimate), so all floor maps share one frame.

Publishes /floor (Int32) and /on_ramp (Bool), both latched; clears the Nav2
costmaps after a floor change.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Int32
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """2D pose composition a * b, poses as (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class FloorManager(Node):

    def __init__(self):
        super().__init__('floor_manager')
        self.floors = self.declare_parameter('floors', 2).value
        self.floor_height = self.declare_parameter('floor_height', 0.6).value
        self.floor = self.declare_parameter('start_floor', 1).value
        self.ramp_tilt = math.radians(self.declare_parameter('ramp_tilt_deg', 3.5).value)
        self.flat_tilt = math.radians(self.declare_parameter('flat_tilt_deg', 1.5).value)
        self.ramp_confirm = self.declare_parameter('ramp_confirm_time', 0.8).value
        self.flat_confirm = self.declare_parameter('flat_confirm_time', 1.0).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        # map -> odom is stamped this far ahead, like slam_toolbox's
        # transform_timeout, so lookups at the latest odom time succeed.
        self.tf_lead = self.declare_parameter('transform_lead', 0.1).value

        self.baseline = None        # IMU pitch on flat ground [rad]
        self.slope = 0.0            # current slope, nose up positive [rad]
        self.on_ramp = False
        self.tilt_since = None      # when the pitch first left the flat band
        self.flat_since = None      # when the pitch came back to it (on a ramp)
        self.dz = 0.0               # height gained on the current ramp [m]
        self.last_odom = None
        self.correction = (0.0, 0.0, 0.0)   # map -> odom
        self.maps = {}

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)

        self.scan_pubs = {f: self.create_publisher(LaserScan, f'floor{f}/scan',
                                                   qos_profile_sensor_data)
                          for f in range(1, self.floors + 1)}
        self.flat_scan_pub = self.create_publisher(LaserScan, 'scan_flat',
                                                   qos_profile_sensor_data)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', LATCHED)
        self.floor_pub = self.create_publisher(Int32, 'floor', LATCHED)
        self.ramp_pub = self.create_publisher(Bool, 'on_ramp', LATCHED)
        for f in range(1, self.floors + 1):
            # volatile: matches Hector's map publisher whatever its durability
            self.create_subscription(OccupancyGrid, f'floor{f}/map',
                                     lambda m, f=f: self.on_map(f, m), 1)
            self.create_subscription(PoseStamped, f'floor{f}/slam_out_pose',
                                     lambda m, f=f: self.on_pose(f, m), 10)
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom, 20)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.clear_clients = [
            self.create_client(ClearEntireCostmap, f'{c}/clear_entirely_{c}')
            for c in ('global_costmap', 'local_costmap')]
        self.create_timer(0.05, self.publish_tf)
        self.publish_state()
        self.get_logger().info(f'Floor manager started on floor {self.floor}/{self.floors}')

    # -- floor detection ----------------------------------------------------
    def on_imu(self, msg):
        q = msg.orientation
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        now = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.baseline is None:
            self.baseline = pitch
            return
        dev = pitch - self.baseline
        self.slope = -dev       # ROS pitch is positive nose-down

        if not self.on_ramp:
            if abs(dev) > self.ramp_tilt:
                if self.tilt_since is None:
                    self.tilt_since = now
                elif now - self.tilt_since > self.ramp_confirm:
                    self.on_ramp = True
                    self.dz = 0.0
                    self.flat_since = None
                    self.get_logger().info(
                        f'Ramp detected (slope {math.degrees(self.slope):+.1f} deg); '
                        f'pausing floor {self.floor} mapping')
                    self.publish_state()
            else:
                self.tilt_since = None
                if abs(dev) < self.flat_tilt:
                    self.baseline += 0.01 * dev     # follow slow drift
        else:
            if abs(dev) < self.flat_tilt:
                if self.flat_since is None:
                    self.flat_since = now
                elif now - self.flat_since > self.flat_confirm:
                    self.leave_ramp()
            else:
                self.flat_since = None

    def on_odom(self, msg):
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.on_ramp and self.last_odom is not None:
            dx, dy = pose[0] - self.last_odom[0], pose[1] - self.last_odom[1]
            ds = dx * math.cos(pose[2]) + dy * math.sin(pose[2])   # signed
            self.dz += ds * math.sin(self.slope)
        self.last_odom = pose

    def leave_ramp(self):
        self.on_ramp = False
        self.tilt_since = None
        # Ramps join adjacent floors. The climb estimate also contains the
        # robot's lean, so only the sign of a large height change is trusted.
        step = 0
        if self.dz > self.floor_height / 2:
            step = 1
        elif self.dz < -self.floor_height / 2:
            step = -1
        new = min(max(self.floor + step, 1), self.floors)
        self.get_logger().info(f'Off the ramp: dz {self.dz:+.2f} m -> floor {new}')
        if new != self.floor:
            self.floor = new
            self.switch_map()
            self.clear_costmaps()
        self.publish_state()

    # -- routing --------------------------------------------------------------
    def on_scan(self, msg):
        if self.on_ramp or self.baseline is None:
            return
        self.scan_pubs[self.floor].publish(msg)
        self.flat_scan_pub.publish(msg)

    def on_map(self, floor, msg):
        self.maps[floor] = msg
        if floor == self.floor:
            self.map_pub.publish(msg)

    def switch_map(self):
        if self.floor in self.maps:
            self.map_pub.publish(self.maps[self.floor])
        elif self.maps:
            # Not mapped yet: same grid, all unknown, until its Hector publishes.
            blank = OccupancyGrid()
            ref = next(iter(self.maps.values()))
            blank.header = ref.header
            blank.header.stamp = self.get_clock().now().to_msg()
            blank.info = ref.info
            blank.data = [-1] * len(ref.data)
            self.map_pub.publish(blank)

    def clear_costmaps(self):
        for c in self.clear_clients:
            if c.service_is_ready():
                c.call_async(ClearEntireCostmap.Request())

    def publish_state(self):
        self.floor_pub.publish(Int32(data=self.floor))
        self.ramp_pub.publish(Bool(data=self.on_ramp))

    # -- localisation ---------------------------------------------------------
    def on_pose(self, floor, msg):
        if floor != self.floor or self.on_ramp:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(f'odom -> base lookup failed: {e}',
                                   throttle_duration_sec=5.0)
            return
        tr = t.transform
        odom_base = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
        p = msg.pose
        map_base = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.correction = compose(map_base, inverse(odom_base))

    def publish_tf(self):
        x, y, yaw = self.correction
        t = TransformStamped()
        t.header.stamp = (self.get_clock().now() +
                          Duration(seconds=self.tf_lead)).to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        self.tf_pub.sendTransform(t)


def main():
    rclpy.init()
    node = FloorManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
