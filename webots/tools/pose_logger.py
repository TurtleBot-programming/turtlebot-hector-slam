#!/usr/bin/env python3
"""Log the SLAM pose (TF map -> base_link) against /ground_truth/pose.

  python3 pose_logger.py [pose_error.csv]

Both poses are relative to the robot's spawn pose, so they can be compared
directly. One row per second of sim time; feed the CSV to evaluate_map.py.
In the two-floor world it also logs the true height (gt_z) and the floor the
floor manager reports (/floor, 1 if it is not running).
Start it together with the SLAM launch and stop it with Ctrl+C.
"""
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return 2 * math.atan2(q.z, q.w)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'pose_error.csv'
    out = open(path, 'w')
    out.write('t,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw,err_xy,err_yaw,gt_z,floor\n')

    rclpy.init()
    node = rclpy.create_node('pose_logger', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    buf = Buffer()
    TransformListener(buf, node)
    last = [-1.0]
    floor = [1]
    node.create_subscription(
        Int32, 'floor', lambda m: floor.__setitem__(0, m.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def on_gt(m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t - last[0] < 1.0:
            return
        try:
            tf = buf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return
        last[0] = t
        g, e = m.pose, tf.transform
        gy, ey = yaw(g.orientation), yaw(e.rotation)
        exy = math.hypot(g.position.x - e.translation.x,
                         g.position.y - e.translation.y)
        eyaw = abs(math.atan2(math.sin(gy - ey), math.cos(gy - ey)))
        out.write(f'{t:.1f},{g.position.x:.3f},{g.position.y:.3f},{gy:.3f},'
                  f'{e.translation.x:.3f},{e.translation.y:.3f},{ey:.3f},'
                  f'{exy:.3f},{eyaw:.3f},{g.position.z:.3f},{floor[0]}\n')
        out.flush()

    node.create_subscription(PoseStamped, '/ground_truth/pose', on_gt, 10)
    node.get_logger().info(f'Logging pose error to {path}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    out.close()


if __name__ == '__main__':
    main()
