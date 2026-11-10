#!/usr/bin/env python3
"""참값 / ESKF / 오도메트리 + 관절 + 단계로그를 한 CSV 로 (20 Hz)."""
import math, sys, time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from rcl_interfaces.msg import Log
from rclpy.qos import QoSProfile, ReliabilityPolicy

J = ["front_lift_joint", "rear_lift_joint", "body_turn_joint"]


def yaw(o):
    return math.degrees(math.atan2(2*(o.w*o.z+o.x*o.y), 1-2*(o.y*o.y+o.z*o.z)))


class R(Node):
    def __init__(self, path):
        super().__init__("rec2")
        self.f = open(path, "w")
        self.f.write("t,gt_x,gt_y,gt_z,gt_roll,gt_pitch,gt_yaw,"
                     "ek_x,ek_y,ek_yaw,od_x,od_y,od_yaw,"
                     "ax,ay,wz," + ",".join(J) + ",note\n")
        self.gt = self.ek = self.od = None
        self.imu = (0.0, 0.0, 0.0)
        self.j = {}
        self.note = ""
        self.t0 = time.time()
        q = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, "/ground_truth", self.on_gt, 20)
        self.create_subscription(Odometry, "/odom_eskf", self.on_ek, 20)
        self.create_subscription(Odometry, "/odom_slip", self.on_od, 20)
        self.create_subscription(Imu, "/imu", self.on_imu, q)
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(Log, "/rosout", self.on_log, 50)
        self.create_timer(0.05, self.tick)

    def on_gt(self, m):
        o = m.pose.pose.orientation; p = m.pose.pose.position
        roll = math.degrees(math.atan2(2*(o.w*o.x+o.y*o.z), 1-2*(o.x*o.x+o.y*o.y)))
        pit = math.degrees(math.asin(max(-1, min(1, 2*(o.w*o.y-o.z*o.x)))))
        self.gt = (p.x, p.y, p.z, roll, pit, yaw(o))

    def on_ek(self, m):
        self.ek = (m.pose.pose.position.x, m.pose.pose.position.y,
                   yaw(m.pose.pose.orientation))

    def on_od(self, m):
        self.od = (m.pose.pose.position.x, m.pose.pose.position.y,
                   yaw(m.pose.pose.orientation))

    def on_imu(self, m):
        self.imu = (m.linear_acceleration.x, m.linear_acceleration.y,
                    m.angular_velocity.z)

    def on_js(self, m):
        for n, p in zip(m.name, m.position):
            if n in J:
                self.j[n] = p

    def on_log(self, m):
        if m.name in ("jack_cycles", "patrol"):
            self.note = m.msg.replace(",", ";").replace("\n", " ")[:110]

    def tick(self):
        if self.gt is None or self.ek is None:
            return
        v = list(self.gt) + list(self.ek) + list(self.od or (0, 0, 0)) + list(self.imu)
        v += [self.j.get(k, float("nan")) for k in J]
        self.f.write(f"{time.time()-self.t0:.3f}," + ",".join(f"{x:.5f}" for x in v)
                     + f",{self.note}\n")
        self.note = ""


def main():
    rclpy.init(); n = R(sys.argv[1])
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 3600
    end = time.time() + dur
    try:
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    n.f.close(); print("saved", sys.argv[1], flush=True)

main()
