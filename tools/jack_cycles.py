#!/usr/bin/env python3
"""잭업 → 바퀴부 90° 회전만 반복시킨다 (주행 없음). C2/E5 구간 계측용.

    python3 jack_cycles.py [반복]
"""
import math, sys, time
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JOINT_OF = {"front_lift": "front_lift_joint", "rear_lift": "rear_lift_joint",
            "front_turn": "front_turn_joint", "rear_turn": "rear_turn_joint",
            "body_turn": "body_turn_joint"}
CLEAR, JACK, HALF = 0.640, -0.080, 1.5708


class Jack(Node):
    def __init__(self):
        super().__init__("jack_cycles")
        self.pub = {t: self.create_publisher(Float64, f"/{t}", 10) for t in JOINT_OF}
        self.pos = {}
        self.gt = None
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(Odometry, "/ground_truth", self.on_gt, 20)

    def on_js(self, m):
        for n, p in zip(m.name, m.position):
            self.pos[n] = p

    def on_gt(self, m):
        q = m.pose.pose.orientation
        self.gt = (m.pose.pose.position.x, m.pose.pose.position.y,
                   m.pose.pose.position.z,
                   math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))

    def rest(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def go(self, targets, label, tol_lin=0.01, tol_ang=0.02, cap=180.0):
        self.get_logger().info("▶ " + label + ": " +
                               ", ".join(f"{k}={v:+.3f}" for k, v in targets.items()))
        t0 = time.time()
        best, last = float("inf"), t0
        while time.time() - t0 < cap:
            for k, v in targets.items():
                m = Float64(); m.data = float(v); self.pub[k].publish(m)
            self.rest(0.2)
            worst, done = 0.0, True
            for k, v in targets.items():
                cur = self.pos.get(JOINT_OF[k])
                if cur is None:
                    done = False; continue
                e = abs(cur - v); worst = max(worst, e)
                if e > (tol_ang if k.endswith("turn") else tol_lin):
                    done = False
            if done:
                break
            if worst < best - 1e-4:
                best, last = worst, time.time()
            elif time.time() - last > 25.0:
                self.get_logger().warn(f"   진전 없음 (최대오차 {worst:.4f})")
                break
        g = self.gt or (0, 0, 0, 0)
        self.get_logger().info(
            f"   완료 ({time.time()-t0:.1f}s) [참값 x {g[0]:+.3f} y {g[1]:+.3f} "
            f"z {g[2]:.4f} yaw {g[3]:+7.2f}°]")


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    rclpy.init(); nd = Jack(); nd.rest(2.0)
    # A: 종주행 자세 (전후단부를 옆 통로로)
    nd.go({"front_lift": CLEAR, "rear_lift": CLEAR}, "A1 전후단부 들기")
    nd.go({"front_turn": -HALF, "rear_turn": -HALF}, "A2 전후단부 −90°")
    nd.go({"body_turn": HALF}, "A3 덱 +90°")
    nd.go({"front_lift": 0.0, "rear_lift": 0.0}, "A4 통로 바닥에 내림")
    nd.rest(4.0)
    for i in range(n):
        nd.get_logger().info(f"── {i+1}/{n} 잭업 회전 ──")
        nd.go({"front_lift": JACK, "rear_lift": JACK}, "C1 잭업")
        nd.rest(3.0)
        nd.go({"body_turn": 0.0}, "C2 바퀴부 +90°")
        nd.rest(3.0)
        nd.go({"front_lift": 0.0, "rear_lift": 0.0}, "C3 착지")
        nd.rest(4.0)
        nd.go({"front_lift": JACK, "rear_lift": JACK}, "E4 잭업")
        nd.rest(3.0)
        nd.go({"body_turn": HALF}, "E5 바퀴부 −90°")
        nd.rest(3.0)
        nd.go({"front_lift": 0.0, "rear_lift": 0.0}, "E6 착지")
        nd.rest(4.0)
    nd.get_logger().info("끝")
    nd.destroy_node(); rclpy.shutdown()

main()
