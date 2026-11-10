#!/usr/bin/env python3
"""앞뒤 왕복만 시키는 최소 주행기 — 스케일 추정 검증용.

    python3 drive_cycles.py [왕복횟수] [거리 m] [속도 m/s]
"""
import math, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

class Drv(Node):
    def __init__(self):
        super().__init__("drive_cycles")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.gt = None
        self.create_subscription(Odometry, "/ground_truth", self.on_gt, 20)
    def on_gt(self, m):
        self.gt = (m.pose.pose.position.x, m.pose.pose.position.y)
    def rest(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
    def go(self, dist, v):
        while self.gt is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        x0, y0 = self.gt
        sgn = 1.0 if dist > 0 else -1.0
        t0 = time.time()
        while time.time() - t0 < abs(dist) / v * 40 + 30:
            x, y = self.gt
            if math.hypot(x - x0, y - y0) >= abs(dist):
                break
            m = Twist(); m.linear.x = sgn * v
            self.pub.publish(m)
            self.rest(0.05)
        for _ in range(30):
            self.pub.publish(Twist()); self.rest(0.05)
        x, y = self.gt
        self.get_logger().info(f"{dist:+.2f} m 지시 -> 참값 이동 {math.hypot(x-x0,y-y0):.3f} m")

def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    d = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    v = float(sys.argv[3]) if len(sys.argv) > 3 else 0.2
    rclpy.init(); node = Drv()
    node.rest(2.0)
    for i in range(n):
        node.get_logger().info(f"-- {i+1}/{n} 왕복 --")
        node.go(+d, v); node.rest(3.0)
        node.go(-d, v); node.rest(3.0)
    node.get_logger().info("끝")
    node.destroy_node(); rclpy.shutdown()

main()
