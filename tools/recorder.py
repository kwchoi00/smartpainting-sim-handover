#!/usr/bin/env python3
"""덱 출렁임 계측기 — 참값 자세 + 관절값 + patrol 단계 로그를 한 CSV 로 모은다.

    python3 recorder.py out.csv [seconds]

덱의 **세계좌표 yaw** = 바퀴부 yaw(참값) + body_turn 관절각. 화면에서 보이는 값이 이것이다.
"""
import math, sys, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import Log

COLS = ["front_turn_joint","rear_turn_joint","body_turn_joint",
        "front_lift_joint","rear_lift_joint","body_lift_joint"]

class Rec(Node):
    def __init__(self, path, dur):
        super().__init__("wobble_rec")
        self.f = open(path, "w")
        self.f.write("t,x,y,z,roll,pitch,base_yaw,deck_yaw," + ",".join(COLS) + ",note\n")
        self.j = {}
        self.t0 = time.time()
        self.dur = dur
        self.note = ""
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.create_subscription(Odometry, "/ground_truth", self.on_gt, 50)
        self.create_subscription(Log, "/rosout", self.on_log, 50)

    def on_js(self, m):
        for n, p in zip(m.name, m.position):
            if n in COLS:
                self.j[n] = p

    def on_log(self, m):
        if m.name in ("patrol", "maneuver"):
            self.note = m.msg.replace(",", ";").replace("\n", " ")[:120]

    def on_gt(self, m):
        q = m.pose.pose.orientation
        yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        roll = math.degrees(math.atan2(2*(q.w*q.x+q.y*q.z), 1-2*(q.x*q.x+q.y*q.y)))
        pit = math.degrees(math.asin(max(-1, min(1, 2*(q.w*q.y-q.z*q.x)))))
        p = m.pose.pose.position
        bt = math.degrees(self.j.get("body_turn_joint", float("nan")))
        vals = ",".join(f"{math.degrees(self.j[c]) if c.endswith('turn_joint') else self.j[c]:.4f}"
                        if c in self.j else "" for c in COLS)
        self.f.write(f"{time.time()-self.t0:.3f},{p.x:.4f},{p.y:.4f},{p.z:.4f},"
                     f"{roll:.4f},{pit:.4f},{yaw:.4f},{yaw+bt:.4f},{vals},{self.note}\n")
        self.note = ""

def main():
    path = sys.argv[1]
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 600.0
    rclpy.init()
    n = Rec(path, dur)
    end = time.time() + dur
    try:
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    n.f.close()
    print("saved", path)

main()
