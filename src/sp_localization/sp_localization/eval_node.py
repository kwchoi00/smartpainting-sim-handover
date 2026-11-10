#!/usr/bin/env python3
"""추정 결과를 참값과 비교한다 — 이 과제가 실제로 재려는 것.

    ros2 run sp_localization eval
    ros2 run sp_localization eval --ros-args -p csv:=/tmp/run1.csv

세 궤적을 같은 기준으로 놓고 오차를 잰다.

    /odom_slip   휠 오도메트리 단독      (엔코더만, slip 주입됨)
    /odom_eskf   IMU + 엔코더 ESKF       (센서융합)
    /ground_truth 참값                   ← **평가 전용. 추정에 절대 쓰지 않는다**

RViz 로 보려면 세 궤적을 `nav_msgs/Path` 로도 낸다:
    /path_gt   /path_odom   /path_eskf

⚠ 세 토픽의 기준 프레임이 다를 수 있다. `/ground_truth` 는 월드 절대좌표,
  `/odom` 계열은 스폰 지점 기준이다. 그래서 **첫 샘플로 정렬(alignment)** 한 뒤
  상대 궤적으로 비교한다. 드리프트 평가는 상대 비교라 이걸로 충분하다.

목표는 "정확한 위치"가 아니라 **"허용 오차를 넘기까지의 시간·거리"** 다.
절대 위치 관측이 없으므로 드리프트는 유계가 아니고, 튜닝으로 없앨 수 없다.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


def yaw_of(o) -> float:
    return math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                      1.0 - 2.0 * (o.y * o.y + o.z * o.z))


class Track:
    """첫 샘플을 원점으로 정렬해 상대 궤적을 들고 있는다."""

    def __init__(self) -> None:
        self.ref: tuple[float, float, float] | None = None
        self.p = (0.0, 0.0, 0.0)
        self.dist = 0.0
        self._last: tuple[float, float] | None = None

    def update(self, x: float, y: float, psi: float) -> None:
        if self.ref is None:
            self.ref = (x, y, psi)
        rx, ry, rpsi = self.ref
        c, s = math.cos(-rpsi), math.sin(-rpsi)
        dx, dy = x - rx, y - ry
        self.p = (c * dx - s * dy, s * dx + c * dy,
                  (psi - rpsi + math.pi) % (2 * math.pi) - math.pi)
        if self._last is not None:
            self.dist += math.hypot(self.p[0] - self._last[0], self.p[1] - self._last[1])
        self._last = (self.p[0], self.p[1])


class Eval(Node):
    def __init__(self) -> None:
        super().__init__("eval")
        self.declare_parameter("csv", "")
        self.declare_parameter("period", 2.0)
        self.declare_parameter("odom_topic", "/odom_slip")
        self.declare_parameter("path_frame", "odom")
        self.gt, self.od, self.ek = Track(), Track(), Track()
        self.paths = {n: Path() for n in ("gt", "odom", "eskf")}
        self.path_pub = {n: self.create_publisher(Path, f"/path_{n}", 5)
                         for n in self.paths}
        self.pframe = self.get_parameter("path_frame").value
        self.peak = {"odom": [0.0, 0.0], "eskf": [0.0, 0.0]}   # [위치오차, yaw오차]
        od_topic = self.get_parameter("odom_topic").value
        self.create_subscription(Odometry, "/ground_truth", self.cb(self.gt, "gt"), 20)
        self.create_subscription(Odometry, od_topic, self.cb(self.od, "odom"), 20)
        self.create_subscription(Odometry, "/odom_eskf", self.cb(self.ek, "eskf"), 20)

        path = self.get_parameter("csv").value
        self.f = open(path, "w") if path else None
        if self.f:
            self.f.write("t,gt_x,gt_y,gt_yaw,odom_x,odom_y,odom_yaw,"
                         "eskf_x,eskf_y,eskf_yaw,odom_err,eskf_err\n")
        self.create_timer(self.get_parameter("period").value, self.report)
        self.t0 = self.get_clock().now()
        self.get_logger().info("평가 시작 — /ground_truth 대비 /odom, /odom_eskf")

    def cb(self, tr: Track, name: str):
        def f(m: Odometry) -> None:
            tr.update(m.pose.pose.position.x, m.pose.pose.position.y,
                      yaw_of(m.pose.pose.orientation))
            # RViz 용 Path. 매 샘플 다 넣으면 무거워서 2 cm 이상 움직였을 때만 찍는다.
            pa = self.paths[name]
            if pa.poses:
                q = pa.poses[-1].pose.position
                if math.hypot(tr.p[0] - q.x, tr.p[1] - q.y) < 0.02:
                    return
            ps = PoseStamped()
            ps.header.stamp = m.header.stamp
            ps.header.frame_id = self.pframe
            ps.pose.position.x, ps.pose.position.y = tr.p[0], tr.p[1]
            ps.pose.orientation.z = math.sin(tr.p[2] / 2.0)
            ps.pose.orientation.w = math.cos(tr.p[2] / 2.0)
            pa.header = ps.header
            pa.poses.append(ps)
            if len(pa.poses) > 5000:
                pa.poses.pop(0)
            self.path_pub[name].publish(pa)
        return f

    @staticmethod
    def err(a: Track, b: Track) -> tuple[float, float]:
        return (math.hypot(a.p[0] - b.p[0], a.p[1] - b.p[1]),
                (a.p[2] - b.p[2] + math.pi) % (2 * math.pi) - math.pi)

    def report(self) -> None:
        if self.gt.ref is None:
            return
        eo, yo = self.err(self.gt, self.od)
        ee, ye = self.err(self.gt, self.ek)
        self.peak["odom"][0] = max(self.peak["odom"][0], eo)
        self.peak["odom"][1] = max(self.peak["odom"][1], abs(yo))
        self.peak["eskf"][0] = max(self.peak["eskf"][0], ee)
        self.peak["eskf"][1] = max(self.peak["eskf"][1], abs(ye))
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        self.get_logger().info(
            f"주행 {self.gt.dist:6.2f} m | "
            f"오도메트리 {eo*1000:7.1f} mm {math.degrees(yo):+6.2f}° | "
            f"ESKF {ee*1000:7.1f} mm {math.degrees(ye):+6.2f}° | "
            f"최대 odom {self.peak['odom'][0]*1000:.0f} mm / "
            f"eskf {self.peak['eskf'][0]*1000:.0f} mm")

        if self.f:
            self.f.write(
                f"{t:.3f},{self.gt.p[0]:.5f},{self.gt.p[1]:.5f},{self.gt.p[2]:.5f},"
                f"{self.od.p[0]:.5f},{self.od.p[1]:.5f},{self.od.p[2]:.5f},"
                f"{self.ek.p[0]:.5f},{self.ek.p[1]:.5f},{self.ek.p[2]:.5f},"
                f"{eo:.5f},{ee:.5f}\n")
            self.f.flush()


def main() -> None:
    rclpy.init()
    node = Eval()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.f:
            node.f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
