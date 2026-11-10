#!/usr/bin/env python3
"""휠 엔코더(회전량) → 속도 → 오도메트리.  `imu_odom.txt` 의 1→2→3 단계 그대로.

    ros2 run sp_localization wheel_odom

`/joint_states` 의 구동륜 관절각(= 엔코더 원시 회전량)만 보고 계산한다.
gz `DiffDrive` 가 내주는 `/odom` 은 쓰지 않는다 — 그건 시뮬레이터가 대신 풀어준
값이라 **엔코더가 실제로 무엇을 아는지**가 가려진다.

━━ 센서 배치 (이상적 배치 기준) ━━
    IMU   : 바퀴부 중심 (0, 0)  — 회전 안 하는 몸체, 레버암 0
    엔코더: 바퀴부의 **좌우 바퀴 양쪽**, 궤간 = 론지 pitch 0.845

━━ 계산 ━━
  1. 회전량   각 바퀴의 관절각 theta [rad] ← 엔코더가 직접 재는 유일한 값
  2. 속도     좌/우를 각각 평균낸 뒤
                 v = R (w_L + w_R) / 2          선속도
                 w = R (w_R - w_L) / gauge      각속도
              **누적 오차가 없는 순간값**이다. ESKF 가 관측으로 쓰는 게 이것.
  3. 위치     v, w 를 적분. **여기서부터 누적 오차가 생긴다.**
              참고용으로 pose 에 실어 보내지만 ESKF 는 쓰지 않는다.

━━ 왜 한쪽에 8개씩인가 ━━
같은 궤도 위 바퀴들은 기구학적으로 같은 값을 봐야 한다. 평균을 내면 개별 바퀴의
접촉 잡음이 1/sqrt(N) 로 줄고, 한 바퀴가 튀면 (헛돎) 바로 드러난다.
`spread` 로 좌우 각각의 편차를 같이 낸다 — 이게 크면 바퀴 하나가 미끄러진 것이다.

⚠ 엔코더 분해능을 양자화로 넣는다 (`ticks_per_rev`). 실물 엔코더는 연속값을
  주지 않는다. 저속에서 이 양자화가 속도 잡음의 지배적 원인이 되는 경우가 많다.

⚠ **출력 주기가 곧 속도 잡음이다 — `rate` 를 함부로 올리지 말 것.**
  gz 의 `/joint_states` 는 물리 스텝마다, 즉 **1000 Hz** 로 온다. 예전에는 그걸
  그대로 받아 1 ms 차분으로 속도를 냈는데, 그러면 양자화 잡음이 이렇게 커진다:

      차분 구간   1 tick 이 만드는 속도 계단     0.2 m/s 에서의 잡음
      1 ms (1000 Hz)   0.093 m/s                +-0.047 m/s  (24 %)
      20 ms (50 Hz)    0.0047 m/s               +-0.0023 m/s (1.2 %)

  ESKF 는 이 관측을 `sigma_enc` 0.02 m/s 로 믿는데, 1000 Hz 에서는 실제 잡음이
  그 2.4 배였다. 게다가 관측이 설계(50 Hz)의 **20 배**로 들어와 필터가 엔코더를
  과신했다 (실측: 보정 횟수가 IMU 표본의 5 배 — 200 Hz IMU 에 1000 Hz 보정).
  그래서 여기서 **50 Hz 로 솎아내고, 차분도 그 간격으로** 한다. `odom_rate`
  (params.xacro) 와 같은 값이다.
"""
from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WheelOdom(Node):
    def __init__(self) -> None:
        super().__init__("wheel_odom")
        p = self.declare_parameter
        p("wheel_radius", 0.0609)     # 접지면~축 = 유효 구름반경 [m]
        p("gauge", 0.845)             # 좌우 궤간 [m] = 론지 pitch
        p("ticks_per_rev", 4096)      # 엔코더 분해능. 0 이면 양자화 안 함
        p("rate", 50.0)               # 출력 주기 [Hz]. 0 이면 /joint_states 그대로
        p("out_topic", "/odom_enc")
        p("frame_id", "odom")
        p("child_frame_id", "base_footprint")

        g = lambda k: self.get_parameter(k).value          # noqa: E731
        self.R, self.gauge = g("wheel_radius"), g("gauge")
        self.tpr = g("ticks_per_rev")
        self.rate = g("rate")
        self.min_dt = (1.0 / self.rate) if self.rate > 0.0 else 0.0
        self.frame, self.child = g("frame_id"), g("child_frame_id")

        self.prev: dict[str, float] = {}
        self.t: float | None = None
        self.x = self.y = self.psi = 0.0
        self.spread_max = 0.0
        self.n = 0

        self.pub = self.create_publisher(Odometry, g("out_topic"), 20)
        self.create_subscription(JointState, "/joint_states", self.on_js, 20)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"휠 엔코더 오도메트리 — 반경 {self.R:.4f} m, 궤간 {self.gauge:.3f} m, "
            f"분해능 {self.tpr} tick/rev, {self.rate:.0f} Hz → {g('out_topic')}")

    def quant(self, a: float) -> float:
        """엔코더 분해능으로 각도를 양자화한다."""
        if not self.tpr:
            return a
        step = 2.0 * math.pi / self.tpr
        return round(a / step) * step

    def on_js(self, m: JointState) -> None:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        cur = {n: self.quant(p) for n, p in zip(m.name, m.position)
               if n.endswith("_wheel_joint")}
        if not cur:
            return
        if self.t is None or not self.prev:
            self.t, self.prev = t, cur
            return
        dt = t - self.t
        if dt <= 0.0 or dt > 0.5:
            self.t, self.prev = t, cur
            return
        # ⚠ **여기서 self.prev 를 갱신하지 않고 그냥 돌아간다.** 그래야 다음 번에
        #   차분 구간이 min_dt 만큼 벌어져서 양자화 잡음이 그만큼 줄어든다.
        #   (매번 갱신하면 솎아내도 차분은 여전히 1 ms 다.)
        if dt < self.min_dt:
            return

        # ── 1. 회전량 → 각 바퀴의 각속도 ──
        wl, wr = [], []
        for n, a in cur.items():
            if n not in self.prev:
                continue
            w = (a - self.prev[n]) / dt
            (wl if n.startswith("l") else wr).append(w)
        self.t, self.prev = t, cur
        if not wl or not wr:
            return

        aL, aR = sum(wl) / len(wl), sum(wr) / len(wr)
        # 같은 궤도 위 바퀴는 같은 값을 봐야 한다. 벌어지면 하나가 헛돈 것이다.
        spread = max(max(wl) - min(wl), max(wr) - min(wr)) * self.R
        self.spread_max = max(self.spread_max, spread)

        # ── 2. 속도 (누적 오차 없는 순간값) ──
        v = self.R * (aL + aR) / 2.0
        w = self.R * (aR - aL) / self.gauge

        # ── 3. 위치 (여기서부터 누적된다 — 참고용) ──
        self.psi = (self.psi + w * dt + math.pi) % (2 * math.pi) - math.pi
        self.x += v * math.cos(self.psi) * dt
        self.y += v * math.sin(self.psi) * dt
        self.n += 1

        o = Odometry()
        o.header.stamp = m.header.stamp
        o.header.frame_id = self.frame
        o.child_frame_id = self.child
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.psi / 2.0)
        o.pose.pose.orientation.w = math.cos(self.psi / 2.0)
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = w
        self.pub.publish(o)

    def report(self) -> None:
        if not self.n:
            return
        self.get_logger().info(
            f"샘플 {self.n} | 적분 위치 x {self.x:+.3f} y {self.y:+.3f} "
            f"psi {math.degrees(self.psi):+.2f}° | "
            f"좌우 내 바퀴 편차 최대 {self.spread_max*1000:.1f} mm/s")


def main() -> None:
    rclpy.init()
    node = WheelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
