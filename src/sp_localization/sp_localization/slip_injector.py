#!/usr/bin/env python3
"""휠 오도메트리에 slip 을 주입한다.

    ros2 run sp_localization slip_injector

`/odom_enc`(휠 엔코더로 우리가 계산한 깨끗한 값)을 받아 오차를 섞어
`/odom_slip` 으로 낸다.
ESKF 는 `/odom_slip` 을 보고 돌고, 평가 노드는 셋(`/ground_truth`, `/odom_slip`,
`/odom_eskf`)을 나란히 비교한다.

━━ 왜 필요한가 ━━
gz `DiffDrive` 는 **이상적인 미끄러짐 없는 구동**을 가정한다. 그래서 시뮬의
휠 오도메트리가 사실상 완벽하고(실측 위치오차 0.0 mm), 센서융합이 이길 대상이
없다. 실제 크롤러 오차의 최대 원인인 slip 이 통째로 빠져 있는 것이다.
이 노드가 그 자리를 메운다.

━━ 무엇을 섞는가 ━━
세 가지 성질이 다른 오차를 겹친다. 성질이 다르면 필터가 다루는 방식도 다르다.

  1. 바퀴 반경 오차 (radius_error)  — **일정한 스케일 오차**
     마모·공차로 실제 반경이 가정과 다르면 거리가 항상 같은 비율로 틀어진다.
     부호가 안 바뀌므로 **거리에 비례해 누적**된다. 가장 지독하다.
  2. 상시 slip (slip_mean/std)      — 평균이 있는 잡음
     궤도가 늘 조금씩 미끄러진다. 평균분은 스케일 오차처럼 누적되고,
     흔들림분은 random walk 로 쌓인다.
  3. 돌발 slip (burst)              — 드물고 큰 이벤트
     Grit 를 밟거나 론지 이음매를 넘을 때 확 미끄러진다. 이건 필터가
     이상치로 걸러내야 하는 종류다.

⚠ 부호 규약: slip 은 **바퀴가 실제보다 더 돈다** = 엔코더가 거리를 **과대**
  보고한다. 그래서 v_enc = v_true x (1 + slip). 실제 크롤러에서 구동 중에는
  이 방향이 맞다 (제동 중에는 반대지만 여기서는 다루지 않는다).

⚠ 이 노드는 pose 도 같이 만든다. 오염된 속도를 그대로 적분해서 낸다 —
  실제 휠 오도메트리가 하는 일과 같다. 그래야 `/odom_slip` 의 pose 가
  "엔코더만 믿었을 때 어디라고 생각하는가" 를 정직하게 보여준다.
"""
from __future__ import annotations

import math
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SlipInjector(Node):
    def __init__(self) -> None:
        super().__init__("slip_injector")
        p = self.declare_parameter
        p("in_topic", "/odom_enc")
        p("out_topic", "/odom_slip")
        p("radius_error", 0.010)     # 바퀴 반경 스케일 오차 [-] (+1% = 1% 과대보고)
        p("slip_mean", 0.030)        # 상시 slip 평균 [-]
        p("slip_std", 0.020)         # 상시 slip 표준편차 [-]
        p("burst_prob", 0.002)       # 샘플당 돌발 slip 확률 [-]
        p("burst_ratio", 0.30)       # 돌발 시 slip 비율 [-]
        p("gyro_scale_err", 0.005)   # 엔코더 각속도 스케일 오차 [-]
        p("gyro_noise", 0.002)       # 엔코더 각속도 잡음 [rad/s]
        p("seed", 0)                 # 0 이면 매번 다르게

        g = lambda k: self.get_parameter(k).value           # noqa: E731
        self.k_r = g("radius_error")
        self.s_mu, self.s_sd = g("slip_mean"), g("slip_std")
        self.b_p, self.b_r = g("burst_prob"), g("burst_ratio")
        self.w_k, self.w_n = g("gyro_scale_err"), g("gyro_noise")
        seed = g("seed")
        self.rng = random.Random(seed if seed else None)

        self.x = self.y = self.psi = 0.0
        self.t: float | None = None
        self.dist_true = self.dist_enc = 0.0
        self.n_burst = 0

        self.pub = self.create_publisher(Odometry, g("out_topic"), 20)
        self.create_subscription(Odometry, g("in_topic"), self.on_odom, 20)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f"slip 주입 — {g('in_topic')} → {g('out_topic')}  "
            f"반경오차 {self.k_r:+.1%}, 상시 slip {self.s_mu:+.1%}±{self.s_sd:.1%}, "
            f"돌발 {self.b_p:.3%}/샘플 x {self.b_r:.0%}")

    def on_odom(self, m: Odometry) -> None:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t is None:
            self.t = t
            return
        dt = t - self.t
        self.t = t
        if dt <= 0.0 or dt > 0.5:
            return

        v_true = m.twist.twist.linear.x
        w_true = m.twist.twist.angular.z

        # ── slip 합성 ──
        slip = self.k_r + self.rng.gauss(self.s_mu, self.s_sd)
        if self.rng.random() < self.b_p:
            slip += self.b_r
            self.n_burst += 1
        v_enc = v_true * (1.0 + slip)
        w_enc = w_true * (1.0 + self.w_k) + self.rng.gauss(0.0, self.w_n)

        # ── 오염된 속도를 그대로 적분 (실제 휠 오도메트리가 하는 일) ──
        self.psi += w_enc * dt
        self.psi = (self.psi + math.pi) % (2 * math.pi) - math.pi
        self.x += v_enc * math.cos(self.psi) * dt
        self.y += v_enc * math.sin(self.psi) * dt
        self.dist_true += abs(v_true) * dt
        self.dist_enc += abs(v_enc) * dt

        o = Odometry()
        o.header = m.header
        o.child_frame_id = m.child_frame_id
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.z = math.sin(self.psi / 2.0)
        o.pose.pose.orientation.w = math.cos(self.psi / 2.0)
        o.twist.twist.linear.x = v_enc
        o.twist.twist.angular.z = w_enc
        self.pub.publish(o)

    def report(self) -> None:
        if self.dist_true < 1e-6:
            return
        err = self.dist_enc - self.dist_true
        self.get_logger().info(
            f"참 주행 {self.dist_true:6.2f} m / 엔코더 보고 {self.dist_enc:6.2f} m "
            f"({err*1000:+7.1f} mm, {err/self.dist_true:+.2%}) | 돌발 slip {self.n_burst}회")


def main() -> None:
    rclpy.init()
    node = SlipInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
