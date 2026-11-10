#!/usr/bin/env python3
"""시저 리프트(X자 팬터그래프) 링키지를 상하축 행정에 맞춰 접고 편다.

    ros2 run sp_gazebo scissor_driver.py

`sim.launch.py` 가 로봇과 같이 띄운다. 따로 실행할 일은 디버깅할 때뿐이다.

━━ 이 노드가 왜 있는가 ━━
시저는 **닫힌 루프**라 URDF 로 직접 표현할 수 없다. 그래서 모델을 이렇게 나눴다.

  · 하중과 행정 : 지금까지대로 prismatic 관절(`body_lift` `front_lift` `rear_lift`)이
                  전부 담당한다. 지령 단위(m)도 제어 게인도 측정 인터페이스도 그대로다.
  · X자 바      : 그 행정을 따라가는 **종동 링키지**. 충돌체가 없고 하중을 받지 않는다.
                  이 노드가 각도를 풀어 위치제어로 넣는다.

⚠ 원래는 mimic 관절로 묶으면 노드 없이 끝난다. 그런데 **dartsim 이 mimic 을
  지원하지 않는다** — gz 가 이렇게 말한다:
      "Attempting to create a mimic constraint for joint [...] but the chosen
       physics engine does not support mimic constraints"
  bullet-featherstone 은 지원하지만, 엔진을 바꾸면 접촉·마찰 모델이 달라져
  이 저장소의 존재 이유인 드리프트 측정이 통째로 무효가 된다. 그래서 종동 제어다.

⚠ 이 노드를 안 띄워도 시뮬은 정상 동작한다. X자가 접힌 채(θ=0) 있을 뿐이고,
  주행·오도메트리·ESKF 는 아무 영향을 받지 않는다.

━━ 푸는 식 ━━
바 길이 L, 단수 N, 바 각도 θ 인 시저 스택의 높이는

      h_stack = N · L · sinθ

이므로 필요한 신장량 ext 에서 역산한다:

      θ = asin( ext / (N · L) )

바들은 끝핀·중간핀으로 엮여 있어서 **관절값이 θ 와 2θ 두 가지뿐**이다
(A 바 절대각 +θ, B 바 −θ → 이웃한 두 바의 상대각은 언제나 2θ).
부호는 URDF 의 <axis> 가 흡수하므로 크기만 실어 보내면 된다.

      ns_m0_scissor_joint  →  θ    (조립체마다 뿌리 바 하나)
      그 외 모든 바         →  2θ

관절 20개를 **JointTrajectory 하나**로 묶어 보낸다. 관절마다 Float64 토픽을 두고
JointPositionController 를 20개 다는 방법도 되고 **성능 차이도 없다**
(실측 RTF 0.185 대 0.19). 궤적 하나로 묶는 이유는 20관절이 **한 틱에 함께**
움직여야 X자가 어긋나 보이지 않기 때문이다.

━━ 신장량 ext 는 행정 q 와 어떻게 이어지는가 ━━
  몸체부   ext = ext0 + q          q 가 커질수록 **펼쳐진다** (덱을 밀어 올린다)
  전후단부 ext = ext_hi − q        q 가 커질수록 **접힌다** (전후단부를 끌어 올린다)

전후단부 쪽 부호가 반대인 이유: 이 시저는 **윗부분(회전부)에 매달려 아랫부분(궤도)을
내리는** 기구다. q=0(아랫부분이 바닥) 이 최대 신장이고, 들어올릴수록 접힌다.
윗부분은 몸체부에 붙어 회전만 하고 내려가지 않는다.

⚠ 아래 기본값은 `sp_description/urdf/params.xacro` 의 시저 절과 **반드시 같아야
  한다.** 한쪽만 바꾸면 X자 상단이 플랫폼에서 떨어지거나 파고든다.
  (params.xacro 를 고쳤으면 여기 기본값도 같이 고칠 것.)
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ScissorDriver(Node):
    def __init__(self) -> None:
        super().__init__("scissor_driver")

        p = self.declare_parameter
        # ── params.xacro 의 시저 절과 같은 값이어야 한다 ──
        p("body_n", 2)            # 몸체부 시저 단수
        p("body_len", 1.550)      # 바 길이 [m] — 바퀴부 앞뒤 길이에 맞춤
        p("body_ext0", 0.110)     # q=0 에서 접힌 높이 [m] (회전 프레임 상면 ↔ 덱 하면)
        p("end_n", 2)             # 전후단부 시저 단수
        p("end_len", 1.000)       # 유닛 앞뒤 길이에 맞춘 긴 바
        p("end_ext_hi", 0.696)    # q=0 에서 펼친 높이 [m]
        # ⚠ 자유단 시저는 **지령이 늦으면 그만큼 눈에 띄게 뒤따라온다.**
        #   상하축이 빨리 움직이는 구간(잭업)에서 특히 그렇다. 50 → 100 Hz.
        p("rate", 100.0)          # 지령 발행 주기 [Hz]

        g = lambda k: self.get_parameter(k).value          # noqa: E731
        self.body_n, self.body_len = g("body_n"), g("body_len")
        self.body_ext0 = g("body_ext0")
        self.end_n, self.end_len = g("end_n"), g("end_len")
        self.end_ext_hi = g("end_ext_hi")

        # 상하축 관절 → (조립체 이름들, 단수, 바 길이, ext 를 푸는 함수)
        self.rigs = {
            "body_lift_joint":  (("body_sc",), self.body_n, self.body_len,
                                 lambda q: self.body_ext0 + q),
            "front_lift_joint": (("front_sc",), self.end_n, self.end_len,
                                 lambda q: self.end_ext_hi - q),
            "rear_lift_joint":  (("rear_sc",),  self.end_n, self.end_len,
                                 lambda q: self.end_ext_hi - q),
        }

        # URDF 가 만든 관절 이름 순서를 그대로 재현한다.
        self.names: list[str] = []
        for nss, n, _, _ in self.rigs.values():
            for ns in nss:
                for k in range(n):
                    self.names += [f"{ns}_m{k}_scissor_joint",
                                   f"{ns}_p{k}_scissor_joint"]

        self.q: dict[str, float] = {j: 0.0 for j in self.rigs}
        self.pub = self.create_publisher(JointTrajectory, "/scissor/traj", 10)

        self.create_subscription(JointState, "/joint_states", self.on_js, 20)
        self.create_timer(1.0 / g("rate"), self.tick)
        self.create_timer(10.0, self.report)
        self.seen = False

        self.get_logger().info(
            f"시저 종동 구동 시작 — 몸체부 {self.body_n}단/{self.body_len*1000:.0f}mm, "
            f"전후단부 {self.end_n}단/{self.end_len*1000:.0f}mm, 관절 {len(self.names)}개. "
            f"/joint_states 의 상하축을 보고 /scissor/traj 로 각도를 낸다.")

    def on_js(self, m: JointState) -> None:
        """상하축 **실제 관절각**을 읽는다. 지령이 아니라 실제값이라
        제어 지연·오버슈트까지 X자가 그대로 따라간다."""
        for n, pos in zip(m.name, m.position):
            if n in self.q:
                self.q[n] = pos
                self.seen = True

    def theta(self, ext: float, n: int, blen: float) -> float:
        """스택 높이 ext 를 내려면 바가 몇 도로 서야 하는가.

        ext > N·L 이면 기구학적으로 불가능하다(바가 짧다). 그럴 땐 90도로 잡되
        params.xacro 가 잘못된 것이므로 경고를 남긴다."""
        s = ext / (n * blen)
        if s > 1.0 or s < -1.0:
            self.get_logger().warn(
                f"시저 신장량 {ext:.3f} m 가 바 길이 한계 {n*blen:.3f} m 를 넘는다 — "
                f"params.xacro 의 단수/바길이를 확인할 것", throttle_duration_sec=10.0)
            s = max(-1.0, min(1.0, s))
        return math.asin(s)

    def tick(self) -> None:
        if not self.seen:
            return                      # 아직 /joint_states 가 안 온다
        pos: list[float] = []
        for joint, (nss, n, blen, ext_of) in self.rigs.items():
            th = self.theta(ext_of(self.q[joint]), n, blen)
            for _ in nss:
                for k in range(n):
                    # 뿌리 바만 θ, 나머지는 전부 2θ. 부호는 URDF 의 axis 가 갖는다.
                    pos += [th if k == 0 else 2.0 * th, 2.0 * th]

        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.names
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in pos]
        msg.points = [pt]               # 시간 0 = 즉시 적용
        self.pub.publish(msg)

    def report(self) -> None:
        if not self.seen:
            self.get_logger().warn(
                "/joint_states 가 아직 없다 — Gazebo 와 브리지가 떠 있는지 확인할 것")
            return
        txt = []
        for joint, (nss, n, blen, ext_of) in self.rigs.items():
            ext = ext_of(self.q[joint])
            txt.append(f"{joint.split('_')[0]} q {self.q[joint]:+.3f} → "
                       f"신장 {ext*1000:5.0f} mm θ "
                       f"{math.degrees(self.theta(ext, n, blen)):5.2f}°")
        self.get_logger().info(" | ".join(txt))


def main() -> None:
    rclpy.init()
    node = ScissorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # launch 가 SIGTERM 을 보내면 rclpy 가 이미 컨텍스트를 내린 뒤라
        # 그냥 shutdown() 하면 RCLError 로 죽는다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
