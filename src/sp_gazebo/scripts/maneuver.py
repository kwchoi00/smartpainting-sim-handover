#!/usr/bin/env python3
"""주행 → 정지 → 몸체부 회전 → 전후단부 착지 → 구획 끝에서 끝까지 왕복.

    ros2 run sp_gazebo maneuver.py                   # 0.5 m 뒤로 주행 후 90° 회전 후 왕복
    ros2 run sp_gazebo maneuver.py -d 0.5            # 앞으로 0.5 m
    ros2 run sp_gazebo maneuver.py --no-sweep        # 왕복 없이 착지까지만
    ros2 run sp_gazebo maneuver.py --cycle           # 위 동작 + 주행 자세 복귀
    ros2 run sp_gazebo maneuver.py --home            # 복귀만
    ros2 run sp_gazebo maneuver.py --angle -1.5708   # 반대로 회전
    ros2 run sp_gazebo maneuver.py -d 0              # 주행 없이 회전만

Gazebo 는 따로 띄워 둘 것:

    ros2 launch sp_gazebo sim.launch.py world:=ship_block_cad_world \\
        x:=-1.97 y:=-0.0325 z:=0.53 rviz:=false software_render:=true

시작 자세 (전 관절 0):
    몸체부는 바퀴부와 나란히(= 론지와 평행), 전후단부도 론지와 평행,
    전후단부는 통로 바닥에 내려앉아 있다.

동작 순서:
    0) 론지를 따라 distance 만큼 주행하고 **선다**
    1) 전후단부 **아랫부분(궤도)** 을 lift_clear(0.64) 로 들어 **론지와 거더를 넘긴다**
       (윗부분인 회전부는 몸체부에 붙어 있어 움직이지 않는다 — 시저가 접힌다)
    2) **전후단부만** -angle 로 먼저 돌린다 (덱이 아직 바퀴부와 나란할 때)
    3) 몸체부를 angle 만큼 돌린다 — 전후단부가 옆 통로로 간다

    ⚠ 2)와 3)의 **순서가 중요하다.** 덱을 먼저 돌려 놓고 전후단부를 돌리면 덱이
      최대 7.5° 출렁인다. 진동이 아니라 **블록 거더에 걸리는 것**이다 — 덱이
      수직이면 전후단부 회전축이 y ±1.690 로 와서 스윕 원이 거더 보강대를 문다.
      나란할 때 돌리면 거더에서 멀어 0.1° 다.
    4) 전후단부를 lift_down(0) 으로 내린다 — 처음과 같은 방향으로 통로 바닥에
    5) **앞쪽(+X) 횡프레임 앞까지** 주행
    6) **뒤쪽(-X) 횡프레임 앞까지** 쭉 주행하고 선다

━━ 왜 회전한 뒤에야 구획 끝까지 갈 수 있는가 ━━
회전 전에는 로봇이 X 로 4.43 m 를 차지해서(전후단부가 ±1.690 에 있으므로)
5.91 m 구획 안에서 앞뒤로 0.74 m 밖에 못 움직인다.

몸체부를 90° 돌리면 전후단부가 **X 가 아니라 Y 로** ±1.690 만큼 벌어진다.
그러면 X 방향으로 남는 건 회전하지 않는 **바퀴부(±0.826, 구동륜 바깥날 0.831)**
뿐이라 로봇의 X 길이가 4.43 → 1.66 m 로 줄고, 구획을 **끝에서 끝까지** 지날 수 있다.

    회전 전  ├──────────── 4.43 m ────────────┤   여유 1.48 m
    회전 후          ├─ 1.66 m ─┤                 여유 4.25 m
             ├────────────── 구획 5.91 m ──────────────┤

⚠ 정지 위치는 **오도메트리 기준**이다 (`spawn_x + /odom.x`). 벽까지의 여유는
  `--margin`(기본 0.10 m)으로 두는데, 이 값이 곧 **오도메트리 오차 예산**이다.
  slip 이 크면 로봇은 자기가 선 줄 알고 프레임을 민다.

⚠ 전후단부는 몸체부의 **자식**이라 `/front_turn` `/rear_turn` 은 몸체부에 대한
  **상대각**이다. 3)에서 -angle 을 주면 월드 기준으로 처음 방향(론지와 평행)이
  된다. 2)와 3)을 한꺼번에 주면 결과는 같지만 전후단부가 수직이 되는 과정이
  안 보인다 — 지시대로 **나눠서** 보낸다.

⚠ **회전할 자리를 남겨야 한다.** 제약이 두 개인데 회전 쪽이 더 빡빡하다.

    직진만    : 로봇 4.10 m 가 구획 5.91 m 안에 있으면 되므로 중심 ±0.90 m
    회전까지  : 몸체부가 돌 때 전후단부가 **반경 2.229 m** 를 쓴다
                (회전축 1.690 + 끝단부 바깥 반길이 0.5225, 폭 0.2715 포함)
                → 구획 중심(x=-1.968)에서 **±0.726 m** 안에서만 돌 수 있다

  기본값 0.5 m 는 그 안쪽이다. 0.73 m 를 넘겨 놓고 돌리면 전후단부가
  횡프레임을 때려서 회전이 안 끝나고, 로봇이 밀려난다 (실측: 0.8 m 에서 발생).

⚠ **착지가 통로 안에 들어오려면 전후단부 회전축 간격이 론지 pitch 의 정수배여야
  한다.** 도면 실측은 ±1.5275 m 인데 pitch 0.845 의 2배는 1.690 m 다 (163 mm 차이).
  90° 돌린 뒤 전후단부는 통로 중심에서 그만큼 어긋난 곳에 오고, 폭 543 mm 가
  통로(695 mm)를 벗어나 론지에 걸터앉는다.
  `params.xacro` 의 `end_turn_x` 를 1.690 으로 바꾸면 그대로 성립한다.
  `--lift-down 0.450` 을 주면 통로 바닥 대신 론지 상면에 앉힐 수도 있다.

⚠ 완료 판정은 **`/joint_states` 관절 엔코더**로 한다.
  상하·회전축은 움직여도 바퀴가 안 구르므로 `/odom` 에 나타나지 않는다.
  `/ground_truth` 는 평가 전용이라 제어 루프에 넣지 않는다.
  고정 시간 대기로 두면 시뮬이 느릴 때(소프트웨어 렌더링은 실시간의 30% 안팎)
  동작이 덜 끝난 채로 다음 단계가 나간다.
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# 지령 토픽 ← → /joint_states 의 관절 이름
JOINT_OF = {
    "front_lift": "front_lift_joint",
    "rear_lift":  "rear_lift_joint",
    "front_turn": "front_turn_joint",
    "rear_turn":  "rear_turn_joint",
    "body_lift":  "body_lift_joint",
    "body_turn":  "body_turn_joint",
    "rail_cmd":   "carriage_joint",
}


class Maneuver(Node):
    def __init__(self, tol_lin: float, tol_ang: float, timeout: float,
                 spawn_x: float = 0.0):
        super().__init__("maneuver")
        self.tol_lin, self.tol_ang, self.timeout = tol_lin, tol_ang, timeout
        self.spawn_x = spawn_x
        self.pos: dict[str, float] = {}
        self.odom: tuple[float, float] | None = None
        self.pub = {t: self.create_publisher(Float64, f"/{t}", 10) for t in JOINT_OF}
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

    def _on_js(self, msg: JointState) -> None:
        for name, p in zip(msg.name, msg.position):
            self.pos[name] = p

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _spin(self, sec: float) -> None:
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self) -> bool:
        """첫 /joint_states 가 올 때까지. 안 오면 Gazebo 가 안 떠 있는 것."""
        for _ in range(200):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pos:
                return True
        return False

    def go(self, targets: dict[str, float], label: str) -> bool:
        """targets 를 계속 쏘면서 관절이 다 도달할 때까지 기다린다.

        한 번만 쏘면 gz 쪽 구독자가 아직 준비 안 됐을 때 그대로 유실된다.
        도달할 때까지 반복 발행한다.
        """
        self.get_logger().info(
            f"▶ {label}: " + ", ".join(f"{t}={v:+.3f}" for t, v in targets.items()))
        start = time.time()
        while time.time() - start < self.timeout:
            for t, v in targets.items():
                m = Float64()
                m.data = float(v)
                self.pub[t].publish(m)
            self._spin(0.2)

            done, worst = True, 0.0
            for t, v in targets.items():
                cur = self.pos.get(JOINT_OF[t])
                if cur is None:
                    done = False
                    continue
                tol = self.tol_ang if t.endswith("turn") else self.tol_lin
                err = abs(cur - v)
                worst = max(worst, err)
                if err > tol:
                    done = False
            if done:
                self.get_logger().info(
                    f"  완료 ({time.time()-start:.1f}s, 최대 오차 {worst:.4f})")
                self._spin(0.5)          # 정착
                return True
        self.get_logger().error(
            f"  ✗ {self.timeout:.0f}s 안에 도달 못 함. "
            f"현재: " + ", ".join(
                f"{JOINT_OF[t]}={self.pos.get(JOINT_OF[t], float('nan')):+.3f}"
                for t in targets))
        return False

    def drive(self, dist: float, speed: float) -> bool:
        """론지를 따라 dist [m] 주행하고 선다.

        완료 판정은 **`/odom`**(휠 오도메트리)으로 한다 — 이 로봇에서 주행 거리를
        아는 유일한 자체 관측이다. `/ground_truth` 는 평가 전용이라 쓰지 않는다.
        ⚠ 그래서 이 스크립트의 정지 위치에는 오도메트리 오차가 그대로 실린다.
          그게 이 과제가 재려는 바로 그 오차다.
        """
        if abs(dist) < 1e-6:
            return True
        for _ in range(100):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is not None:
                break
        if self.odom is None:
            self.get_logger().error("/odom 이 안 옵니다.")
            return False

        x0, y0 = self.odom
        v = speed if dist > 0 else -speed
        self.get_logger().info(f"▶ 0) 론지 따라 {dist:+.2f} m 주행 (v={v:+.2f} m/s)")
        start = time.time()
        while time.time() - start < self.timeout:
            x, y = self.odom
            moved = ((x - x0) ** 2 + (y - y0) ** 2) ** 0.5
            if moved >= abs(dist):
                break
            m = Twist()
            m.linear.x = v
            self.cmd.publish(m)
            self._spin(0.05)
        else:
            self.get_logger().error("  ✗ 주행이 시간 안에 안 끝났습니다")
            self.cmd.publish(Twist())
            return False

        # 정지 — 한 번만 쏘면 씹힐 수 있어 잠깐 반복해서 확실히 세운다
        for _ in range(20):
            self.cmd.publish(Twist())
            self._spin(0.05)
        x, y = self.odom
        moved = ((x - x0) ** 2 + (y - y0) ** 2) ** 0.5
        self.get_logger().info(
            f"  정지 ({time.time()-start:.1f}s, /odom 기준 {moved:.3f} m 이동)")
        self._spin(1.0)
        return True

    def world_x(self) -> float | None:
        """오도메트리로 추정한 월드 X.

        `/odom` 은 스폰 지점을 원점으로 낸다 (실측: x=-18 에 스폰 후 12 m 주행하면
        `/odom.x` = 11.970). 그래서 `spawn_x` 만 더하면 월드 좌표가 된다.
        ⚠ 어디까지나 **오도메트리 추정**이다. 참값이 아니다.
        """
        return None if self.odom is None else self.spawn_x + self.odom[0]

    def drive_to(self, target: float, speed: float, label: str) -> bool:
        """월드 X = target 까지 론지를 따라 주행하고 선다.

        방향은 **출발할 때 한 번만** 정하고, 목표를 지나치면 멈춘다.
        매 주기 부호를 다시 계산하면 정지 구간에서 앞뒤로 떨린다.
        """
        for _ in range(100):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is not None:
                break
        x0 = self.world_x()
        if x0 is None:
            self.get_logger().error("/odom 이 안 옵니다.")
            return False
        if abs(target - x0) < self.tol_lin:
            self.get_logger().info(f"▶ {label}: 이미 x={x0:+.3f} 이라 건너뜁니다")
            return True

        sign = 1.0 if target > x0 else -1.0
        v = sign * speed
        # 시간 예산은 거리에 맞춘다. 고정값으로 두면 긴 주행이 잘린다 —
        # 소프트웨어 렌더링은 실시간의 30% 안팎이라 벽시계로는 3배 이상 걸린다.
        budget = max(self.timeout, abs(target - x0) / max(speed, 1e-3) * 8.0)
        self.get_logger().info(
            f"▶ {label}: x {x0:+.3f} → {target:+.3f} "
            f"({target-x0:+.2f} m, v={v:+.2f} m/s, 예산 {budget:.0f}s)")
        start = time.time()
        while time.time() - start < budget:
            x = self.world_x()
            if sign * (x - target) >= -self.tol_lin:      # 도달했거나 지나쳤다
                break
            m = Twist()
            m.linear.x = v
            self.cmd.publish(m)
            self._spin(0.05)
        else:
            self.get_logger().error(
                f"  ✗ {budget:.0f}s 안에 못 갔습니다 (현재 x={self.world_x():+.3f}). "
                f"프레임에 막혔을 수 있습니다")
            self.cmd.publish(Twist())
            return False

        for _ in range(20):
            self.cmd.publish(Twist())
            self._spin(0.05)
        self.get_logger().info(
            f"  정지 ({time.time()-start:.1f}s, /odom 기준 x={self.world_x():+.3f})")
        self._spin(1.0)
        return True

    def report(self) -> None:
        """지령 가능한 관절만 찍는다.

        구동륜 26개·접지롤러 8개·시저 링키지 20개는 **종동체**라 여기 섞이면
        읽을 수가 없다 (전체 61개 중 54개가 종동체다). 구동륜 회전량이 필요하면
        `/joint_states` 를 직접 보거나 `wheel_odom` 을 쓸 것."""
        skip = ("_wheel_joint", "_roller_joint", "_scissor_joint")
        self.get_logger().info(
            "관절 상태: " + ", ".join(f"{n}={self.pos[n]:+.3f}"
                                   for n in sorted(self.pos)
                                   if not n.endswith(skip)))


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-d", "--distance", type=float, default=-0.5,
                    help="회전 전에 론지를 따라 주행할 거리 [m] (기본 -0.5 = 뒤로). "
                         "부호가 방향이다 — 양수면 +X, 음수면 -X. "
                         "⚠ 회전에 반경 2.229 m 가 필요해 구획 중심에서 ±0.726 m "
                         "안에서만 돌 수 있다. 0 이면 주행 없이 회전만")
    ap.add_argument("-s", "--speed", type=float, default=0.20,
                    help="주행 속도 [m/s] (기본 0.20)")
    ap.add_argument("--angle", type=float, default=1.5708,
                    help="몸체부 회전각 [rad] (기본 1.5708 = 90°)")
    ap.add_argument("--home", action="store_true",
                    help="회전 대신 **주행 자세로 복귀** (몸체부 0°, 전후단부 바닥)")
    ap.add_argument("--cycle", action="store_true", help="회전 → 복귀 연속")
    ap.add_argument("--lift-clear", type=float, default=0.64,
                    help="회전 중 전후단부 높이 [m]. 론지(0.450)가 아니라 **거더**"
                         "(0.590)를 넘겨야 한다 — params.xacro 의 end_lift_clear")
    ap.add_argument("--lift-down", type=float, default=0.0,
                    help="회전 뒤 전후단부를 내릴 높이 [m]. 기본 0 = 통로 바닥 "
                         "(처음 자세와 같다). 0.450 을 주면 론지 상면에 앉힌다")
    # ── 착지 후 구획 왕복 ──
    ap.add_argument("--no-sweep", dest="sweep", action="store_false",
                    help="착지까지만 하고 구획 왕복은 하지 않는다")
    ap.add_argument("--spawn-x", type=float, default=-1.97,
                    help="스폰한 월드 X [m]. `/odom` 이 여기를 원점으로 내므로 "
                         "월드 좌표를 알려면 필요하다 (런치의 x:= 와 같게 줄 것)")
    ap.add_argument("--wall-front", type=float, default=0.985,
                    help="앞쪽 횡프레임 **안쪽 면**의 월드 X [m] "
                         "(도면 실측: 프레임 중심 +1.03, 두께 90 mm)")
    ap.add_argument("--wall-rear", type=float, default=-4.925,
                    help="뒤쪽 횡프레임 안쪽 면의 월드 X [m] (중심 -4.97)")
    ap.add_argument("--half-len", type=float, default=0.84,
                    help="회전 후 로봇의 X 반길이 [m]. 회전하지 않는 바퀴부가 "
                         "지배한다 — 차체 0.826, 구동륜 바깥날 0.831")
    ap.add_argument("--margin", type=float, default=0.10,
                    help="프레임까지 남길 여유 [m]. 이 값이 곧 오도메트리 오차 예산이다")
    ap.add_argument("--sweep-speed", type=float, default=None,
                    help="왕복 속도 [m/s] (기본: --speed 와 같게)")
    ap.add_argument("--timeout", type=float, default=180.0,
                    help="단계별 최대 대기 [s]. 소프트웨어 렌더링은 느리다")
    ap.add_argument("--tol-lin", type=float, default=0.01, help="상하축 허용오차 [m]")
    ap.add_argument("--tol-ang", type=float, default=0.02, help="회전축 허용오차 [rad]")
    args = ap.parse_args()

    rclpy.init()
    node = Maneuver(args.tol_lin, args.tol_ang, args.timeout, args.spawn_x)
    try:
        if not node.wait_ready():
            node.get_logger().error(
                "/joint_states 가 안 옵니다. Gazebo 가 떠 있는지, "
                "robot:=true 인지 확인하세요.")
            return 1
        node.report()

        # 회전한 자리를 기억해 둔다. --cycle 로 되돌릴 때 여기로 와야 한다 —
        # 구획 끝에서는 몸체부를 되돌릴 반경(2.229 m)이 안 나온다.
        turn_x: list[float] = []

        def sweep() -> bool:
            """착지 뒤, 구획을 앞 끝에서 뒤 끝까지 훑는다.

            회전으로 X 길이가 4.43 → 1.66 m 로 줄었기 때문에 가능한 구간이다.
            """
            if not args.sweep:
                return True
            sp = args.sweep_speed if args.sweep_speed is not None else args.speed
            x_f = args.wall_front - args.half_len - args.margin
            x_r = args.wall_rear + args.half_len + args.margin
            node.get_logger().info(
                f"── 구획 왕복 구간: x {x_r:+.3f} ~ {x_f:+.3f} "
                f"({x_f-x_r:.2f} m) | 프레임 {args.wall_rear:+.3f}/{args.wall_front:+.3f}, "
                f"반길이 {args.half_len:.3f}, 여유 {args.margin:.3f}")
            return (node.drive_to(x_f, sp, "5) 앞쪽 횡프레임 앞까지")
                    and node.drive_to(x_r, sp, "6) 뒤쪽 횡프레임 앞까지 쭉"))

        def mark_turn_spot() -> bool:
            turn_x.append(node.world_x() or args.spawn_x)
            return True

        def rotate(angle: float) -> bool:
            return (
                node.drive(args.distance, args.speed)
                and mark_turn_spot()
                and node.go({"front_lift": args.lift_clear,
                         "rear_lift": args.lift_clear}, "1) 전후단부 들어 론지 넘기기")
                # ⚠ **전후단부를 먼저 돌린다.** 덱을 먼저 돌려 놓고 전후단부를 돌리면
                #   덱이 최대 7.5° 출렁인다 — 덱이 수직일 때 전후단부 회전축이
                #   y ±1.690 로 와서 스윕 원이 블록 거더 보강대를 물기 때문이다.
                #   나란할 때 돌리면 거더에서 멀어 0.1° 다 (실측).
                #   두 관절의 최종값은 순서와 무관하므로 자세는 똑같다.
                and node.go({"front_turn": -angle, "rear_turn": -angle},
                            f"2) 전후단부를 먼저 {-angle:+.4f} rad "
                            f"(덱이 나란할 때라 안 흔들린다)")
                and node.go({"body_turn": angle},
                            f"3) 몸체부 {angle:+.4f} rad 회전 "
                            f"(전후단부가 옆 통로로 간다)")
                and node.go({"front_lift": args.lift_down,
                             "rear_lift": args.lift_down},
                            f"4) 전후단부를 {args.lift_down:.3f} 로 내리기 "
                            f"(처음과 같은 방향으로 통로 바닥에)")
                and sweep())

        def home() -> bool:
            # 구획 끝에 서 있으면 몸체부를 되돌릴 자리가 없다 — 되돌릴 때도 전후단부가
            # 반경 2.229 m 를 쓴다. 돌았던 자리(모르면 구획 중심)로 먼저 간다.
            spot = turn_x[0] if turn_x else (args.wall_rear + args.wall_front) / 2.0
            return node.drive_to(spot, args.speed, "0) 회전할 수 있는 자리로 복귀") and (
                node.go({"front_lift": args.lift_clear,
                         "rear_lift": args.lift_clear}, "1) 전후단부 들어 론지 넘기기")
                and node.go({"front_turn": 0.0, "rear_turn": 0.0},
                            "2) 전후단부를 몸체부와 나란히 되돌리기")
                and node.go({"body_turn": 0.0},
                            "3) 몸체부를 론지와 평행하게 되돌리기")
                and node.go({"front_lift": 0.0, "rear_lift": 0.0},
                            "4) 전후단부를 통로 바닥에 내리기"))

        if args.home:
            ok = home()
        elif args.cycle:
            ok = rotate(args.angle) and home()
        else:
            ok = rotate(args.angle)

        node.report()
        node.get_logger().info("끝" if ok else "✗ 중간에 실패했습니다")
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
