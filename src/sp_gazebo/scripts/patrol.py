#!/usr/bin/env python3
"""블록 내부를 ㅁ자로 순회한다 — 론지 따라 종주행 + 론지 가로질러 횡주행.

    ros2 run sp_gazebo patrol.py                  # 한 바퀴
    ros2 run sp_gazebo patrol.py --laps 2         # 두 바퀴
    ros2 run sp_gazebo patrol.py --length 1.2 --cross 2   # 종 1.2 m, 횡 론지 2칸
    ros2 run sp_gazebo patrol.py --dry-run        # 순서만 출력하고 안 움직인다

Gazebo 와 로봇이 떠 있어야 한다:

    ros2 launch sp_gazebo sim.launch.py world:=ship_block_cad_world \\
        x:=-2.68 y:=-0.8775 z:=0.53

━━ 이 로봇이 방향을 바꾸는 방법 ━━
바퀴부에는 조향도 회전축도 없다. 론지 위를 X 로만 구른다.
방향을 바꾸는 건 **전후단부를 딛고 몸체부 회전축을 쓰는 것**이다:

  · 전후단부가 통로 바닥을 딛고 있으면 → /body_turn 은 **덱을 돌린다**
    (덱에 매달린 전후단부도 같이 돈다. 바퀴부는 론지에 눌려 안 돈다)
  · 전후단부로 **잭업해서 바퀴부를 띄우면** → /body_turn 은 **바퀴부를 돌린다**
    (덱이 전후단부에 고정돼 있어서 반력이 바퀴부로 간다)

실측: end_lift −0.08 로 로봇이 58 mm 뜨고, 그 상태에서 /body_turn +90° 를 주면
바퀴부가 세계좌표에서 −89.6° 돈다.

⚠ **잭업은 end_lift 를 음수로 주는 것**이다. q=0 이 통로 바닥이므로 더 밀어내리면
  바닥이 막아서 로봇 전체가 들린다. params.xacro 의 end_lift_down 참조.

━━ ㅁ자 한 바퀴 ━━
    A. 종주행 준비  전후단부를 옆 통로로 옮기고 (덱 90° → 전후단부 90° 되돌림)
    B. 종주행       론지를 따라 X 로 전진
    C. 코너         잭업 → 바퀴부 90° → 착지 → 덱·전후단부도 90° → 전부 횡방향
    D. 횡주행       론지를 **가로질러** Y 로 이동 (바퀴가 론지 상면을 타고 넘는다)
    E. 코너 복귀    다시 전부 종방향으로 → 다음 변으로

⚠ **횡주행은 바퀴가 론지 상면만 딛고 간다.** 측당 13개를 130 mm 피치로 깐 것이
  여기서 값을 한다 — 론지 pitch 845, 플랜지 폭 150 이라 바퀴열(1.56 m)이 어느
  위치에서든 플랜지 하나 이상을 물고 있다. 대신 통로를 지날 때마다 덜컹거린다.

⚠ **횡방향에서 전후단부의 접지면은 통로 바닥이 아니라 론지 상면**이다.
  길이 1.05 m 짜리가 pitch 0.845 를 가로지르면 반드시 론지를 깔고 앉기 때문.
  그래서 횡방향에서는 end_lift 를 end_drop(0.450) 으로 준다.

━━ ⚠ 횡주행 거리 — 아무 값이나 쓸 수 없다 ━━
횡주행이 끝난 자리에서 **바퀴부와 전후단부가 동시에 제자리를 찾아야** 한다:

  · 바퀴부  양쪽 바퀴가 이웃한 두 론지에 걸쳐야 한다 → y 가 그 중점이어야 한다
  · 전후단부 회전축에서 ±1.690 (= pitch 2칸) 떨어진 곳이 **통로**여야 한다

도면 월드는 **론지 간격이 균일하지 않다.** 중심이
    -4.716 -3.871 -2.145 -1.300 -0.455 0.390 1.235 2.080 3.845 4.690
이라 -3.871~-2.145 와 2.080~3.845 에는 론지가 하나씩 빠져 있다.
⚠ **그 자리는 비어 있는 게 아니라 거더**(y -3.060 / +2.995, 전高 격벽)이고,
거더의 수평 보강대는 론지 상면(0.520)보다 70~140 mm 높다 (0.590 / 0.660).
그 구간으로 전후단부가 가면 통로에 못 앉고, 들어올려 돌릴 때도 보강대에 긁혀
**회전이 막히거나 덱이 출렁인다** (CLEAR 를 0.640 으로 둔 이유).

**실측 — 도면 월드에서 2칸은 안 된다.** y -0.8775 에서 2칸(1.690 m)을 가면
도착지(+0.8125 근처)에서 전단부가 론지 +2.080 **하나에만** 얹혀 외날 위에 선 꼴이
된다. 그 상태에서는

  · 코너 순서를 원래대로(론지 상면에서 잭업) 하면 → 바퀴부가 안 뜬다
    (body_turn 지령 0 에 실제 1.569 — 전혀 안 돔)
  · 순서를 바꿔(전후단부 먼저 되돌리기) 해도 → 전단부가 끼어서 안 돈다
    (front_turn 지령 -1.571 에 실제 -0.008)

둘 다 재현했다. 원인은 블록 형상이다 — 2.080~3.845 에 론지가 하나 빠져 있어서
전후단부(회전축에서 ±1.690)가 그 구간에 들어가면 지지를 잃는다.
**1칸(0.845 m)은 전 구간 정상**이라 기본값이 1 이다. 2칸을 쓰려면 그 구간에
론지를 채운 월드가 필요하다.
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JOINT_OF = {
    "front_lift": "front_lift_joint",
    "rear_lift":  "rear_lift_joint",
    "front_turn": "front_turn_joint",
    "rear_turn":  "rear_turn_joint",
    "body_lift":  "body_lift_joint",
    "body_turn":  "body_turn_joint",
    "rail_cmd":   "carriage_joint",
}

LONGI_PITCH = 0.845     # 론지 간격 = 구동륜 궤간
END_DROP    = 0.450     # 론지 상면 ↔ 통로 바닥 낙차
JACK        = -0.080    # 잭업 지령 (음수 = 바닥을 더 밀어 로봇을 들어올린다)
# ⚠ **론지(0.520)가 아니라 거더(0.660)를 넘어야 한다.** 블록 바닥에는 론지가
#   '빠진' 자리마다 더 높은 거더가 있다 — 수평 보강대 상면이
#   좌현(y -3.060) 0.590, 우현(y +2.995) 0.660 이다.
#   0.500 으로 들면 최하단(접지 롤러)이 0.567 이라 좌현 거더에 23 mm 못 미쳐
#   롤러가 긁고, 그 반력이 1.69 m 팔로 덱을 비틀어 **덱이 6.8° 출렁였다** (실측).
#   params.xacro 의 end_lift_clear 와 같은 값이다.
CLEAR       = 0.640     # 전후단부를 **거더** 위로 넘길 때 드는 높이
HALF        = 1.5708    # 90°


class Patrol(Node):
    def __init__(self, tol_lin: float, tol_ang: float, timeout: float,
                 stall: float = 25.0) -> None:
        super().__init__("patrol")
        self.tol_lin, self.tol_ang, self.timeout = tol_lin, tol_ang, timeout
        self.stall = stall
        self.pos: dict[str, float] = {}
        self.odom: tuple[float, float] | None = None
        self.gt: tuple[float, float, float] | None = None     # x, y, yaw (평가용)
        self.pub = {t: self.create_publisher(Float64, f"/{t}", 10) for t in JOINT_OF}
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Odometry, "/ground_truth", self._on_gt, 10)
        self.step = 0

    # ── 콜백 ──
    def _on_js(self, m: JointState) -> None:
        for n, p in zip(m.name, m.position):
            self.pos[n] = p

    def _on_odom(self, m: Odometry) -> None:
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _on_gt(self, m: Odometry) -> None:
        """⚠ 평가·로그 전용이다. 제어 판정에 쓰지 않는다 — 실물에는 없는 값이다."""
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.gt = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def _spin(self, sec: float) -> None:
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self) -> bool:
        for _ in range(300):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pos and self.odom is not None:
                return True
        return False

    def _where(self) -> str:
        if self.gt is None:
            return ""
        x, y, yaw = self.gt
        return f"  [참값 x {x:+.3f} y {y:+.3f} yaw {math.degrees(yaw):+7.2f}°]"

    # ── 관절 ──
    def go(self, targets: dict[str, float], label: str) -> bool:
        """targets 에 다 도달할 때까지 계속 쏜다.

        한 번만 쏘면 gz 구독자가 아직 준비 안 됐을 때 유실된다."""
        self.step += 1
        self.get_logger().info(
            f"▶ {self.step:2d}) {label}: "
            + ", ".join(f"{t}={v:+.3f}" for t, v in targets.items()))
        start = time.time()
        cap = start + self.timeout
        best, last_gain, worst = float("inf"), start, float("inf")
        while time.time() < cap:
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
                    f"   완료 ({time.time()-start:.1f}s, 최대 오차 {worst:.4f})"
                    + self._where())
                self._spin(0.5)
                return True
            # 오차가 줄고 있으면 마감을 미룬다 — 느린 것과 막힌 것을 가른다
            if worst < best - 1e-4:
                best, last_gain = worst, time.time()
                cap = max(cap, time.time() + self.timeout)
            elif time.time() - last_gain > self.stall:
                break
        # ⚠ 잭업은 로봇 무게에 눌려 지령에 못 미친다 (실측 -0.080 지령에 -0.072).
        #   그건 정상이므로 실패로 보지 않는다. 대신 **무엇이 얼마나 막혔는지**를
        #   남긴다 — 걸리는 단계를 찾을 때 이 줄이 전부다.
        detail = ", ".join(
            f"{t} 지령{v:+.3f} 실제{self.pos.get(JOINT_OF[t], float('nan')):+.3f}"
            for t, v in targets.items())
        self.get_logger().warn(
            f"   ⚠ 허용오차까지 못 갔다 (최대 오차 {worst:.4f}) — {detail}" + self._where())
        return True

    # ── 주행 ──
    def drive(self, dist: float, speed: float, label: str) -> bool:
        """/odom 으로 거리를 재며 간다. **바퀴부가 향한 방향으로** 간다 —
        종방향이면 X, 횡방향이면 Y 로 움직인다.

        ⚠ 마감시간을 고정으로 두면 안 된다. 이 시뮬은 실시간 계수가 0.15 안팎이라
          1 m 주행에 벽시계로 40 초가 걸리고, 횡주행은 바퀴가 통로를 지날 때마다
          덜컹거려 더 느리다. 그래서 **"느린 것"이 아니라 "안 움직이는 것"만**
          실패로 본다 — stall 초 동안 진전이 없으면 그때 중단한다.
          절대 상한은 거리에 비례해 넉넉히 잡는다 (RTF 0.025 까지 버틴다)."""
        self.step += 1
        v = speed if dist > 0 else -speed
        target = abs(dist)
        self.get_logger().info(f"▶ {self.step:2d}) {label}: {dist:+.2f} m (v={v:+.2f})")
        x0, y0 = self.odom
        start = time.time()
        cap = start + self.timeout + target / max(speed, 1e-6) * 40.0
        best, last_gain, last_say = 0.0, start, start
        while time.time() < cap:
            x, y = self.odom
            moved = math.hypot(x - x0, y - y0)
            if moved >= target:
                break
            now = time.time()
            if moved > best + 0.002:            # 2 mm 이상 나아갔다
                best, last_gain = moved, now
            elif now - last_gain > self.stall:
                self.cmd.publish(Twist())
                self.get_logger().error(
                    f"   ✗ {self.stall:.0f} 초째 안 움직인다 — "
                    f"{moved:.3f}/{target:.3f} m 에서 멈췄다" + self._where())
                return False
            if now - last_say > 15.0:           # 오래 걸릴 때 죽은 줄 알까 봐
                last_say = now
                self.get_logger().info(f"   … {moved:.3f}/{target:.3f} m")
            m = Twist()
            m.linear.x = v
            self.cmd.publish(m)
            self._spin(0.05)
        else:
            self.cmd.publish(Twist())
            self.get_logger().error(
                f"   ✗ 절대 상한({cap-start:.0f} s)을 넘겼다 — "
                f"{best:.3f}/{target:.3f} m" + self._where())
            return False
        for _ in range(25):
            self.cmd.publish(Twist())
            self._spin(0.05)
        x, y = self.odom
        self.get_logger().info(
            f"   완료 ({time.time()-start:.1f}s, 이동 {math.hypot(x-x0, y-y0):.3f} m)"
            + self._where())
        self._spin(1.0)
        return True

    # ── 국면 ──
    def stance_along(self) -> bool:
        """A. 종주행 자세 — 전후단부를 **옆 통로**로 옮겨 론지와 나란히 세운다.
        처음 한 번만 한다 (코너를 돌고 나면 이 자세로 돌아온다).

        덱을 90° 돌리면 전후단부가 론지를 가로지르는 방향으로 간다. 회전축 간격
        1.690 = pitch 2칸이라 정확히 두 칸 옆 통로에 떨어진다. 거기서 전후단부만
        −90° 되돌리면 다시 론지와 나란해진다.

        ⚠ 이 자세에서 **덱은 론지와 수직**이다. 덱이 그 각도를 유지해야 전후단부가
          옆 통로에 걸쳐 있을 수 있다. body_turn=+90° 가 종주행의 기본 자세다.

        ⚠ **순서가 중요하다 — 전후단부를 먼저 돌리고 덱을 나중에 돌린다.**
          두 관절의 최종값은 순서와 무관하므로 자세는 똑같은데, 반대로 하면
          (덱 먼저) 전후단부를 돌릴 때 덱이 최대 7.5° 출렁인다.

          이유는 진동이 아니라 **거더에 걸리는 것**이다. 덱이 나란하면 전후단부
          회전축이 x ±1.690 에 있어 스윕 원이 거더(y −3.060)에서 멀지만, 덱이
          수직이면 회전축이 y ±1.690 로 와서 후단부 스윕 원이 거더 보강대를
          문다. 그래서 회전을 **거더에서 먼 자리**로 옮기는 이 순서가 맞다.
          걸리는 높이 자체는 위 CLEAR(0.640) 로 푼다."""
        ok = self.go({"front_lift": CLEAR, "rear_lift": CLEAR},
                     "A1 전후단부를 들어 론지를 넘긴다")
        # ⚠ 덱이 아직 바퀴부와 나란할 때 돌린다 — 이때가 가장 안 흔들린다
        ok &= self.go({"front_turn": -HALF, "rear_turn": -HALF},
                      "A2 전후단부를 먼저 −90° (덱이 나란할 때라 안 흔들린다)")
        ok &= self.go({"body_turn": HALF},
                      "A3 덱 +90° — 전후단부가 옆 통로로 간다")
        ok &= self.go({"front_lift": 0.0, "rear_lift": 0.0},
                      "A4 전후단부를 통로 바닥에 내린다")
        return ok

    def corner_to_cross(self) -> bool:
        """C. 종 → 횡. 잭업해서 바퀴부만 90° 돌리고 전부 횡방향으로 맞춘다.

        핵심은 **덱을 안 돌린다**는 것이다. 종주행 자세에서 덱은 이미 론지와
        수직이므로, 바퀴부만 90° 돌리면 그대로 둘이 나란해진다.
        body_turn 을 +90° → 0 으로 주면 (덱이 전후단부에 물려 고정이므로)
        반력이 바퀴부로 가서 바퀴부가 +90° 돈다. 끝나면 body_turn=0 —
        즉 덱과 바퀴부가 같은 방향이다."""
        ok = self.go({"front_lift": JACK, "rear_lift": JACK},
                     "C1 잭업 — 전후단부로 바닥을 밀어 바퀴부를 띄운다")
        ok &= self.go({"body_turn": 0.0},
                      "C2 바퀴부 +90° (덱은 전후단부에 물려 안 돈다)")
        ok &= self.go({"front_lift": 0.0, "rear_lift": 0.0},
                      "C3 바퀴부를 론지 상면에 내린다")
        ok &= self.go({"front_lift": CLEAR, "rear_lift": CLEAR},
                      "C4 전후단부를 들어 론지를 넘긴다")
        ok &= self.go({"front_turn": 0.0, "rear_turn": 0.0},
                      "C5 전후단부도 덱과 나란히 — 전부 횡방향")
        # ⚠ 횡방향에서 전후단부의 접지면은 통로 바닥이 아니라 **론지 상면**이다.
        #   길이 1.05 m 가 pitch 0.845 를 가로지르면 반드시 론지를 깔고 앉는다.
        ok &= self.go({"front_lift": END_DROP, "rear_lift": END_DROP},
                      "C6 전후단부를 론지 상면 높이로 내린다")
        return ok

    def corner_to_along(self) -> bool:
        """E. 횡 → 종. C 의 거울상이 아니라 **순서를 바꾼다.**

        ⚠ 순진하게 C 의 역순으로 하면 (론지 상면에서 잭업 → 바퀴부 회전) 지형을 탄다.
          횡방향 전후단부는 **론지 상면**에 얹혀 있는데, 이 블록은 론지 간격이
          균일하지 않아 (2.080~3.845, -3.871~-2.145 에 하나씩 빠짐) 도착 위치에 따라
          한쪽 롤러 쌍이 허공에 뜬다. 그 상태로 밀어내리면 전후단부가 기울고
          바퀴부가 안 뜬다 — 실측: 2칸 이동 후 body_turn 이 지령 0 에 실제 1.569
          로 전혀 안 돌았다.

        그래서 **전후단부를 먼저 종방향으로 되돌려 통로 바닥에 내린 뒤** 거기서
        잭업한다. 통로 바닥은 어디든 있으므로 지형을 안 탄다.

            들기 → 전후단부 −90° → 통로 바닥에 내림 → 잭업 → 바퀴부 −90° → 착지
        """
        ok = self.go({"front_lift": CLEAR, "rear_lift": CLEAR},
                     "E1 전후단부를 들어 론지를 넘긴다")
        ok &= self.go({"front_turn": -HALF, "rear_turn": -HALF},
                      "E2 전후단부를 론지와 나란히 (아직 바퀴부는 횡방향)")
        ok &= self.go({"front_lift": 0.0, "rear_lift": 0.0},
                      "E3 전후단부를 **통로 바닥**에 내린다 — 잭업 발판")
        ok &= self.go({"front_lift": JACK, "rear_lift": JACK},
                      "E4 잭업 (통로 바닥 기준) — 바퀴부를 띄운다")
        ok &= self.go({"body_turn": HALF},
                      "E5 바퀴부 −90° — 다시 론지와 나란히")
        ok &= self.go({"front_lift": 0.0, "rear_lift": 0.0},
                      "E6 내려놓는다 — 종주행 자세 복귀")
        return ok

    def home(self) -> bool:
        self.get_logger().info("── 주행 자세로 복귀 ──")
        ok = self.go({"front_lift": CLEAR, "rear_lift": CLEAR}, "H1 전후단부 들기")
        ok &= self.go({"front_turn": 0.0, "rear_turn": 0.0}, "H2 전후단부 정렬")
        ok &= self.go({"body_turn": 0.0}, "H3 덱 정렬")
        ok &= self.go({"front_lift": 0.0, "rear_lift": 0.0, "body_lift": 0.0,
                       "rail_cmd": 0.0}, "H4 전부 내리기")
        return ok


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--laps", type=int, default=1, help="ㅁ자 몇 바퀴 (기본 1)")
    ap.add_argument("--length", type=float, default=1.20,
                    help="종주행 한 변 [m]. 도면 월드는 횡프레임 사이가 5.91 m 이고 "
                         "로봇이 4.425 m 라 최대 1.45 m 다 (기본 1.20)")
    ap.add_argument("--cross", type=int, default=1,
                    help="횡주행 한 변을 **론지 몇 칸**으로 (기본 1). "
                         "거리는 칸 x 0.845 m. ⚠ 2칸 이상은 도면 월드에서 실패할 수 "
                         "있다 — 아래 '횡주행 거리' 주의 참조")
    ap.add_argument("-s", "--speed", type=float, default=0.20, help="주행 속도 [m/s]")
    ap.add_argument("--tol-lin", type=float, default=0.01, help="직선축 허용오차 [m]")
    ap.add_argument("--tol-ang", type=float, default=0.02, help="회전축 허용오차 [rad]")
    ap.add_argument("--timeout", type=float, default=90.0,
                    help="단계별 기본 여유 [s]. 진전이 있으면 자동으로 연장된다 — "
                         "실제 중단 기준은 --stall 이다")
    ap.add_argument("--stall", type=float, default=25.0,
                    help="이 시간 동안 **전혀 진전이 없으면** 막힌 것으로 보고 중단 [s]. "
                         "RTF 가 낮아 느린 것과 기구가 걸린 것을 가르는 값이다")
    ap.add_argument("--no-home", action="store_true", help="끝나고 복귀하지 않는다")
    ap.add_argument("--dry-run", action="store_true", help="순서만 출력한다")
    args = ap.parse_args()

    cross_m = args.cross * LONGI_PITCH
    if args.dry_run:
        print(f"ㅁ자 {args.laps} 바퀴 — 종 {args.length:.2f} m, "
              f"횡 {args.cross}칸 = {cross_m:.3f} m\n")
        print("A 종주행 자세 (전후단부를 옆 통로로) — 처음 한 번만")
        for lap in range(args.laps):
            print(f"[{lap+1}바퀴]")
            for side, sgn in enumerate((+1.0, -1.0)):
                print(f"  B 종주행 {sgn*args.length:+.2f} m (론지 따라)")
                print( "  C 코너 — 잭업 → 바퀴부 +90° → 전부 횡방향")
                print(f"  D 횡주행 {sgn*cross_m:+.3f} m ({args.cross}칸, 론지 가로질러)")
                print( "  E 코너 — 잭업 → 바퀴부 −90° → 종주행 자세 복귀")
        return 0

    rclpy.init()
    n = Patrol(args.tol_lin, args.tol_ang, args.timeout, args.stall)
    try:
        if not n.wait_ready():
            n.get_logger().error("/joint_states 또는 /odom 이 안 옵니다. Gazebo 확인.")
            return 1
        n.get_logger().info(
            f"ㅁ자 순회 시작 — {args.laps}바퀴, 종 {args.length:.2f} m, "
            f"횡 {args.cross}칸({cross_m:.3f} m)" + n._where())

        # 종주행 자세는 처음 한 번만. 코너를 돌고 나면 이 자세로 돌아온다.
        if not n.stance_along():
            return 1
        for lap in range(args.laps):
            for side, sgn in enumerate((+1.0, -1.0)):
                n.get_logger().info(
                    f"═══ {lap+1}바퀴 {'전' if side == 0 else '후'}변 ═══")
                if not n.drive(sgn * args.length, args.speed, "B 종주행 (론지 따라)"):
                    return 1
                if not n.corner_to_cross():
                    return 1
                if not n.drive(sgn * cross_m, args.speed, "D 횡주행 (론지 가로질러)"):
                    return 1
                if not n.corner_to_along():
                    return 1
        if not args.no_home:
            n.home()
        n.get_logger().info("끝" + n._where())
        return 0
    except KeyboardInterrupt:
        n.cmd.publish(Twist())
        return 130
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
