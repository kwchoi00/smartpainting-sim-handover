#!/usr/bin/env python3
"""참값 vs ESKF 를 실시간 3분할로 보고, **한 바퀴 돌면 멈춰서 결과를 남긴다.**

    ros2 run sp_localization lap_eval
    ros2 run sp_localization lap_eval --ros-args -p save:=/tmp/lap.png
    ros2 run sp_localization lap_eval --ros-args -p headless:=true -p save:=/tmp/lap.png

창 하나에 셋을 그린다:

  ① XY 궤적          참값과 ESKF 를 겹쳐 그린다. 현재 위치는 점.
  ② 위치 오차 RMSE   지금까지의 **누적** RMSE [mm] — 순간 오차가 아니다
  ③ 방위각 오차      |ESKF yaw − 참값 yaw| [°]

`patrol.py` 로 ㅁ자 한 바퀴를 돌리면서 같이 띄우면, 출발점으로 돌아온 순간
자동으로 멈추고 최종 수치를 창에 찍는다.

━━ 왜 첫 샘플로 정렬하는가 ━━
`/ground_truth` 는 **월드 절대좌표**이고 `/odom_eskf` 는 **스폰 지점 기준**이다.
그대로 겹쳐 그리면 궤적이 통째로 어긋난다. 그래서 각자의 첫 샘플을 원점·0도로
맞춘 뒤 **상대 궤적**으로 비교한다. 드리프트 평가는 상대 비교라 이게 맞다.
방위각 오차도 같은 이유로 0 에서 시작한다.

⚠ `/ground_truth` 는 **평가 전용**이다. 여기 그린다고 추정에 쓰이는 게 아니다 —
  ESKF 는 `/imu` 와 `/odom_slip` 만 본다.

━━ 한 바퀴 판정 ━━
출발점에서 `away` 이상 멀어졌다가 다시 `close` 안으로 들어오면 한 바퀴로 본다.
patrol.py 의 기본 한 바퀴는 종 1.2 m × 횡 0.845 m 라 away 0.6 / close 0.25 가 맞다.
짧게 돌리면(`--length 0.4`) away 를 0.3 쯤으로 낮출 것.

파라미터:
    eskf_topic  (기본 /odom_eskf)   비교 대상. /odom_slip 으로 바꾸면 순수 오도메트리
    away        (기본 0.6)  [m]  이만큼 멀어져야 '나갔다'고 본다
    close       (기본 0.25) [m]  이 안으로 들어오면 '돌아왔다'
    rate        (기본 10)   [Hz] 표본 주기
    timeout     (기본 1800) [s]  이 시간이 지나면 한 바퀴 못 채워도 끝낸다
    save        (기본 "")        PNG 경로. 주면 끝나고 저장한다
    headless    (기본 false)     창을 안 띄운다 (원격·배치 실행용)
"""
from __future__ import annotations

import math
import os
import threading

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(o) -> float:
    return math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                      1.0 - 2.0 * (o.y * o.y + o.z * o.z))


def wrap(a: float) -> float:
    """[-pi, pi) 로 접는다. 이걸 안 하면 ±180° 근처에서 오차가 360° 로 튄다."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Frame:
    """첫 샘플을 원점·0도로 맞춰 상대 궤적을 만든다.

    ⚠ 위치만 빼면 안 된다. 두 토픽의 **기준 방위각도 다르므로** 첫 yaw 만큼
      되돌려 회전시켜야 궤적이 겹친다."""

    def __init__(self) -> None:
        self.ref: tuple[float, float, float] | None = None

    def to_local(self, x: float, y: float, psi: float) -> tuple[float, float, float]:
        if self.ref is None:
            self.ref = (x, y, psi)
        rx, ry, rp = self.ref
        c, s = math.cos(-rp), math.sin(-rp)
        dx, dy = x - rx, y - ry
        return (c * dx - s * dy, s * dx + c * dy, wrap(psi - rp))


class LapEval(Node):
    """⚠ 이 클래스와 아래 `main` 은 `monitor`(live_monitor.py)도 그대로 쓴다.
    둘의 차이는 **기본값뿐**이므로, 평가식이나 그림을 고치면 양쪽에 동시에 반영된다.
    일부러 그렇게 뒀다 — 예전에는 두 벌이 따로 놀아서 같은 것을 다르게 그렸다."""

    def __init__(self, defaults: dict | None = None, name: str = "lap_eval") -> None:
        super().__init__(name)
        d = defaults or {}
        # 기본값을 호출자가 덮어쓸 수 있게 한다. --ros-args -p 로 준 값이 여전히 최우선이다.
        p = lambda k, v: self.declare_parameter(k, d.get(k, v))   # noqa: E731
        p("eskf_topic", "/odom_eskf")
        p("away", 0.6)
        p("close", 0.25)
        p("rate", 10.0)
        p("timeout", 1800.0)
        p("save", "")
        p("headless", False)
        g = lambda k: self.get_parameter(k).value          # noqa: E731

        self.away, self.close = g("away"), g("close")
        self.timeout = g("timeout")
        self.f_gt, self.f_ek = Frame(), Frame()
        self.gt: tuple[float, float, float] | None = None
        self.ek: tuple[float, float, float] | None = None

        # 표본 (그리기 전용 — 락으로 보호한다. ROS 스레드와 GUI 스레드가 같이 본다)
        self.lock = threading.Lock()
        self.t: list[float] = []
        self.gx: list[float] = []
        self.gy: list[float] = []
        self.ex: list[float] = []
        self.ey: list[float] = []
        self.rmse: list[float] = []      # [mm] 누적
        self.dyaw: list[float] = []      # [deg] 절대값
        self.sq_sum = 0.0                # RMSE 누적용 제곱합

        self.t0: float | None = None
        self.went_out = False
        self.done = False
        self.done_reason = ""

        self.create_subscription(Odometry, "/ground_truth", self.on_gt, 20)
        self.create_subscription(Odometry, g("eskf_topic"), self.on_ek, 20)
        self.create_timer(1.0 / g("rate"), self.sample)
        self.get_logger().info(
            f"참값 vs {g('eskf_topic')} — 한 바퀴(출발점에서 {self.away} m 이상 "
            f"나갔다가 {self.close} m 안으로 복귀)면 자동 종료")

    # ── 콜백 ──
    def on_gt(self, m: Odometry) -> None:
        self.gt = self.f_gt.to_local(m.pose.pose.position.x, m.pose.pose.position.y,
                                     yaw_of(m.pose.pose.orientation))

    def on_ek(self, m: Odometry) -> None:
        self.ek = self.f_ek.to_local(m.pose.pose.position.x, m.pose.pose.position.y,
                                     yaw_of(m.pose.pose.orientation))

    def sample(self) -> None:
        if self.done or self.gt is None or self.ek is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0

        gx, gy, gpsi = self.gt
        ex, ey, epsi = self.ek
        err = math.hypot(ex - gx, ey - gy)
        self.sq_sum += err * err
        n = len(self.t) + 1

        with self.lock:
            self.t.append(t)
            self.gx.append(gx); self.gy.append(gy)
            self.ex.append(ex); self.ey.append(ey)
            self.rmse.append(1000.0 * math.sqrt(self.sq_sum / n))
            self.dyaw.append(abs(math.degrees(wrap(epsi - gpsi))))

        # ── 한 바퀴 판정 (참값 기준. 추정으로 판정하면 드리프트에 속는다) ──
        d0 = math.hypot(gx, gy)
        if not self.went_out and d0 > self.away:
            self.went_out = True
            self.get_logger().info(f"출발점에서 {d0:.2f} m — 나갔다")
        elif self.went_out and d0 < self.close:
            self.finish(f"한 바퀴 완주 (출발점 {d0*1000:.0f} mm 이내 복귀)")
        elif t > self.timeout:
            self.finish(f"제한시간 {self.timeout:.0f} s 초과 — 한 바퀴를 못 채웠다")

    def finish(self, reason: str) -> None:
        if self.done:
            return
        self.done, self.done_reason = True, reason
        self.get_logger().info(f"■ {reason}")
        # ⚠ 요약은 print 로 낸다. Ctrl+C 로 끝낼 때는 rclpy 컨텍스트가 먼저 내려가
        #   get_logger() 가 조용히 사라진다 (그래서 예전엔 결과가 안 보였다).
        print(self.summary(), flush=True)

    def summary(self) -> str:
        with self.lock:
            if not self.t:
                return "표본 없음"
            path = sum(math.hypot(self.gx[i] - self.gx[i-1], self.gy[i] - self.gy[i-1])
                       for i in range(1, len(self.gx)))
            fin = math.hypot(self.ex[-1] - self.gx[-1], self.ey[-1] - self.gy[-1])
            return ("결과 — "
                    f"주행 {path:.2f} m | 표본 {len(self.t)} | "
                    f"최종 RMSE {self.rmse[-1]:.1f} mm | "
                    f"최종 위치오차 {1000*fin:.1f} mm | "
                    f"방위각 오차 최대 {max(self.dyaw):.2f}° / 최종 {self.dyaw[-1]:.2f}°")


def main(defaults: dict | None = None, name: str = "lap_eval") -> None:
    rclpy.init()
    node = LapEval(defaults, name)
    save = node.get_parameter("save").value
    headless = node.get_parameter("headless").value

    import matplotlib
    # ⚠ 화면이 없으면(원격·배치) 창을 못 열고 죽는다. headless 를 안 줬어도 떨어뜨린다.
    if headless or not os.environ.get("DISPLAY"):
        matplotlib.use("Agg")          # 창 없이 PNG 만
        headless = True
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    # ⚠ 한글 폰트를 안 잡으면 라벨이 전부 □ 로 나온다. 설치된 것 중에서 고른다.
    #   (Ubuntu 기본 이미지에는 Noto Sans CJK 가 들어 있다)
    import matplotlib.font_manager as fm
    have = {f.name for f in fm.fontManager.ttflist}
    for cand in ("NanumGothic", "Malgun Gothic", "Noto Sans CJK KR",
                 "Noto Sans CJK JP", "UnDotum", "Baekmuk Gulim"):
        if cand in have:
            plt.rcParams["font.family"] = cand
            break
    else:
        node.get_logger().warn(
            "한글 폰트를 못 찾았다 — 라벨이 깨진다. "
            "`sudo apt install fonts-nanum` 후 matplotlib 캐시를 지울 것")
    plt.rcParams["axes.unicode_minus"] = False   # 한글 폰트에 −(U+2212)가 없다

    # ROS 는 별도 스레드에서 돌린다 — matplotlib 은 메인 스레드를 요구한다
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    fig = plt.figure(figsize=(13, 5.2))
    fig.canvas.manager.set_window_title(f"참값 vs ESKF — 한 바퀴 ({name})")
    ax1 = fig.add_subplot(1, 3, 1)
    ax2 = fig.add_subplot(1, 3, 2)
    ax3 = fig.add_subplot(1, 3, 3)

    l_gt, = ax1.plot([], [], "-", lw=1.6, label="참값")
    l_ek, = ax1.plot([], [], "-", lw=1.6, label="ESKF")
    p_gt, = ax1.plot([], [], "o", ms=6)
    p_ek, = ax1.plot([], [], "o", ms=6)
    ax1.set_title("① XY 궤적 [m]"); ax1.set_xlabel("x"); ax1.set_ylabel("y")
    ax1.grid(alpha=.3); ax1.legend(loc="best", fontsize=9); ax1.set_aspect("equal")

    l_rm, = ax2.plot([], [], "-", lw=1.6)
    ax2.set_title("② 위치 오차 RMSE (누적) [mm]"); ax2.set_xlabel("t [s]")
    ax2.grid(alpha=.3)

    l_dy, = ax3.plot([], [], "-", lw=1.6)
    ax3.set_title("③ |방위각 오차| [°]"); ax3.set_xlabel("t [s]")
    ax3.grid(alpha=.3)

    banner = fig.text(0.5, 0.965, "", ha="center", fontsize=10)

    # ⚠ 저장 지점이 둘이다 (완주 시 upd 안, 그리고 종료 시 finally).
    #   플래그가 없으면 헤드리스에서 같은 파일을 두 번 쓰고 로그도 두 줄 나온다.
    saved = False

    def store(path: str) -> None:
        nonlocal saved
        try:
            fig.savefig(path, dpi=130, bbox_inches="tight")
            print(f"그래프 저장 → {path}", flush=True)
            saved = True
        except Exception as e:                        # noqa: BLE001
            print(f"그래프 저장 실패: {e}", flush=True)

    def autoscale(ax, xs, ys, pad=0.08, square=False):
        if not xs:
            return
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
        if square:
            cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
            r = max(x1 - x0, y1 - y0, 0.2) / 2 * (1 + pad)
            ax.set_xlim(cx - r, cx + r); ax.set_ylim(cy - r, cy + r)
        else:
            dx = max((x1 - x0) * pad, 1e-3); dy = max((y1 - y0) * pad, 1e-3)
            ax.set_xlim(x0 - dx, x1 + dx); ax.set_ylim(min(0, y0 - dy), y1 + dy)

    def upd(_):
        with node.lock:
            t = list(node.t); gx = list(node.gx); gy = list(node.gy)
            ex = list(node.ex); ey = list(node.ey)
            rm = list(node.rmse); dy = list(node.dyaw)
        if not t:
            banner.set_text("표본 기다리는 중 — Gazebo 와 ESKF 가 떠 있는지 확인")
            return
        l_gt.set_data(gx, gy); l_ek.set_data(ex, ey)
        p_gt.set_data([gx[-1]], [gy[-1]]); p_ek.set_data([ex[-1]], [ey[-1]])
        l_rm.set_data(t, rm); l_dy.set_data(t, dy)
        autoscale(ax1, gx + ex, gy + ey, square=True)
        autoscale(ax2, t, rm)
        autoscale(ax3, t, dy)
        banner.set_text(
            (f"■ {node.done_reason}   {node.summary()}" if node.done else
             f"주행 중 — t {t[-1]:5.1f} s | RMSE {rm[-1]:6.1f} mm | "
             f"|Δyaw| {dy[-1]:5.2f}°"))
        if node.done:
            if ani is not None:
                ani.event_source.stop()
            if save and not saved:
                store(save)
            if headless:
                plt.close(fig)

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    # ⚠ 헤드리스면 FuncAnimation 을 만들지 않는다 — 한 번도 렌더 안 하고 폐기돼서
    #   matplotlib 이 "Animation was deleted without rendering anything" 경고를 낸다.
    ani = None if headless else FuncAnimation(fig, upd, interval=200,
                                              cache_frame_data=False)
    try:
        if headless:
            # 창이 없으니 끝날 때까지 기다렸다가 한 번 그려서 저장한다
            import time
            t0 = time.time()
            # ⚠ rclpy.ok() 를 같이 봐야 한다 — rclpy 는 SIGINT 를 가로채 컨텍스트만
            #   내리고 KeyboardInterrupt 를 안 던질 때가 있어서, 이게 없으면
            #   Ctrl+C 를 줘도 이 루프가 안 끝난다.
            while (not node.done and rclpy.ok()
                   and time.time() - t0 < node.timeout + 60):
                time.sleep(0.5)
            upd(None)
        else:
            plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        # ⚠ Ctrl+C 면 rclpy 가 컨텍스트를 먼저 내려서 get_logger() 가 조용히 사라진다.
        #   마지막 수치는 print 로 낸다 (summary() 가 이미 "결과 —" 로 시작한다).
        if not node.done:
            node.finish("사용자 중단")      # 여기서 요약도 같이 낸다
        if save and not saved:
            store(save)
        # ⚠ **순서가 중요하다.** spin 이 도는 채로 destroy_node 하면 종료할 때
        #   C++ 쪽에서 terminate 가 뜬다. shutdown → join → destroy 순으로 내린다.
        if rclpy.ok():
            rclpy.shutdown()
        spin.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:                             # noqa: BLE001
            pass


if __name__ == "__main__":
    main()
