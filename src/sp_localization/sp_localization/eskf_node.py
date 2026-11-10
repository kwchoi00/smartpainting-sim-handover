#!/usr/bin/env python3
"""IMU + 휠 엔코더 오차상태 확장 칼만필터 (ESKF, Error-State EKF).

    ros2 run sp_localization eskf

구조는 `imu_odom.txt` 에 정리된 그대로다.

  예측 (IMU, 200 Hz)   : IMU 를 **시스템 입력**으로 써서 실제상태를 적분(mechanization)
                         하고 오차 공분산을 전파한다.
  보정 (엔코더, 50 Hz) : 엔코더의 **속도**를 관측으로 쓴다. 위치가 아니다 —
                         위치에는 누적오차가 섞여 있고 속도에는 없기 때문이다.
                         측면 속도 0(비홀로노믹 구속, NHC)도 같이 관측으로 넣는다.
  주입·리셋            : 추정한 오차상태를 실제상태에 더하고 오차상태는 0 으로 되돌린다.

실제상태 (9차원)
    X = [x, y, psi, vx, vy, bax, bay, bg, k]
      x, y      전역 2D 위치            [m]
      psi       전역 방위각             [rad]
      vx, vy    **전역 좌표계** 속도    [m/s]
      bax, bay  가속도계 바이어스(로봇 좌표계) [m/s^2]
      bg        자이로 바이어스         [rad/s]
      k         **엔코더 전진속도 스케일 오차** [-]   v_enc = (1+k) * v_true

오차상태 dX 도 같은 9차원이고, 필터가 실제로 추정하는 것은 이쪽이다.

━━ 왜 스케일 k 를 상태로 넣는가 ━━
바퀴 반경 오차·상시 slip 은 **거리에 비례해 누적되는 계통 오차**라 융합으로는
안 없어진다. 필터가 엔코더 속도를 관측으로 쓰는데 그 관측 자체가 치우쳐 있기
때문이다 (실측: 22.5 m 주행에 휠 897 mm / ESKF 824 mm — 융합 이득이 8 %뿐).

k 를 상태로 세우면 **정지-가속을 반복하는 동안 관측 가능**해진다:
  · 정지 중 ZUPT 가 속도를 0 으로 묶어 **가속도 바이어스**를 잡는다.
  · 가속 구간에서 IMU 적분 ∫(a - b_a) 는 참 속도 변화를, 엔코더는 (1+k)배를 준다.
    그 차이가 곧 k 다.
즉 **등속 구간에서는 관측 불가능하고, 가감속이 있어야 잡힌다.** patrol 처럼
서고 가고를 반복하는 동작에서 잘 잡히고, 한 방향 등속 주행만 하면 안 잡힌다.

⚠ 그래도 **절대 위치 관측을 대신하지는 못한다.** k 가 수렴하기 전 구간의 오차는
  이미 위치에 적분돼 남아 있고, k 자체도 실제로 변하면 따라가는 데 시간이 걸린다.

━━ 왜 이 구조인가 ━━
엔코더만으로도 위치까지 낼 수 있지만(그게 /odom 이다) 그 위치는 누적오차를 안고 간다.
반대로 엔코더 **속도**는 순간값이라 누적이 없다. 그래서 속도만 관측으로 떼어내
IMU 바이어스를 역산하고, 위치 적분은 필터 내부에 맡긴다.

━━ 이 로봇에서 주의할 점 ━━
· IMU 는 **바퀴부(base_link)** 에 있다. 몸체부는 /body_turn 으로 돌아가므로
  거기 실으면 방위각이 주행체 방위각이 아니게 된다.
· 이 로봇은 론지 위를 1차원으로만 달린다. 그래서 NHC(측면속도 0)가 매우 잘 맞고,
  yaw 를 흔드는 관측이 거의 없다. 대신 **몸체부·전후단부를 돌릴 때 반력으로
  바퀴부가 돌아가는데(실측 최대 0.83도), 그건 엔코더에 안 잡힌다.**
  자이로가 그걸 잡아주는지가 이 필터의 관전 포인트다.
· gz IMU 의 `orientation` 은 쓰지 않는다. 그건 물리엔진의 참자세라 반칙이다.
  각속도와 가속도만 쓴다.
· 중력은 z 로만 작용한다고 본다(2D 평면 모델). 로봇이 기울면 중력이 ax, ay 로
  새어 들어와 바이어스로 오인된다 — 론지 위 주행에서는 무시할 만하다.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

IX, IY, IPSI, IVX, IVY, IBAX, IBAY, IBG, IK = range(9)
N = 9


def wrap(a: float) -> float:
    """[-pi, pi) 로 접는다."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ESKF(Node):
    def __init__(self) -> None:
        super().__init__("eskf")

        p = self.declare_parameter
        # ── 잡음 파라미터 ──
        # ⚠ IMU 쪽은 params.xacro 의 노이즈 모델과 맞춰 둔 값이다. 실측 Allan variance
        #   로 IMU 모델을 갈아끼우면 **여기도 같이 바꿔야** 필터가 제 성능을 낸다.
        p("sigma_a", 0.0017)        # 가속도계 white noise [m/s^2/sqrt(Hz)]
        p("sigma_g", 0.0003)        # 자이로 white noise   [rad/s/sqrt(Hz)]
        p("sigma_ba", 1.0e-4)       # 가속도 바이어스 random walk [m/s^3]
        p("sigma_bg", 2.0e-6)       # 자이로 바이어스 random walk [rad/s^2]
        p("sigma_enc", 0.02)        # 엔코더 전진속도 관측 잡음 [m/s]
        # ⚠ **NHC 를 느슨하게 두면 그만큼 옆으로 흐른다.** 측면속도 추정 오차가
        #   그대로 적분돼 횡방향 위치가 random walk 로 쌓인다 (실측: 0.02 로 두고
        #   3.7 m 왕복했더니 y 가 38 mm 흘렀고, 그게 ESKF 위치오차의 대부분이었다).
        #   이 로봇은 바퀴가 론지 플랜지에 물려 1차원으로만 달려서 **측면속도가
        #   사실상 정확히 0** 이다. 일반 궤도차량 기준(0.02)보다 세게 믿는 게 맞다.
        p("sigma_nhc", 0.005)       # 측면속도(NHC) 관측 잡음 [m/s]
        p("use_enc_gyro", True)     # 엔코더 각속도를 관측으로 쓸지 (아래 ⚠)
        p("sigma_wenc", 0.01)       # 엔코더 각속도 관측 잡음 [rad/s]
        # ⚠ **바퀴가 안 구르는 회전을 걸러낸다.** 전후단부로 잭업해서 바퀴부를
        #   돌리면(patrol.py) 바퀴는 안 구르므로 엔코더는 "안 돈다"고 한다. 그 말을
        #   믿으면 필터가 회전을 통째로 놓친다 (실측 방위각 오차 88°).
        #   자이로와 엔코더가 이만큼 어긋나면 **그 관측만 버린다** — 나머지 구간에서는
        #   계속 써서 자이로 바이어스를 잡는다.
        #   0 이면 게이트를 끈다 (예전 동작).
        p("enc_gyro_gate", 0.05)    # |w_gyro - w_enc| 가 이보다 크면 버린다 [rad/s]
        # ⚠⚠ **차이만 보는 게이트로는 회전의 꼬리를 못 막는다.** 위치제어 회전은
        #   지수적으로 잦아들어서, 마지막 몇 도가 |w| < gate 구간에서 진행된다.
        #   그 구간에서는 차이도 작아 게이트를 통과하고, 필터는 "엔코더가 0 이라니
        #   지금 도는 건 전부 자이로 바이어스다" 라고 결론지어 **그 회전을 통째로
        #   삼킨다.** 실측: 잭업 90° 회전 한 번에 방위각 오차가 +4.0° 생겼고,
        #   반대로 도는 회전에서 그만큼 되돌아왔다 (= 회전량에 비례하는 스케일 오차).
        #   그래서 **자이로가 조금이라도 돌고 있으면 관측을 아예 안 쓴다**:
        p("enc_gyro_wmax", 0.005)   # |w_gyro| 가 이보다 크면 '회전 중'으로 본다 [rad/s]
        p("enc_gyro_hold", 2.0)     # 회전이 멎은 뒤에도 이만큼 더 쉰다 [s] (지수 꼬리)
        p("imu_topic", "/imu")
        p("odom_topic", "/odom")
        p("publish_tf", False)      # EKF 가 odom->base TF 를 소유할지
        p("frame_id", "odom")
        p("child_frame_id", "base_footprint")
        p("zupt", True)             # 정지 판정 시 속도 0 관측 추가
        p("zupt_thresh", 0.01)      # |v_enc| 가 이보다 작으면 정지로 본다 [m/s]
        # ── 엔코더 스케일 추정 ──
        # ⚠ 등속으로만 달리면 k 와 속도가 구분되지 않는다. 가감속·정지가 있어야
        #   관측된다 (위 독스트링 참조). 끄면 예전 8차원 필터와 완전히 같다.
        p("estimate_scale", True)   # 엔코더 스케일 오차 k 를 상태로 추정할지
        p("k_init_std", 0.05)       # k 의 초기 불확실도 [-] (기본 ±5 %)
        p("sigma_k", 1.0e-5)        # k 의 random walk [1/sqrt(s)] — 거의 상수다
        # ⚠⚠ **서 있을 때 k 를 건드리면 −1 로 붕괴한다.** 관측식이
        #     z = (1+k)·v 라서, v≈0 이면 어떤 k 든 식을 만족한다(관측 불가능).
        #     그 상태에서 필터의 v 에 조금이라도 오차가 있으면, 필터는 그것을
        #     "속도가 틀렸다"가 아니라 **"엔코더 스케일이 0 이다"** 로 설명해 버린다.
        #     그러면 1+k→0 이 되어 **엔코더가 속도를 전혀 관측하지 못하게 되고**,
        #     ZUPT 도 무력해져 위치가 폭주한다 (실측: 잭업 회전만 4번 반복했더니
        #     k −99.99 %, 추정 x 가 −18 m 까지 날아갔다).
        #     그래서 **바퀴가 실제로 구를 때만** k 를 갱신하고, 범위도 묶는다.
        p("scale_v_min", 0.05)      # |v_enc| 가 이보다 작으면 k 를 갱신하지 않는다 [m/s]
        p("scale_max", 0.25)        # |k| 상한 [-]. 이걸 넘는 스케일 오차는 없다
        # ⚠⚠ **혁신 게이트(이상치 배제)를 넣어 봤다가 뺐다 — 여기서는 해롭다.**
        #   동기는 옳았다: 잭업·착지 순간 IMU 가 순간적으로 큰 가속도를 읽고
        #   (실측 |a| 최대 14.7 m/s²) 그 충격으로 필터 속도가 튀는데, 관측행렬의
        #   psi 열이 속도에 비례하므로 그때 방위각이 엉뚱하게 끌려간다.
        #   그런데 이 필터는 **ZUPT 가 R 을 100배 조인다.** 그래서 S 가 아주 작고,
        #   정지 중 잡음 수준의 혁신도 3σ 를 넘는다 — 게이트가 **정작 필요한 순간의
        #   ZUPT 보정을 골라서 버린다.** 실측:
        #       통계 조건만 (3σ)             잭업 반복 시 위치오차 444 mm
        #       + 절대 문턱 (5·sqrt(R))      잭업 반복은 21 mm 로 정상, 그러나
        #                                    한 바퀴 주행에서 3523회 발동, 1394 mm
        #   → 게이트를 쓰려면 ZUPT 처럼 R 을 조이는 관측과 **같이 쓰면 안 된다.**

        g = lambda k: self.get_parameter(k).value          # noqa: E731
        self.s_a, self.s_g = g("sigma_a"), g("sigma_g")
        self.s_ba, self.s_bg = g("sigma_ba"), g("sigma_bg")
        self.s_enc, self.s_nhc = g("sigma_enc"), g("sigma_nhc")
        self.use_wenc, self.s_wenc = g("use_enc_gyro"), g("sigma_wenc")
        self.wenc_gate = g("enc_gyro_gate")
        self.wenc_wmax = g("enc_gyro_wmax")
        self.wenc_hold = g("enc_gyro_hold")
        self.turn_until = -1.0e18   # 이 시각까지는 '회전 중'으로 보고 관측을 쉰다
        self.n_gated = 0            # 차이 게이트에 걸려 버린 관측 수
        self.n_turning = 0          # 회전 중이라 안 쓴 관측 수
        self.use_tf = g("publish_tf")
        self.frame, self.child = g("frame_id"), g("child_frame_id")
        self.zupt, self.zupt_th = g("zupt"), g("zupt_thresh")
        self.est_k = g("estimate_scale")
        self.s_k = g("sigma_k") if self.est_k else 0.0
        self.k_vmin, self.k_max = g("scale_v_min"), g("scale_max")
        self.n_k = 0                # k 를 실제로 갱신한 관측 수

        # ── 상태 ──
        self.X = np.zeros(N)
        # ⚠ k 의 초기 분산이 0 이면 (estimate_scale=false) 관측이 k 를 못 건드리므로
        #   상태가 있어도 예전 8차원 필터와 수치적으로 동일하다.
        k_var = (g("k_init_std") ** 2) if self.est_k else 0.0
        self.P = np.diag([1e-6, 1e-6, 1e-6,    # 시작 위치·자세는 원점으로 확정
                          1e-4, 1e-4,          # 속도
                          1e-2, 1e-2,          # 가속도 바이어스 (turn-on bias 를 모른다)
                          1e-4,                # 자이로 바이어스
                          k_var])              # 엔코더 스케일
        self.t_imu: float | None = None
        self.wz_raw = 0.0           # 최근 자이로 원시값 (각속도 관측에 필요)
        self.n_imu = self.n_upd = 0

        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, g("imu_topic"), self.on_imu, qos)
        self.create_subscription(Odometry, g("odom_topic"), self.on_odom, 20)
        self.pub = self.create_publisher(Odometry, "/odom_eskf", 10)
        self.tf = TransformBroadcaster(self) if self.use_tf else None
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f"ESKF 시작 — IMU {g('imu_topic')} 로 예측, {g('odom_topic')} 속도로 보정. "
            f"추정 결과는 /odom_eskf")

    # ─────────────────────────── 예측 (IMU) ───────────────────────────
    def on_imu(self, m: Imu) -> None:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.t_imu is None:
            self.t_imu = t
            return
        dt = t - self.t_imu
        self.t_imu = t
        if dt <= 0.0 or dt > 0.5:      # 시뮬 리셋/점프 방어
            return
        self.n_imu += 1

        X = self.X
        # 바이어스를 뺀 순수 측정값
        self.wz_raw = m.angular_velocity.z
        wz = self.wz_raw - X[IBG]
        ab = np.array([m.linear_acceleration.x - X[IBAX],
                       m.linear_acceleration.y - X[IBAY]])

        # ① 실제상태 적분 (mechanization)
        psi_new = wrap(X[IPSI] + wz * dt)
        # 회전은 구간 중앙값으로 — 1차 적분보다 방위 오차가 작다
        psi_mid = wrap(X[IPSI] + 0.5 * wz * dt)
        c, s = math.cos(psi_mid), math.sin(psi_mid)
        R = np.array([[c, -s], [s, c]])            # 로봇 → 전역
        an = R @ ab                                 # 전역 좌표계 가속도

        X[IX] += X[IVX] * dt + 0.5 * an[0] * dt * dt
        X[IY] += X[IVY] * dt + 0.5 * an[1] * dt * dt
        X[IVX] += an[0] * dt
        X[IVY] += an[1] * dt
        X[IPSI] = psi_new

        # ② 오차 공분산 전파  P = F P F^T + Q
        F = np.eye(N)
        F[IX, IVX] = dt
        F[IY, IVY] = dt
        # 자세 오차가 속도 오차에 미치는 영향: d(R a)/d(psi) = R * [[0,-1],[1,0]] a
        Ra = np.array([-an[1], an[0]])              # = dR/dpsi @ ab
        F[IVX, IPSI] = Ra[0] * dt
        F[IVY, IPSI] = Ra[1] * dt
        # 가속도 바이어스 오차 → 속도 오차 (로봇→전역 회전을 거쳐 들어온다)
        F[IVX, IBAX] = -R[0, 0] * dt
        F[IVX, IBAY] = -R[0, 1] * dt
        F[IVY, IBAX] = -R[1, 0] * dt
        F[IVY, IBAY] = -R[1, 1] * dt
        # 자이로 바이어스 오차 → 자세 오차
        F[IPSI, IBG] = -dt

        Q = np.zeros((N, N))
        Q[IPSI, IPSI] = (self.s_g ** 2) * dt
        # 가속도 white noise 는 전역으로 회전돼 들어온다
        Qa = R @ np.diag([self.s_a ** 2, self.s_a ** 2]) @ R.T * dt
        Q[IVX:IVY + 1, IVX:IVY + 1] = Qa
        Q[IBAX, IBAX] = (self.s_ba ** 2) * dt
        Q[IBAY, IBAY] = (self.s_ba ** 2) * dt
        Q[IBG, IBG] = (self.s_bg ** 2) * dt
        Q[IK, IK] = (self.s_k ** 2) * dt

        self.P = F @ self.P @ F.T + Q
        self.publish(m.header.stamp)

    # ────────────────────── 보정 (엔코더 속도) ──────────────────────
    def on_odom(self, m: Odometry) -> None:
        """엔코더 **속도**만 쓴다. 위치(pose)는 누적오차라 일부러 버린다."""
        if self.t_imu is None:
            return
        v_enc = m.twist.twist.linear.x
        X = self.X
        c, s = math.cos(X[IPSI]), math.sin(X[IPSI])
        Rt = np.array([[c, s], [-s, c]])           # 전역 → 로봇

        # 예측된 관측치: 전역속도를 로봇 좌표계로 되돌린 것.
        # ⚠ 전진 성분에는 **엔코더 스케일 (1+k) 를 곱한다** — 엔코더는 참 속도가
        #   아니라 (1+k)배를 보고하기 때문이다. 측면(NHC)은 엔코더가 재는 값이
        #   아니라 기구학적 구속이므로 스케일이 안 붙는다.
        vb = Rt @ np.array([X[IVX], X[IVY]])       # 로봇 좌표계 속도
        kk = 1.0 + X[IK]
        z_hat = np.array([kk * vb[0], vb[1]])
        z = np.array([v_enc, 0.0])                 # 관측: [엔코더 전진속도, NHC 측면 0]

        H = np.zeros((2, N))
        H[0, IVX], H[0, IVY] = kk * Rt[0, 0], kk * Rt[0, 1]
        H[1, IVX], H[1, IVY] = Rt[1, 0], Rt[1, 1]
        # 자세 오차가 관측에 미치는 영향: d(R^T v)/d(psi)
        dRt = np.array([[-s, c], [-c, -s]])
        dz = dRt @ np.array([X[IVX], X[IVY]])
        H[0, IPSI], H[1, IPSI] = kk * dz[0], dz[1]
        # 스케일 오차가 관측에 미치는 영향: d(kk * vb_x)/dk = vb_x
        # ⚠ **바퀴가 구를 때만 연다.** 판정은 필터의 vb[0] 이 아니라 **측정값
        #   v_enc** 로 한다 — vb[0] 은 필터가 틀렸을 때 같이 틀리므로,
        #   그걸로 문을 열면 위에서 말한 붕괴를 스스로 부추긴다.
        if self.est_k and abs(v_enc) >= self.k_vmin:
            H[0, IK] = vb[0]
            self.n_k += 1

        rm = [self.s_enc ** 2, self.s_nhc ** 2]
        stopped = abs(v_enc) < self.zupt_th
        if self.zupt and stopped:
            # 정지 중이면 속도 0 을 더 세게 믿는다 — 바이어스를 잡을 최고의 기회다
            rm = [(self.s_enc * 0.1) ** 2, (self.s_nhc * 0.1) ** 2]
        # ⚠ **정지 중에 psi 열을 0 으로 막아 보고 싶어지는데, 하면 안 된다.**
        #   H 의 psi 열은 dRt @ v 라서 속도에 비례하고, 정지 중에는 원래 정보가
        #   거의 없다. 그래서 잭업·착지 충격으로 v 가 튀는 순간 이 열이 방위각을
        #   흔드는 것은 사실이다 (잭업 한 번에 1.1°).
        #   그런데 막아 봤더니 **오히려 32° 로 터졌다** (실측). 이 열이 psi 의
        #   공분산을 눌러 주는 유일한 관측이라, 막으면 P[psi,psi] 가 자라고
        #   자이로 바이어스 관측(H[·,b_g])의 교차항을 타고 psi 가 크게 끌려간다.
        #   → 잭업 충격의 1.1° 는 남겨 둔다. 손대려면 psi 를 직접 보는 관측
        #     (절대 방위각·지자기·LiDAR 정합)이 하나 더 있어야 한다.

        # ── 엔코더 각속도 관측 ──
        # ⚠ 이게 없으면 **자이로 바이어스가 사실상 관측되지 않는다.**
        #   직진만 하는 로봇에서는 NHC 만으로 yaw 를 잡을 수 없어서, 바이어스 잔차가
        #   그대로 적분돼 방위각이 흘러간다 (실측: 100초 남짓에 10.75도, 필터 자신도
        #   sigma_psi 5.2도로 "모르겠다"고 말한다).
        #   좌우 엔코더 속도차로 얻는 각속도는 **누적되지 않는 순간값**이라
        #   전진속도와 같은 자격으로 쓸 수 있고, 바이어스를 직접 관측한다:
        #       z_w = w_enc,   z_hat_w = w_gyro - b_g   →   dz_hat/d(db_g) = -1
        #
        #   ⚠⚠ **바퀴가 안 구르는 회전은 게이트로 걸러낸다** (enc_gyro_gate).
        #     전후단부로 잭업해서 바퀴부를 돌리면(patrol.py) 바퀴가 안 구르므로
        #     엔코더는 "안 돈다"고 한다. 자이로와 이만큼 어긋나면 그 한 줄만 버린다.
        #     그래야 회전도 따라가고, 나머지 구간에서 바이어스도 계속 잡는다.
        #
        #   ⚠⚠ 단, 이 로봇에서 엔코더 각속도는 **거의 항상 0** 이다 (론지 위 1차원 주행).
        #     즉 "차체는 안 돈다"는 강한 사전정보를 넣는 셈이다. 실제로 몸체부·전후단부를
        #     돌릴 때 반력으로 바퀴부가 돌아가는데(실측 0.83도) 그건 엔코더에 안 잡히므로,
        #     이 관측은 그 회전을 "없다"고 우겨 오히려 오차를 만든다.
        #     회전 동작 구간에서는 use_enc_gyro 를 끄고 보는 것이 맞다.
        if self.use_wenc:
            w_enc = m.twist.twist.angular.z
            w_gyro = self.wz_raw - X[IBG]
            # ── 게이트 ──
            #  ① 회전 중이면 아예 안 쓴다. 자이로가 조금이라도 돌면 그때부터
            #     hold 초 동안 쉰다 — 위치제어 회전의 **지수 꼬리**까지 덮는다.
            #     이걸 안 하면 꼬리 구간의 회전이 자이로 바이어스로 흡수된다.
            #  ② 그래도 어긋나면(바퀴가 헛돌거나 튀면) 그 한 줄만 버린다.
            t_now = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if self.wenc_wmax > 0.0 and abs(w_gyro) > self.wenc_wmax:
                self.turn_until = t_now + self.wenc_hold
            if t_now < self.turn_until:
                self.n_turning += 1
            elif self.wenc_gate > 0.0 and abs(w_gyro - w_enc) > self.wenc_gate:
                self.n_gated += 1
            else:
                z = np.append(z, w_enc)
                z_hat = np.append(z_hat, w_gyro)
                Hw = np.zeros((1, N))
                Hw[0, IBG] = -1.0
                H = np.vstack([H, Hw])
                rm.append(self.s_wenc ** 2)

        y = z - z_hat
        Rm = np.diag(rm)

        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        dX = K @ y

        # ── 주입 (Injection) ──
        self.X[:IPSI] += dX[:IPSI]
        self.X[IPSI] = wrap(self.X[IPSI] + dX[IPSI])
        self.X[IVX:] += dX[IVX:]
        # ⚠ 안전벨트. 게이트가 있어도 수치적으로 튈 수 있는데, 1+k 가 0 에 가까워지면
        #   엔코더가 속도를 못 보게 되어 필터가 통째로 무너진다. 범위를 묶어 둔다.
        self.X[IK] = float(np.clip(self.X[IK], -self.k_max, self.k_max))

        # 조셉 형식 — 대칭성·양정치성이 오래 유지된다
        I_KH = np.eye(N) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self.n_upd += 1
        # ── 리셋 ── 오차상태는 주입과 동시에 0 이 된다 (dX 를 따로 들고 있지 않다)

    # ─────────────────────────── 출력 ───────────────────────────
    def publish(self, stamp) -> None:
        X, P = self.X, self.P
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.frame
        o.child_frame_id = self.child
        o.pose.pose.position.x = float(X[IX])
        o.pose.pose.position.y = float(X[IY])
        o.pose.pose.orientation.z = math.sin(X[IPSI] / 2.0)
        o.pose.pose.orientation.w = math.cos(X[IPSI] / 2.0)
        # 전역속도를 로봇 좌표계로 되돌려 싣는다 (Odometry.twist 는 child_frame 기준)
        c, s = math.cos(X[IPSI]), math.sin(X[IPSI])
        o.twist.twist.linear.x = float(c * X[IVX] + s * X[IVY])
        o.twist.twist.linear.y = float(-s * X[IVX] + c * X[IVY])
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = P[IX, IX], P[IY, IY], P[IPSI, IPSI]
        o.pose.covariance = cov
        self.pub.publish(o)

        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.frame
            t.child_frame_id = self.child
            t.transform.translation.x = float(X[IX])
            t.transform.translation.y = float(X[IY])
            t.transform.rotation.z = o.pose.pose.orientation.z
            t.transform.rotation.w = o.pose.pose.orientation.w
            self.tf.sendTransform(t)

    def report(self) -> None:
        X = self.X
        self.get_logger().info(
            f"IMU {self.n_imu:6d} / 보정 {self.n_upd:5d} / "
            f"각속도 회전중 {self.n_turning:6d} 게이트 {self.n_gated:5d} / "
            f"k갱신 {self.n_k:6d} | "
            f"x {X[IX]:+.3f} y {X[IY]:+.3f} psi {math.degrees(X[IPSI]):+7.2f}° | "
            f"bias a ({X[IBAX]:+.4f}, {X[IBAY]:+.4f}) g {X[IBG]:+.6f} "
            f"k {X[IK]:+.4%}(σ{math.sqrt(max(self.P[IK, IK], 0)):.4%}) | "
            f"sig x {math.sqrt(max(self.P[IX, IX], 0)):.4f} "
            f"psi {math.degrees(math.sqrt(max(self.P[IPSI, IPSI], 0))):.3f}°")


def main() -> None:
    rclpy.init()
    node = ESKF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
