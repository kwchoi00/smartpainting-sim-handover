#!/usr/bin/env python3
"""평가 CSV 를 궤적·오차 그래프로 그린다.

    ros2 run sp_localization plot /tmp/run1.csv
    ros2 run sp_localization plot /tmp/run1.csv -o /tmp/run1.png

CSV 는 `eval` 노드가 `csv:=` 인자로 남긴 것이다.

네 칸으로 그린다.
  ① XY 궤적        — 참값 / 휠 오도메트리 / ESKF 를 겹쳐 본다. 시작점이 원점.
  ② 위치 오차-시간 — 드리프트가 시간에 따라 어떻게 불어나는지
  ③ 위치 오차-거리 — 같은 것을 주행거리 기준으로. 스케일 오차면 **직선**으로 는다
  ④ 방위각 오차    — yaw 가 흘러가는지

⚠ 이 과제의 질문은 "얼마나 정확한가" 가 아니라
  **"허용 오차를 넘기까지 얼마나 갈 수 있는가"** 다. ②③의 기울기가 그 답이다.
"""
from __future__ import annotations

import argparse
import csv as csvmod
import math
import sys


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("csv", help="eval 노드가 남긴 CSV")
    ap.add_argument("-o", "--out", default=None, help="PNG 경로 (기본: CSV 옆)")
    ap.add_argument("--tol", type=float, default=0.075,
                    help="허용 오차 선 [m] (기본 0.075 = 론지 플랜지 반폭)")
    a = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    # 한글 라벨이 깨지지 않게 CJK 폰트를 찾아 쓴다. 없으면 기본 폰트로 두고
    # 경고만 나오게 한다 (그래프 자체는 읽을 수 있다).
    from matplotlib import font_manager
    for cand in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic",
                 "Malgun Gothic", "AppleGothic"):
        if any(f.name == cand for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = cand
            break
    else:
        print("⚠ CJK 폰트를 못 찾았습니다. 한글 라벨이 깨질 수 있습니다.",
              file=sys.stderr)
    plt.rcParams["axes.unicode_minus"] = False

    rows = list(csvmod.DictReader(open(a.csv)))
    if not rows:
        print("CSV 가 비었습니다.", file=sys.stderr)
        return 1
    f = lambda k: [float(r[k]) for r in rows]        # noqa: E731
    t = f("t")
    gx, gy, gyaw = f("gt_x"), f("gt_y"), f("gt_yaw")
    ox, oy, oyaw = f("odom_x"), f("odom_y"), f("odom_yaw")
    ex, ey, eyaw = f("eskf_x"), f("eskf_y"), f("eskf_yaw")
    oe = [v * 1000 for v in f("odom_err")]
    ee = [v * 1000 for v in f("eskf_err")]

    # 참값 기준 누적 주행거리
    dist, d = [0.0], 0.0
    for i in range(1, len(gx)):
        d += math.hypot(gx[i] - gx[i - 1], gy[i] - gy[i - 1])
        dist.append(d)

    C = {"gt": "#111111", "odom": "#d1495b", "eskf": "#0077b6"}
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(f"자기위치 추정 비교  —  {a.csv}   (주행 {dist[-1]:.2f} m, "
                 f"{t[-1]:.0f} s)", fontsize=13)

    # ① XY 궤적
    p = ax[0][0]
    p.plot(gx, gy, color=C["gt"], lw=2.5, label="참값 (ground truth)")
    p.plot(ox, oy, color=C["odom"], lw=1.6, ls="--", label="휠 오도메트리 (slip)")
    p.plot(ex, ey, color=C["eskf"], lw=1.6, label="ESKF (IMU+엔코더)")
    p.plot(gx[0], gy[0], "o", color=C["gt"], ms=8)
    p.set_title("① XY 궤적 (시작점 정렬)")
    p.set_xlabel("x [m]"); p.set_ylabel("y [m]")
    p.axis("equal"); p.grid(alpha=0.3); p.legend()

    # ② 위치 오차 - 시간
    p = ax[0][1]
    p.plot(t, oe, color=C["odom"], label="휠 오도메트리")
    p.plot(t, ee, color=C["eskf"], label="ESKF")
    p.axhline(a.tol * 1000, color="gray", ls=":", label=f"허용 {a.tol*1000:.0f} mm")
    p.set_title("② 위치 오차 vs 시간")
    p.set_xlabel("t [s]"); p.set_ylabel("오차 [mm]")
    p.grid(alpha=0.3); p.legend()

    # ③ 위치 오차 - 주행거리
    p = ax[1][0]
    p.plot(dist, oe, color=C["odom"], label="휠 오도메트리")
    p.plot(dist, ee, color=C["eskf"], label="ESKF")
    p.axhline(a.tol * 1000, color="gray", ls=":")
    p.set_title("③ 위치 오차 vs 주행거리  (스케일 오차면 직선으로 는다)")
    p.set_xlabel("주행거리 [m]"); p.set_ylabel("오차 [mm]")
    p.grid(alpha=0.3); p.legend()

    # ④ 방위각 오차
    p = ax[1][1]
    wrap = lambda v: (v + math.pi) % (2 * math.pi) - math.pi   # noqa: E731
    p.plot(t, [math.degrees(wrap(o - g)) for o, g in zip(oyaw, gyaw)],
           color=C["odom"], label="휠 오도메트리")
    p.plot(t, [math.degrees(wrap(e - g)) for e, g in zip(eyaw, gyaw)],
           color=C["eskf"], label="ESKF")
    p.axhline(0, color="gray", lw=0.8)
    p.set_title("④ 방위각 오차")
    p.set_xlabel("t [s]"); p.set_ylabel("오차 [°]")
    p.grid(alpha=0.3); p.legend()

    out = a.out or (a.csv.rsplit(".", 1)[0] + ".png")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"저장: {out}")
    print(f"  주행 {dist[-1]:.2f} m / {t[-1]:.0f} s")
    print(f"  휠 오도메트리  최종 {oe[-1]:7.1f} mm   최대 {max(oe):7.1f} mm")
    print(f"  ESKF           최종 {ee[-1]:7.1f} mm   최대 {max(ee):7.1f} mm")
    if dist[-1] > 0.1:
        print(f"  거리당 오차   오도메트리 {oe[-1]/dist[-1]:6.1f} mm/m, "
              f"ESKF {ee[-1]/dist[-1]:6.1f} mm/m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
