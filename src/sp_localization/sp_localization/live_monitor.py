#!/usr/bin/env python3
"""참값 vs ESKF 실시간 창 — **`lap_eval` 과 똑같이** 평가하고 똑같이 그린다.

    ros2 run sp_localization monitor
    ros2 run sp_localization monitor --ros-args -p save:=/tmp/lap.png

위 첫 줄은 아래와 완전히 같다:

    ros2 run sp_localization lap_eval --ros-args -p away:=0.30 -p close:=0.15

━━ 무엇을 그리는가 ━━
창 하나에 셋 (`lap_eval` 과 동일):

  ① XY 궤적          참값과 ESKF 를 겹쳐 그린다. 현재 위치는 점.
  ② 위치 오차 RMSE   지금까지의 **누적** RMSE [mm] — 순간 오차가 아니다
  ③ 방위각 오차      |ESKF yaw − 참값 yaw| [°]

**한 바퀴를 돌면 자동으로 멈추고** 최종 수치를 창 위에 찍는다 (`away` 만큼
나갔다가 `close` 안으로 복귀). `patrol.py` 와 같이 띄워 쓰는 것이 원래 용도다.

━━ 왜 `lap_eval` 을 그대로 쓰는가 ━━
예전 `monitor` 는 자기만의 평가식(순간 오차)과 자기만의 4분할 그림을 갖고 있었다.
같은 것을 두 프로그램이 다르게 재고 다르게 그려서, 어느 숫자를 믿어야 하는지가
매번 헷갈렸다. 그래서 **평가·그림은 `lap_eval.py` 한 곳에만 두고**, 여기서는
기본값만 바꿔 부른다. 평가식이나 그림을 고치면 양쪽에 동시에 반영된다.

기본값 차이는 **한 바퀴 판정 거리 둘뿐**이다:

    away  0.6  → 0.30   (patrol --length 0.4 --cross 1 처럼 짧게 돌 때 맞다)
    close 0.25 → 0.15

⚠ 그래서 **휠 오도메트리(`/odom_slip`)는 이 창에 안 나온다.** `lap_eval` 이
  참값 대 ESKF 만 보기 때문이다. 셋을 나란히 보려면 `eval` 노드의 로그
  (`주행 x.xx m | 오도메트리 ... | ESKF ...`, 2초마다)나 `plot` 을 쓸 것.

파라미터는 `lap_eval` 과 같다 — `eskf_topic` / `away` / `close` / `rate` /
`timeout` / `save` / `headless`. 자세한 설명은 `lap_eval.py` 독스트링 참조.

    ros2 run sp_localization monitor --ros-args \\
        -p eskf_topic:=/odom_slip          # ESKF 대신 순수 오도메트리를 평가
    ros2 run sp_localization monitor --ros-args \\
        -p headless:=true -p save:=/tmp/lap.png    # 창 없이 PNG 만
"""
from __future__ import annotations

from sp_localization.lap_eval import main as lap_main

# `patrol.py --length 0.4 --cross 1` 한 바퀴(종 0.4 m × 횡 0.845 m)에 맞춘 값이다.
# 더 크게 돌리면 away 를 키울 것 — 안 그러면 코스 중간에서 '복귀'로 오판한다.
DEFAULTS = {"away": 0.30, "close": 0.15}


def main() -> None:
    lap_main(defaults=DEFAULTS, name="live_monitor")


if __name__ == "__main__":
    main()
