#!/usr/bin/env python3
"""선박 블록 내부 Gazebo 월드(SDF) 생성기.

론지 개수·간격 같은 파라미터를 바꿔가며 월드를 다시 뽑아야 하므로
SDF 를 손으로 쓰지 않고 YAML 로부터 생성한다.

사용법:
    python3 gen_world.py config/block_mockup.yaml -o worlds/ship_block_mockup.sdf

모든 형상은 기본 도형(box)으로만 만든다. 메시가 없으므로
GZ_SIM_RESOURCE_PATH 설정 없이 어디서든 로드된다.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit("PyYAML 이 필요합니다:  pip install pyyaml   (또는 apt install python3-yaml)")


# ── SDF 조각 헬퍼 ──────────────────────────────────────────────

def static_box(name: str, size: tuple, pose: tuple, rgba: str) -> str:
    """정적 박스 링크 하나. pose = (x, y, z)."""
    sx, sy, sz = size
    px, py, pz = pose
    return f"""
    <link name="{name}">
      <pose>{px:.4f} {py:.4f} {pz:.4f} 0 0 0</pose>
      <collision name="{name}_col">
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <surface>
          <friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="{name}_vis">
        <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>
        <material>
          <ambient>{rgba}</ambient>
          <diffuse>{rgba}</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>"""


STEEL = "0.42 0.45 0.48 1"
STEEL_DARK = "0.30 0.32 0.35 1"
PRIMER = "0.55 0.35 0.20 1"   # 프라이머 도장색 — 론지 구분용


# ── 구성 요소 ──────────────────────────────────────────────────

def build_plate(cfg: dict) -> tuple:
    """바닥 플레이트. 반환: (sdf, 상면 z)."""
    p = cfg["plate"]
    base_z = p["height"] if p.get("raised") else 0.0
    # 박스 중심 z. 상면이 base_z 에 오도록 두께의 절반만큼 내린다.
    center_z = base_z - p["thickness"] / 2.0
    sdf = static_box(
        "deck_plate",
        (p["length"], p["width"], p["thickness"]),
        (0.0, 0.0, center_z),
        STEEL,
    )

    # 띄운 경우, 추락하지 않도록 받치는 기둥 4개 (시각적 지지대)
    if p.get("raised") and base_z > 0.05:
        for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
            sdf += static_box(
                f"deck_support_{i}",
                (0.2, 0.2, base_z),
                (sx * p["length"] * 0.4, sy * p["width"] * 0.4, base_z / 2.0),
                STEEL_DARK,
            )
    return sdf, base_z


def build_longis(cfg: dict, deck_z: float) -> str:
    """론지(종방향 보강재). X 축으로 길게, Y 로 pitch 간격 배열."""
    lg = cfg["longi"]
    if not lg.get("enabled"):
        return ""
    p = cfg["plate"]
    n = lg["count"]
    sdf = ""

    # count 개를 Y=0 기준 대칭 배치
    for i in range(n):
        y = (i - (n - 1) / 2.0) * lg["pitch"]

        # 수직 웨브
        sdf += static_box(
            f"longi_{i}_web",
            (p["length"], lg["web_thickness"], lg["web_height"]),
            (0.0, y, deck_z + lg["web_height"] / 2.0),
            PRIMER,
        )

        # 상부 플랜지
        fz = deck_z + lg["web_height"] + lg["flange_thickness"] / 2.0
        if lg["type"].upper() == "T":
            # T바: 웨브 중심 기준 양쪽 대칭
            sdf += static_box(
                f"longi_{i}_flange",
                (p["length"], lg["flange_width"], lg["flange_thickness"]),
                (0.0, y, fz),
                PRIMER,
            )
        else:
            # ㄱ자(L바): 한쪽으로만 뻗음
            sdf += static_box(
                f"longi_{i}_flange",
                (p["length"], lg["flange_width"], lg["flange_thickness"]),
                (0.0, y + lg["flange_width"] / 2.0, fz),
                PRIMER,
            )
    return sdf


def build_transverse(cfg: dict, deck_z: float) -> str:
    """횡방향 웨브 프레임. 중앙에 맨홀(통로)을 남긴다."""
    tr = cfg["transverse"]
    if not tr.get("enabled"):
        return ""
    p = cfg["plate"]
    n = tr["count"]
    t = tr["thickness"]
    mw, mh = tr["manhole_width"], tr["manhole_height"]
    sdf = ""

    for i in range(n):
        # X 방향 균등 배치 (양 끝단은 피함)
        x = (i + 1) / (n + 1) * p["length"] - p["length"] / 2.0

        # 맨홀 좌/우 벽
        side_w = (p["width"] - mw) / 2.0
        if side_w > 0.01:
            for sgn, tag in ((1, "left"), (-1, "right")):
                y = sgn * (mw / 2.0 + side_w / 2.0)
                sdf += static_box(
                    f"tframe_{i}_{tag}",
                    (t, side_w, tr["height"]),
                    (x, y, deck_z + tr["height"] / 2.0),
                    STEEL_DARK,
                )

        # 맨홀 상부 인방(lintel)
        lintel_h = tr["height"] - mh
        if lintel_h > 0.01:
            sdf += static_box(
                f"tframe_{i}_lintel",
                (t, mw, lintel_h),
                (x, 0.0, deck_z + mh + lintel_h / 2.0),
                STEEL_DARK,
            )
    return sdf


def build_side_walls(cfg: dict, deck_z: float) -> str:
    """측면 외판."""
    sw = cfg["side_wall"]
    if not sw.get("enabled"):
        return ""
    p = cfg["plate"]
    sdf = ""
    for sgn, tag in ((1, "port"), (-1, "starboard")):
        sdf += static_box(
            f"side_wall_{tag}",
            (p["length"], sw["thickness"], sw["height"]),
            (0.0, sgn * p["width"] / 2.0, deck_z + sw["height"] / 2.0),
            STEEL,
        )
    return sdf


# ── 월드 조립 ──────────────────────────────────────────────────

WORLD_TEMPLATE = """<?xml version="1.0" ?>
<!--
  자동 생성 파일 — 직접 수정하지 말 것.
  생성: sp_gazebo/scripts/gen_world.py
  원본: {source}
-->
<sdf version="1.10">
  <world name="{world_name}">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gravity>0 0 -9.8066</gravity>

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.15 0.16 0.18 1</background>
      <grid>false</grid>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <!-- 블록 외부 지면. 플레이트를 띄운 경우 추락 지점이 된다. -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material>
            <ambient>0.10 0.10 0.11 1</ambient>
            <diffuse>0.10 0.10 0.11 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- ═══ 선박 블록 구조 ═══ -->
    <model name="ship_block">
      <static>true</static>{body}
    </model>

  </world>
</sdf>
"""


def generate(cfg: dict, source: str) -> str:
    plate_sdf, deck_z = build_plate(cfg)
    body = plate_sdf
    body += build_longis(cfg, deck_z)
    body += build_transverse(cfg, deck_z)
    body += build_side_walls(cfg, deck_z)
    return WORLD_TEMPLATE.format(
        world_name=cfg["world_name"],
        body=body,
        source=source,
    )


def check(cfg: dict) -> list:
    """로봇이 통과 못 하는 형상을 미리 잡아낸다. 치수를 바꿀 때마다 유용."""
    warnings = []
    # ⚠ sp_description/urdf/params.xacro 의 overall_width / overall_height 와 맞출 것.
    #    거기 치수를 바꾸면 여기도 같이 고쳐야 검사가 유효하다.
    ROBOT_WIDTH = 0.63    # wheel_separation(0.55) + wheel_width(0.08)
    ROBOT_HEIGHT = 0.39   # ground_clearance + body_height + IMU

    lg = cfg.get("longi", {})
    if lg.get("enabled"):
        clear = lg["pitch"] - lg["web_thickness"]
        if lg["type"].upper() == "T":
            clear -= lg["flange_width"]
        if clear < ROBOT_WIDTH:
            warnings.append(
                f"론지 통로 유효폭 {clear:.3f}m < 로봇 폭 {ROBOT_WIDTH}m — 로봇이 끼입니다. "
                f"longi.pitch 를 키우거나 flange_width 를 줄이세요."
            )

    tr = cfg.get("transverse", {})
    if tr.get("enabled"):
        if tr["manhole_width"] < ROBOT_WIDTH:
            warnings.append(
                f"맨홀 폭 {tr['manhole_width']}m < 로봇 폭 {ROBOT_WIDTH}m — 통과 불가."
            )
        if tr["manhole_height"] < ROBOT_HEIGHT:
            warnings.append(
                f"맨홀 높이 {tr['manhole_height']}m < 로봇 높이 {ROBOT_HEIGHT}m — 통과 불가."
            )
    return warnings


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("config", type=Path, help="블록 형상 YAML")
    ap.add_argument("-o", "--output", type=Path, required=True, help="출력 SDF 경로")
    args = ap.parse_args()

    cfg = yaml.safe_load(args.config.read_text(encoding="utf-8"))

    for w in check(cfg):
        print(f"[warn] {w}", file=sys.stderr)

    sdf = generate(cfg, args.config.name)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(sdf, encoding="utf-8")

    n_longi = cfg["longi"]["count"] if cfg["longi"].get("enabled") else 0
    print(f"생성: {args.output}  (world={cfg['world_name']}, 론지 {n_longi}개)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
