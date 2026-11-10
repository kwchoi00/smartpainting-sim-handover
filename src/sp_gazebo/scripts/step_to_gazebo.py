#!/usr/bin/env python3
"""STEP(.stp/.step) 도면을 Gazebo 모델로 변환한다.

조선 CAD 도면을 그대로 Gazebo 에 넣으면 거의 항상 세 가지가 문제가 된다.
이 스크립트는 그 셋을 자동으로 처리한다.

  1. 단위    — STEP 은 보통 mm, Gazebo 는 m. 바운딩박스로 추정해 스케일한다.
  2. 원점    — CAD 원점이 형상에서 수십 m 떨어져 있는 경우가 많다. 재정렬한다.
  3. 폴리곤 수 — B-rep 을 그대로 삼각화하면 충돌 메시가 수십만 삼각형이 되어
                물리 계산이 멈춘다. visual / collision 을 다른 해상도로 뽑는다.

사용:
    python3 step_to_gazebo.py block.stp -o src/sp_gazebo

    # 충돌 메시가 너무 무거우면 해상도를 낮춘다
    python3 step_to_gazebo.py block.stp -o src/sp_gazebo --collision-size 0.5

    # 단위를 알고 있으면 추정에 맡기지 말 것
    python3 step_to_gazebo.py block.stp -o src/sp_gazebo --scale 0.001

필요:
    pip install gmsh
"""
from __future__ import annotations

import argparse
import math
import shutil
import sys
from pathlib import Path

try:
    import gmsh
except ImportError:
    sys.exit("gmsh 가 필요합니다:  pip install gmsh")


# ── 유틸 ───────────────────────────────────────────────────────

def log(msg: str) -> None:
    print(f"[step2gz] {msg}", flush=True)


def bbox_of_all() -> tuple:
    """현재 OCC 모델 전체의 바운딩박스."""
    return gmsh.model.getBoundingBox(-1, -1)


def guess_scale(bbox: tuple) -> float:
    """바운딩박스 크기로 단위를 추정한다.

    선박 블록은 실제 크기가 수 m ~ 수십 m 다.
    수치가 1000 을 넘으면 mm, 100 을 넘으면 cm 로 본다.
    """
    xmin, ymin, zmin, xmax, ymax, zmax = bbox
    span = max(xmax - xmin, ymax - ymin, zmax - zmin)
    if span > 1000.0:
        return 0.001, "mm"
    if span > 100.0:
        return 0.01, "cm"
    return 1.0, "m"


def mesh_and_write(size_max: float, out_path: Path, label: str) -> int:
    """지정 해상도로 표면 메시를 생성하고 STL 로 쓴다. 삼각형 수를 반환."""
    gmsh.model.mesh.clear()
    gmsh.option.setNumber("Mesh.MeshSizeMax", size_max)
    gmsh.option.setNumber("Mesh.MeshSizeMin", size_max * 0.05)
    # CAD 곡률/경계를 메시 크기에 반영하지 않음 — 평판 구조라 균일 메시가 낫고 훨씬 빠르다
    gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
    gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
    gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)

    gmsh.model.mesh.generate(2)

    gmsh.option.setNumber("Mesh.Binary", 1)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    gmsh.write(str(out_path))

    tris = 0
    for etype, tags in zip(*gmsh.model.mesh.getElements(2)[:2]):
        if etype == 2:          # 3-node triangle
            tris += len(tags)
    size_mb = out_path.stat().st_size / 1e6
    log(f"{label}: 메시 크기 {size_max} m → 삼각형 {tris:,}개, {size_mb:.1f} MB → {out_path.name}")
    return tris


# ── SDF / config 생성 ──────────────────────────────────────────

MODEL_SDF = """<?xml version="1.0" ?>
<!--
  자동 생성 — 직접 수정하지 말 것.
  생성: sp_gazebo/scripts/step_to_gazebo.py
  원본: {source}

  스케일 {scale} 적용 (원본 단위 추정: {unit}), 원점 모드: {origin_mode}
  실제 크기: {sx:.2f} x {sy:.2f} x {sz:.2f} m
-->
<sdf version="1.10">
  <model name="{name}">
    <static>true</static>
    <link name="body">
      <visual name="visual">
        <geometry>
          <mesh><uri>model://{name}/meshes/visual.stl</uri></mesh>
        </geometry>
        <material>
          <ambient>0.40 0.42 0.45 1</ambient>
          <diffuse>0.48 0.50 0.53 1</diffuse>
          <specular>0.15 0.15 0.15 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <mesh><uri>model://{name}/meshes/collision.stl</uri></mesh>
        </geometry>
        <surface>
          <friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.10">model.sdf</sdf>
  <description>
    STEP 도면에서 변환한 선박 블록. 원본: {source}
  </description>
</model>
"""

WORLD_SDF = """<?xml version="1.0" ?>
<!--
  자동 생성 — 직접 수정하지 말 것.
  생성: sp_gazebo/scripts/step_to_gazebo.py

  최하단 데크 상면 z = {spawn_z:.3f} m (여유 0.05 포함).
  스폰 XY 는 inspect_block.py 로 주행 가능 영역을 확인한 뒤 정할 것 —
  데크에 개구부가 있으면 가운데로 스폰한 로봇이 그대로 빠진다.

  실행:
    ros2 launch sp_gazebo sim.launch.py world:={world_name} x:=<X> y:=<Y> z:={spawn_z:.2f}
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

    <!-- 내부 확인용 조명.
         선체 블록은 사방이 막힌 구조라 그림자를 켜면 내부가 새까맣게 된다.
         그림자를 끄고 환경광을 높이되, 완전 무지향으로 두면 면 구분이 사라져
         형상이 안 보이므로 서로 다른 방향의 약한 무그림자 조명 3개로 음영을 준다. -->
    <scene>
      <ambient>0.75 0.75 0.78 1</ambient>
      <background>0.15 0.16 0.18 1</background>
      <grid>false</grid>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="key_light">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.35 0.35 0.36 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>

    <light type="directional" name="fill_light">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.25 0.25 0.27 1</diffuse>
      <specular>0.0 0.0 0.0 1</specular>
      <direction>0.6 -0.5 -0.6</direction>
    </light>

    <!-- 아래에서 살짝 올려쳐 바닥판과 론지 옆면이 죽지 않게 -->
    <light type="directional" name="bounce_light">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 1 0 0 0</pose>
      <diffuse>0.18 0.18 0.20 1</diffuse>
      <specular>0.0 0.0 0.0 1</specular>
      <direction>0.3 0.7 0.4</direction>
    </light>

    <!-- 블록 외부 지면 -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>500 500</size></plane></geometry>
          <material>
            <ambient>0.10 0.10 0.11 1</ambient>
            <diffuse>0.10 0.10 0.11 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <include>
      <uri>model://{name}</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>

  </world>
</sdf>
"""


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", type=Path, help="입력 STEP 파일 (.stp / .step)")
    ap.add_argument("-o", "--output", type=Path, required=True,
                    help="sp_gazebo 패키지 경로 (models/ 와 worlds/ 를 여기 쓴다)")
    ap.add_argument("-n", "--name", default="ship_block_cad", help="Gazebo 모델 이름")
    ap.add_argument("--scale", type=float, default=None,
                    help="원본→m 스케일. 미지정 시 바운딩박스로 추정 (mm=0.001)")
    ap.add_argument("--visual-size", type=float, default=0.15,
                    help="visual 메시 최대 변 길이 [m] (기본 0.15)")
    ap.add_argument("--collision-size", type=float, default=0.40,
                    help="collision 메시 최대 변 길이 [m] (기본 0.40). "
                         "물리 성능에 직결되므로 visual 보다 크게 잡을 것")
    ap.add_argument("--up-axis", choices=["x", "y", "z"], default="z",
                    help="CAD 에서 '위'인 축. Gazebo 의 +Z 로 돌려세운다. "
                         "조선 CAD 는 Y-up 인 경우가 많다 — 부품 이름에 BOTTOM PLATE 가 "
                         "있으면 그 판의 법선 축이 up 축이다. 기본 z(회전 없음)")
    ap.add_argument("--origin", choices=["center", "floor-center", "min", "none"],
                    default="floor-center",
                    help="원점 재정렬. floor-center = XY 중심 + Z 최저면을 0 에 (기본)")
    ap.add_argument("--no-world", action="store_true", help="월드 SDF 생성 생략")
    ap.add_argument("--collision-warn", type=int, default=60000,
                    help="충돌 메시 삼각형 수 경고 임계값")
    args = ap.parse_args()

    if not args.step.is_file():
        sys.exit(f"파일을 찾을 수 없습니다: {args.step}")

    model_dir = args.output / "models" / args.name
    mesh_dir = model_dir / "meshes"

    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    try:
        log(f"STEP 로드: {args.step.name} ({args.step.stat().st_size/1e6:.1f} MB)")
        gmsh.model.occ.importShapes(str(args.step))
        gmsh.model.occ.synchronize()

        solids = gmsh.model.getEntities(3)
        faces = gmsh.model.getEntities(2)
        log(f"솔리드 {len(solids)}개, 면 {len(faces)}개")
        if not faces:
            sys.exit("면이 하나도 없습니다. STEP 이 비어 있거나 읽기에 실패했습니다.")

        # ── 1. 단위 ──
        bb = bbox_of_all()
        guessed, unit = guess_scale(bb)
        scale = args.scale if args.scale is not None else guessed
        raw_span = (bb[3]-bb[0], bb[4]-bb[1], bb[5]-bb[2])
        log(f"원본 바운딩박스: {raw_span[0]:.1f} x {raw_span[1]:.1f} x {raw_span[2]:.1f} "
            f"(단위 추정: {unit})")
        if args.scale is not None:
            log(f"스케일: {scale} (사용자 지정)")
        else:
            log(f"스케일: {scale} (자동 추정 — 틀렸으면 --scale 로 직접 주세요)")

        if scale != 1.0:
            gmsh.model.occ.dilate(gmsh.model.getEntities(), 0, 0, 0, scale, scale, scale)
            gmsh.model.occ.synchronize()

        # ── 1.5 up 축 정렬 ──
        # 조선 CAD 는 Y-up 으로 그리는 경우가 흔하다. 그대로 넣으면 블록이 옆으로 눕는다.
        # 원점 정렬(floor-center)은 z 최소값을 바닥으로 잡으므로 반드시 회전 뒤에 해야 한다.
        if args.up_axis == "y":
            # +Y → +Z : X 축 기준 +90°
            gmsh.model.occ.rotate(gmsh.model.getEntities(), 0, 0, 0, 1, 0, 0, math.pi / 2)
            gmsh.model.occ.synchronize()
            log("up 축 정렬: CAD +Y → Gazebo +Z (X축 +90° 회전)")
        elif args.up_axis == "x":
            # +X → +Z : Y 축 기준 -90°
            gmsh.model.occ.rotate(gmsh.model.getEntities(), 0, 0, 0, 0, 1, 0, -math.pi / 2)
            gmsh.model.occ.synchronize()
            log("up 축 정렬: CAD +X → Gazebo +Z (Y축 -90° 회전)")

        # ── 2. 원점 ──
        bb = bbox_of_all()
        xmin, ymin, zmin, xmax, ymax, zmax = bb
        if args.origin == "center":
            dx, dy, dz = -(xmin+xmax)/2, -(ymin+ymax)/2, -(zmin+zmax)/2
        elif args.origin == "floor-center":
            dx, dy, dz = -(xmin+xmax)/2, -(ymin+ymax)/2, -zmin
        elif args.origin == "min":
            dx, dy, dz = -xmin, -ymin, -zmin
        else:
            dx = dy = dz = 0.0

        if any(abs(v) > 1e-9 for v in (dx, dy, dz)):
            log(f"원점 이동 ({args.origin}): ({dx:.2f}, {dy:.2f}, {dz:.2f}) m")
            gmsh.model.occ.translate(gmsh.model.getEntities(), dx, dy, dz)
            gmsh.model.occ.synchronize()

        bb = bbox_of_all()
        sx, sy, sz = bb[3]-bb[0], bb[4]-bb[1], bb[5]-bb[2]
        log(f"최종 크기: {sx:.2f} x {sy:.2f} x {sz:.2f} m   "
            f"X[{bb[0]:.2f},{bb[3]:.2f}] Y[{bb[1]:.2f},{bb[4]:.2f}] Z[{bb[2]:.2f},{bb[5]:.2f}]")
        if max(sx, sy, sz) > 200 or max(sx, sy, sz) < 0.5:
            log("⚠ 크기가 선박 블록치고 이상합니다. --scale 을 직접 지정하세요.")

        # ── 3. 메시 ──
        mesh_and_write(args.visual_size, mesh_dir / "visual.stl", "visual")
        ctris = mesh_and_write(args.collision_size, mesh_dir / "collision.stl", "collision")

        if ctris > args.collision_warn:
            log(f"⚠ 충돌 메시 삼각형이 {ctris:,}개입니다 (임계 {args.collision_warn:,}). "
                f"물리 계산이 느려집니다 — --collision-size 를 키우세요.")

        # ── 4. 데크 탐지 ──
        # 블록 바닥이 z=0 에 있다고 가정하면 안 된다. 선박 블록은 다층이고,
        # 최하단은 바닥판이 없는 빈 구획인 경우가 흔하다. 실제 수평판 높이를 찾는다.
        decks = []
        for dim, tag in gmsh.model.getEntities(3):
            bx0, by0, bz0, bx1, by1, bz1 = gmsh.model.getBoundingBox(dim, tag)
            d = (bx1-bx0, by1-by0, bz1-bz0)
            if d.index(min(d)) == 2 and d[2] < 0.12 and d[0]*d[1] > 2.0:
                decks.append(round(bz1, 3))
        decks = sorted(set(decks))

    finally:
        gmsh.finalize()

    # ── 4. 모델/월드 파일 ──
    fmt = dict(name=args.name, source=args.step.name, scale=scale, unit=unit,
               origin_mode=args.origin, sx=sx, sy=sy, sz=sz)
    (model_dir / "model.sdf").write_text(MODEL_SDF.format(**fmt), encoding="utf-8")
    (model_dir / "model.config").write_text(MODEL_CONFIG.format(**fmt), encoding="utf-8")
    log(f"모델: {model_dir}")

    world_name = f"{args.name}_world"
    if not args.no_world:
        if decks:
            spawn_z = decks[0] + 0.05
            log(f"수평 데크 {len(decks)}개 감지: " +
                ", ".join(f"z={d:.3f}" for d in decks))
        else:
            spawn_z = 0.3
            log("⚠ 수평 데크를 찾지 못했습니다. 스폰 높이를 직접 정하세요.")

        world_path = args.output / "worlds" / f"{world_name}.sdf"
        world_path.parent.mkdir(parents=True, exist_ok=True)
        world_path.write_text(
            WORLD_SDF.format(world_name=world_name, spawn_z=spawn_z, **fmt),
            encoding="utf-8")
        log(f"월드: {world_path}")
        log("")
        log("빌드:")
        log("  colcon build --packages-select sp_gazebo && source install/setup.bash")
        log("")
        log("⚠ 스폰 XY 는 반드시 확인하세요. 선박 블록 데크에는 맨홀·경감공이 뚫려 있어")
        log("  가운데로 스폰하면 로봇이 구멍으로 빠집니다. 아래로 주행 가능 영역을 확인:")
        log(f"  python3 {Path(__file__).name} 대신 → inspect_block.py {args.step.name} "
            f"--robot-width <로봇폭>")
        log("")
        log(f"확인 후: ros2 launch sp_gazebo sim.launch.py world:={world_name} "
            f"x:=<X> y:=<Y> z:={spawn_z:.2f}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
