#!/usr/bin/env python3
"""로봇 STEP(`모바일 로봇 ASSY.stp`) 을 링크별 STL 메시로 쪼갠다.

블록 변환기(`step_to_gazebo.py`)는 도면 하나를 **정적 모델 하나**로 만든다.
로봇은 그럴 수 없다 — 움직이는 부분이 있으니 **링크별로 나눠야** URDF 에 붙는다.
이 스크립트가 하는 일은 그 분해다.

  1. 단위·up 축·원점  — 블록 변환기와 같은 처리 (mm, Y-up, 접지면을 z=0 으로)
  2. 솔리드 → 링크 분류 — 부품명 규칙 + 위치 규칙 (아래 classify)
  3. 링크별 STL       — 각 링크 **자기 좌표계** 로 옮겨서 쓴다.
                        URDF 의 <visual> 에 origin 을 따로 줄 필요가 없다.

기계 구성 (도면 실측, 로봇 좌표 X = 주행방향, 길이 4.10 m):

    X   -2.05        -1.02   -0.87    +0.88   +1.02        +2.05
        │◀── 후단부 ──▶│      │◀ 바퀴부 ▶│      │◀── 전단부 ──▶│
        └ 무한궤도 (z 0~0.126)         └ 무한궤도
          + 회전부 400 dia (z 0.394~0.454)
    ───────────────── 상판 4.10 m (z 0.454~0.464) ─────────────────
                      + HSR25 리니어축 + 캐리지 + CRX-20iA/L

  **가운데 바퀴부**가 론지 위에 얹혀 론지를 따라 주행하고,
  **양 끝단부**는 론지가 아니라 바닥에 닿는다 (그래서 상하 스트로크가 필요하다 —
  론지 상면과 바닥은 450 mm 차이다). 끝단부와 상판은 각각 회전축을 갖는다.

  도면은 **전부 한 평면에 놓인 자세**(운반 자세)로 그려져 있다. 바퀴부 하단 z=0.008,
  끝단 궤도 하단 z=0.000 으로 거의 같다. 론지 주행 자세는 URDF 에서 만든다.

⚠ 도면에는 실제 부품이 아닌 솔리드가 섞여 있다.
  FANUC CRX 의 `J2ARM_UNIT_LONG` / `J3ARM_UNIT` 은 체적이 6 L / 1.3 L 인데
  바운딩박스가 4.6 × 2.2 × 3.1 m 다. 팔 링크가 아니라 **작동영역(또는 잘못 배치된
  인스턴스)** 이다. 그대로 메시로 뽑으면 로봇이 방만 한 덩어리가 된다. 버린다.

사용:
    python3 step_to_robot.py "모바일 로봇 ASSY.stp" -o src/sp_description

필요:
    pip install gmsh
"""
from __future__ import annotations

import argparse
import json
import math
import re
import struct
import sys
from collections import defaultdict
from pathlib import Path

try:
    import gmsh
except ImportError:
    sys.exit("gmsh 가 필요합니다:  pip install gmsh")


def log(msg: str) -> None:
    print(f"[step2robot] {msg}", flush=True)


# ── 솔리드 → 링크 분류 ────────────────────────────────────────────
#
# 도면 부품명이 사내 도번(NEWRVB2405xxxxx)이라 의미를 알 수 없다.
# 대신 **어느 서브어셈블리에 속하는가** 와 **어디에 있는가** 로 가른다.
# 아래 상수는 전부 원본 CAD 좌표(mm, Y-up)다. 도면이 바뀌면 여기를 고친다.

DROP_NAMES = ("J2ARM_UNIT_LONG", "J3ARM_UNIT")   # 위 ⚠ 참조

# 아래 경계는 전부 **원본 CAD 좌표(mm)** 다. 로봇 좌표로 옮기면
#   X_robot = X_cad*0.001 - 4.758,  Z_robot = Y_cad*0.001 - 2.1993
# 도면이 바뀌면 이 숫자들만 고치면 된다.
DECK_BOTTOM_CAD_Y = 2650.0   # 상판(파트1) 하면. 이 위는 전부 상부 구조다
WHEEL_UNIT_X0 = 3870.0       # 가운데 바퀴부 CAD x 범위 (robot X -0.888 ~ +0.890)
WHEEL_UNIT_X1 = 5648.0
# 끝단부는 바퀴부 바깥. 앞/뒤는 조립체 중심(4758)으로 가른다.
ASSY_CENTER_CAD_X = 4758.0

# ── 끝단부는 **한 덩어리가 아니다** ──
#   윗부분(회전부 턴테이블)은 몸체부에 붙어 **회전만** 하고,
#   아랫부분(궤도)은 시저로 매달려 **바닥까지 내려간다.**
#   도면 서브어셈블리가 셋으로 딱 갈라진다 (로봇 z 기준):
#       회전부 Ø400        0.394 ~ 0.454   → 윗부분
#       NEWRVB240523200    0.109 ~ 0.384   → 시저 (앞뒤 1.002 m, 유닛에 꽉 참)
#       NEWRVB240524000    0.000 ~ 0.126   → 아랫부분(궤도)
#   경계는 그 사이를 잡는다. 이름이 없는 솔리드(볼트·브래킷 등)는 중심 높이로 가른다.
END_LOWER_TOP_CAD_Y   = 2325.3   # robot z 0.126 — 궤도 상면
END_SCISSOR_TOP_CAD_Y = 2583.3   # robot z 0.384 — 시저 상단 = 회전부 하면

# 도면의 시저 형상은 **메시에서 뺀다.** 접힌 자세로 굳어 있어서 그대로 쓰면
# 아랫부분을 내렸을 때 늘어나지 않고 허공에 남는다. URDF 가 같은 자리에
# 파라메트릭 X자 바를 세워 대체한다 (바퀴부 도면 체인을 뺀 것과 같은 이유).
END_SCISSOR_NAMES = ("NEWRVB240523200",)


def classify(name: str, bb: tuple) -> str:
    """솔리드 하나를 링크 이름으로 분류한다. bb 는 원본 CAD 좌표 바운딩박스.

    링크는 다섯 개다:
      wheel_unit  가운데 바퀴부 — 인접한 두 론지에 양쪽 바퀴를 걸치고 구른다 (base_link)
      wheel_chain 바퀴부의 도면 체인 — **메시에서 제외**한다 (아래 참조)
      deck        상판 + 리니어 레일 — 바퀴부 위에서 상하·회전
      carriage    리니어축 캐리지 (상판 위를 X 로 달린다)
      arm         FANUC CRX-20iA/L + 툴
      front_upper / rear_upper — 끝단부 **윗부분**(회전부). 몸체부에 붙어 회전만 한다
      front_lower / rear_lower — 끝단부 **아랫부분**(궤도). 시저로 바닥까지 내려간다
      end_scissor — 도면의 시저 형상. **메시에서 제외**한다 (위 상수 절 참조)
    """
    if any(d in name for d in DROP_NAMES):
        return "drop"
    if "CRX-20IA" in name or re.search(r"NEWRVB2405151\d\d", name):
        return "arm"                       # 팔 본체 + 툴
    if re.search(r"NEWRVB2405150\d\d", name) or any(
            k in name for k in ("HSR25M1CSSB", "KBS 40", "NJ2205", "40x55x8")):
        return "carriage"                  # 리니어축 캐리지 + 팔 마운트

    cx = (bb[0] + bb[3]) / 2.0
    # 상판·레일: 조립체 전체에 걸친 판이라 X 로는 못 가른다. 높이로 가른다.
    if bb[1] >= DECK_BOTTOM_CAD_Y:
        return "deck"
    # ⚠ **X 범위를 이름 규칙보다 먼저** 본다.
    #   가운데 바퀴부(NEWRVB240510000)도 자기 체인·스프라켓을 갖고 있다
    #   (체인 690솔리드, X ±0.877, 궤간 y ±0.227 — 끝단 궤도는 ±0.271).
    #   "체인" 이름만 보고 끝단부로 보내면 바퀴부가 껍데기만 남는다.
    if WHEEL_UNIT_X0 <= cx <= WHEEL_UNIT_X1:
        # 바퀴부의 도면 체인·스프라켓은 **메시에서 뺀다.**
        # 도면 궤간(421.5 mm)으로는 론지 플랜지(150 mm)에 얹히지 않아서,
        # URDF 에서 론지 pitch(845 mm) 궤간의 바퀴 여러 개로 갈아끼웠기 때문이다.
        # 도면 체인을 같이 그리면 통로 허공에 떠 있는 것처럼 보인다.
        # (치수 기준으로는 계속 쓰므로 ref 에는 남는다.)
        if any(k in name for k in ("체인", "DRC80", "RS80")):
            return "wheel_chain"
        return "wheel_unit"
    # ── 끝단부: 윗부분 / 시저 / 아랫부분으로 가른다 ──
    side = "front" if cx > ASSY_CENTER_CAD_X else "rear"
    if any(k in name for k in END_SCISSOR_NAMES):
        return "end_scissor"                    # 메시에서 뺀다
    cy = (bb[1] + bb[4]) / 2.0                  # 솔리드 중심 높이 (CAD Y-up)
    if cy >= END_SCISSOR_TOP_CAD_Y:
        return f"{side}_upper"                  # 회전부 — 몸체부에 붙어 회전만 한다
    if cy < END_LOWER_TOP_CAD_Y:
        return f"{side}_lower"                  # 궤도 — 시저로 내려간다
    return "end_scissor"                        # 그 사이는 전부 시저 영역


# ── 바이너리 STL 쓰기 ──────────────────────────────────────────────
# gmsh.write() 는 모델 전체를 쓴다. 링크별로 잘라야 하므로 직접 쓴다.

def box_tris(b: tuple) -> list:
    """축정렬 박스 → 삼각형 12개. (x0,y0,z0,x1,y1,z1)"""
    x0, y0, z0, x1, y1, z1 = b
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 3, 2), (0, 2, 1),      # bottom
             (4, 5, 6), (4, 6, 7),      # top
             (0, 1, 5), (0, 5, 4),      # -y
             (2, 3, 7), (2, 7, 6),      # +y
             (1, 2, 6), (1, 6, 5),      # +x
             (3, 0, 4), (3, 4, 7)]      # -x
    return [(v[a], v[b_], v[c]) for a, b_, c in faces]


def write_stl(path: Path, tris: list) -> None:
    """삼각형 목록(이미 링크 좌표계, m)을 바이너리 STL 로 쓴다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(b"generated by step_to_robot.py".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for (ax, ay, az), (bx, by, bz), (cx, cy, cz) in tris:
            ux, uy, uz = bx - ax, by - ay, bz - az
            vx, vy, vz = cx - ax, cy - ay, cz - az
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            n = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            f.write(struct.pack("<12fH", nx / n, ny / n, nz / n,
                                ax, ay, az, bx, by, bz, cx, cy, cz, 0))


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", type=Path, help="로봇 STEP 파일")
    ap.add_argument("-o", "--output", type=Path, required=True,
                    help="sp_description 패키지 경로 (meshes/ 를 여기 쓴다)")
    ap.add_argument("--scale", type=float, default=0.001, help="원본→m (기본 mm)")
    ap.add_argument("--up-axis", choices=["x", "y", "z"], default="y",
                    help="CAD 의 '위' 축. 이 도면은 블록과 같은 Y-up 이다")
    ap.add_argument("--mesh-size", type=float, default=0.02,
                    help="메시 최대 변 길이 [m]. visual 전용이므로 물리에 영향 없음")
    ap.add_argument("--min-volume", type=float, default=5e-6,
                    help="이보다 작은 솔리드는 메시에서 뺀다 [m³]. "
                         "기본 5 cm³ — 볼트·핀을 빼서 삼각형 수를 줄인다")
    ap.add_argument("--cache", type=Path, default=None,
                    help="형상 캐시 디렉터리 (기본 <output>/.step_cache). "
                         "STEP 파싱만 13분이 걸리므로 BREP + 분류 결과를 캐시한다. "
                         "--no-cache 로 끌 수 있다")
    ap.add_argument("--no-cache", action="store_true", help="캐시를 쓰지도 만들지도 않는다")
    ap.add_argument("--proxy", action="store_true",
                    help="**박스 프록시 모드.** 삼각화 대신 각 솔리드의 바운딩박스를 "
                         "그대로 상자로 찍어 STL 을 만든다. 수 초면 끝나고 절대 실패하지 "
                         "않는다. 이 도면은 면이 42,678개라 정식 삼각화가 2시간을 넘고 "
                         "중간에 못 다루는 회전면에서 멎는다 — 형상 확인·물리 시뮬에는 "
                         "프록시로 충분하다. 체인 링크가 작은 상자로 찍혀 궤도 모양도 산다.")
    args = ap.parse_args()

    if not args.step.is_file():
        sys.exit(f"파일을 찾을 수 없습니다: {args.step}")

    mesh_dir = args.output / "meshes"
    cache_dir = args.cache or (args.output / ".step_cache" / args.step.stem)
    cache_brep, cache_meta = cache_dir / "geom.brep", cache_dir / "meta.json"

    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    gmsh.option.setNumber("Geometry.OCCImportLabels", 1)   # 부품명이 있어야 분류된다
    try:
        # ── 0. 캐시 ──
        # 이 도면은 **STEP 파싱만 13분**이 걸린다. 메시 전략을 몇 번 바꿔보려면
        # 그때마다 13분을 다시 낼 수 없다. BREP(형상) + 분류 결과를 캐시한다.
        # BREP 에는 부품명이 안 들어가므로 tag→그룹 대응을 JSON 으로 같이 저장하고,
        # 되읽을 때 **솔리드 개수와 바운딩박스로 대응이 맞는지 검증**한다.
        use_cache = (not args.no_cache and cache_brep.is_file() and cache_meta.is_file()
                     and cache_meta.stat().st_mtime > args.step.stat().st_mtime)
        meta = None
        if use_cache:
            log(f"형상 캐시 사용: {cache_brep}")
            gmsh.model.occ.importShapes(str(cache_brep))
            gmsh.model.occ.synchronize()
            meta = json.loads(cache_meta.read_text(encoding="utf-8"))
            if meta["nsolid"] != len(gmsh.model.getEntities(3)):
                sys.exit(f"캐시가 STEP 과 안 맞습니다 ({cache_dir}). 지우고 다시 돌리세요.")
        else:
            log(f"STEP 로드: {args.step.name} "
                f"({args.step.stat().st_size/1e6:.1f} MB) — 수 분 걸린다")
            gmsh.model.occ.importShapes(str(args.step))
            gmsh.model.occ.synchronize()

        solids = gmsh.model.getEntities(3)
        log(f"솔리드 {len(solids)}개, 면 {len(gmsh.model.getEntities(2))}개")

        # ── 1. 분류는 **원본 CAD 좌표**에서 한다 (규칙 상수가 그 좌표계다) ──
        groups: dict[str, list] = defaultdict(list)
        volumes: dict[str, float] = defaultdict(float)
        # 링크 원점을 잡을 기준 부품. 그룹 바운딩박스 중심을 쓰면 브래킷 하나에
        # 끌려다녀서 값이 안 예쁘다. 실제 기구 기준면을 쓴다.
        ref: dict[str, list] = defaultdict(list)
        skipped = 0
        cache_rows = []
        for dim, tag in solids:
            bb = gmsh.model.getBoundingBox(dim, tag)
            if meta is not None:
                row = meta["solids"].get(str(tag))
                if row is None:
                    sys.exit(f"캐시에 솔리드 {tag} 가 없습니다. 캐시를 지우고 다시 돌리세요.")
                if max(abs(a - b) for a, b in zip(row["bb"], bb)) > 1e-3:
                    sys.exit(f"캐시 솔리드 {tag} 의 위치가 다릅니다. 캐시를 지우고 다시 돌리세요.")
                name, vol = row["name"], row["vol"]
            else:
                name = gmsh.model.getEntityName(dim, tag) or ""
                vol = gmsh.model.occ.getMass(3, tag) * args.scale ** 3
                cache_rows.append((tag, name, vol, [round(v, 4) for v in bb]))

            cx = (bb[0] + bb[3]) / 2.0
            if "회전부" in name:                      # 끝단부 턴테이블 = 회전축
                ref["turn_front" if cx > ASSY_CENTER_CAD_X else "turn_rear"].append(tag)
            if "체인" in name:                        # 궤도 접지면·궤간 기준
                if WHEEL_UNIT_X0 <= cx <= WHEEL_UNIT_X1:
                    ref["chain_mid"].append(tag)
                else:
                    ref["chain_front" if cx > ASSY_CENTER_CAD_X
                        else "chain_rear"].append(tag)
            if "HSR25M1CSSB" in name:               # 리니어 가이드 슬라이더
                ref["slider"].append(tag)
            if "HSR25M1C1390030" in name:           # 리니어 가이드 레일
                ref["rail"].append(tag)
            g = classify(name, bb)
            if g == "drop":
                log(f"  버림: {name.split('/')[-1][:50]} "
                    f"(bbox {bb[3]-bb[0]:.0f}×{bb[4]-bb[1]:.0f}×{bb[5]-bb[2]:.0f} mm)")
                continue
            volumes[g] += vol
            if vol < args.min_volume:
                skipped += 1
                continue
            groups[g].append(tag)
        log(f"분류: " + ", ".join(f"{k} {len(v)}개" for k, v in sorted(groups.items())))
        log(f"작은 부품 {skipped}개는 메시에서 제외 (< {args.min_volume*1e6:.0f} cm³)")

        if meta is None and not args.no_cache:
            cache_dir.mkdir(parents=True, exist_ok=True)
            gmsh.write(str(cache_brep))
            cache_meta.write_text(json.dumps({
                "step": args.step.name, "nsolid": len(solids),
                "solids": {str(t): {"name": n, "vol": v, "bb": bb}
                           for t, n, v, bb in cache_rows},
            }, ensure_ascii=False), encoding="utf-8")
            log(f"형상 캐시 저장: {cache_dir}  (다음 실행부터 STEP 파싱을 건너뛴다)")

        # ── 2. 좌표 변환 ──
        # ⚠ **형상을 건드리지 않는다.** OCC 로 dilate(0.001) 하면 이 도면에서
        #   "BSpline curve: Knots interval values too close" 로 죽는다 (mm → m 로
        #   줄이면 노트 간격이 허용오차 밑으로 내려간다). 어차피 STL 을 직접 쓰므로
        #   **좌표만 숫자로 변환**한다. 빠르고 실패하지 않는다.
        s = args.scale

        def to_robot(p, off=(0.0, 0.0, 0.0)):
            """원본 CAD 점 → 로봇 좌표계 [m]. up 축 회전 + 스케일 + 원점 이동."""
            x, y, z = p[0] * s, p[1] * s, p[2] * s
            if args.up_axis == "y":        # +Y → +Z (X축 +90°)
                x, y, z = x, -z, y
            elif args.up_axis == "x":      # +X → +Z (Y축 -90°)
                x, y, z = -z, y, x
            return (x + T[0] - off[0], y + T[1] - off[1], z + T[2] - off[2])

        T = (0.0, 0.0, 0.0)                # 원점 이동. 아래에서 확정한다

        def bbox_of(tags, off=(0.0, 0.0, 0.0)):
            """CAD 바운딩박스들을 로봇(또는 링크) 좌표계 축정렬 박스로 합친다."""
            pts = []
            for tg in tags:
                b = gmsh.model.getBoundingBox(3, tg)
                for cx in (b[0], b[3]):
                    for cy in (b[1], b[4]):
                        for cz in (b[2], b[5]):
                            pts.append(to_robot((cx, cy, cz), off))
            return (min(p[0] for p in pts), min(p[1] for p in pts), min(p[2] for p in pts),
                    max(p[0] for p in pts), max(p[1] for p in pts), max(p[2] for p in pts))

        def group_bbox(g):
            return bbox_of(groups[g])

        # 로봇 원점: 조립체 XY 중심, z=0 은 **가장 낮은 접지면**(끝단 궤도 하단).
        # 도면이 전부 한 평면에 놓인 자세라 바퀴부 하단도 거의 같은 높이다.
        allb = [group_bbox(g) for g in groups]
        whole = (min(b[0] for b in allb), min(b[1] for b in allb), min(b[2] for b in allb),
                 max(b[3] for b in allb), max(b[4] for b in allb), max(b[5] for b in allb))
        T = (-(whole[0] + whole[3]) / 2.0, -(whole[1] + whole[4]) / 2.0, -whole[2])
        log(f"원점 정렬: 조립체 XY 중심 + 최하 접지면을 z=0 으로 → "
            f"({T[0]:.3f}, {T[1]:.3f}, {T[2]:.3f}) m")

        # ── 3. 메시 ──
        # 두 가지 길이 있다.
        #
        #   --proxy (권장) : 솔리드 바운딩박스를 그대로 상자로 찍는다. 수 초.
        #   기본(삼각화)   : gmsh 로 실제 표면을 뽑는다. **이 도면에서는 2시간+**.
        #
        # 왜 삼각화가 이렇게 느린가 — 면이 42,678개다. 게다가 gmsh 가 못 다루는
        # 회전면("Impossible to mesh periodic surface")이 섞여 있어서 한 번에
        # generate(2) 하면 그 면 앞까지만 만들어지고 멎는다. 그래서 필요한 면만
        # 배치로 나눠 메시하고 실패한 배치는 반씩 쪼개(이분 탐색) 문제 면만 버리는데,
        # 그 대가로 generate() 호출이 수십 번 반복돼 더 느려진다.
        # 실측 1,600면/7분 → 27,381면이면 2시간을 넘는다.
        #
        # 프록시로 충분한 이유: 이 모델의 용도는 **주행 드리프트 시뮬**이다.
        # 충돌·관성은 어차피 박스·실린더로 단순화하고, visual 은 "어디에 무엇이
        # 있는지" 만 보이면 된다. 체인 링크 1,470개가 각각 작은 상자로 찍히므로
        # 궤도 모양도 그대로 산다.
        if args.proxy:
            log(f"박스 프록시 모드 — 삼각화 없이 솔리드 바운딩박스를 상자로 찍는다")
        else:
            # 형상이 원본 단위(mm)이므로 메시 크기도 원본 단위로 준다.
            gmsh.option.setNumber("Mesh.MeshSizeMax", args.mesh_size / s)
            gmsh.option.setNumber("Mesh.MeshSizeMin", args.mesh_size / s * 0.05)
            gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
            gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
            gmsh.option.setNumber("Mesh.MeshSizeExtendFromBoundary", 0)
            gmsh.option.setNumber("Mesh.MeshOnlyVisible", 1)
            gmsh.model.setVisibility(gmsh.model.getEntities(), 0)

            surf_of = {}
            for g, tags in groups.items():
                sset = set()
                for tg in tags:
                    for _, ft in gmsh.model.getBoundary([(3, tg)], oriented=False):
                        sset.add(abs(ft))
                surf_of[g] = sset
            todo = sorted(set().union(*surf_of.values()))
            log(f"메시 생성 (최대 변 {args.mesh_size} m, 면 {len(todo):,}개) — 매우 오래 걸린다")

            bad = set()

            def mesh_batch(faces: list) -> None:
                """faces 를 메시한다. 실패하면 반으로 쪼개고, 단일 면이면 버린다."""
                if not faces:
                    return
                ents = [(2, f) for f in faces]
                gmsh.model.setVisibility(ents, 1, recursive=True)
                try:
                    gmsh.model.mesh.generate(2)
                    ok = True
                except Exception:
                    ok = False
                gmsh.model.setVisibility(ents, 0, recursive=True)
                if ok:
                    return
                if len(faces) == 1:
                    bad.add(faces[0])
                    return
                mid = len(faces) // 2
                mesh_batch(faces[:mid])
                mesh_batch(faces[mid:])

            BATCH = 400
            for i in range(0, len(todo), BATCH):
                mesh_batch(todo[i:i + BATCH])
                log(f"  ...{min(i + BATCH, len(todo)):,}/{len(todo):,} 면 "
                    f"(실패 {len(bad)}개)")
            if bad:
                log(f"⚠ 끝내 메시가 안 된 면 {len(bad)}개는 버렸다 "
                    f"(gmsh 가 다루지 못하는 회전면). 형상에 작은 구멍이 남는다.")

            ntags, ncoord, _ = gmsh.model.mesh.getNodes()
            if len(ntags) == 0:
                sys.exit("메시가 하나도 생성되지 않았습니다. --mesh-size 를 키워 보세요.")
            node = {tg: (ncoord[3*i], ncoord[3*i+1], ncoord[3*i+2])
                    for i, tg in enumerate(ntags)}

        # ── 4. 링크 좌표계 원점 ──
        # 각 링크 메시를 자기 원점으로 옮겨 쓴다. URDF 조인트 원점과 짝이 맞아야 한다.
        #   wheel_unit : 로봇 원점 그대로 (base_link)
        #   deck       : 로봇 원점 — 회전축이 조립체 중심을 지난다
        #   front/rear : **끝단 회전부(턴테이블)의 중심** — 그 축으로 돌기 때문
        #   carriage   : 슬라이더 4개의 중심 = 리니어축의 실제 이동 기준점
        #   arm        : 캐리지와 같은 원점 (캐리지에 고정)
        info = {g: group_bbox(g) for g in groups}
        tf, tr_ = bbox_of(ref["turn_front"]), bbox_of(ref["turn_rear"])
        cf, cr_ = bbox_of(ref["chain_front"]), bbox_of(ref["chain_rear"])
        cm = bbox_of(ref["chain_mid"])
        sld, rail = bbox_of(ref["slider"]), bbox_of(ref["rail"])
        slider_cx = (sld[0] + sld[3]) / 2.0
        turn_f_x = (tf[0] + tf[3]) / 2.0
        turn_r_x = (tr_[0] + tr_[3]) / 2.0
        origins = {
            "wheel_unit": (0.0, 0.0, 0.0),
            "deck":       (0.0, 0.0, 0.0),
            # 윗부분·아랫부분 모두 **회전축 x** 를 링크 원점으로 쓴다.
            # 그래야 URDF 에서 회전축(윗부분)과 승강축(아랫부분)이 같은 x 에 선다.
            "front_upper": (turn_f_x, 0.0, 0.0),
            "front_lower": (turn_f_x, 0.0, 0.0),
            "rear_upper":  (turn_r_x, 0.0, 0.0),
            "rear_lower":  (turn_r_x, 0.0, 0.0),
            "carriage":   (slider_cx, 0.0, 0.0),
            "arm":        (slider_cx, 0.0, 0.0),
        }

        log("")
        log("링크별 결과 (로봇 좌표계, 접지면 z=0, 차체 XY 중심 원점):")
        # ⚠ arm 은 STL 로 쓰지 않는다. 도면 팔은 접힌 자세로 굳은 덩어리라
        #   "레일을 따라 왔다갔다 하는 팔"을 표현하지 못한다. URDF 가 공개 제원으로
        #   6축을 세운다 (crawler.urdf.xacro 의 매니퓰레이터 절).
        #   치수 요약(아래 log)에는 계속 나오므로 도면값 확인은 그대로 된다.
        for g in ("wheel_unit", "deck", "front_upper", "front_lower",
                  "rear_upper", "rear_lower", "carriage"):
            if g not in groups:
                continue
            off = origins[g]
            tris = []
            empty = 0
            if args.proxy:
                # 솔리드마다 바운딩박스 상자 하나. 링크 원점으로 옮겨서 찍는다.
                for tg in groups[g]:
                    tris += box_tris(bbox_of([tg], off))
            else:
                for sf in surf_of[g]:
                    etypes, etags, enodes = gmsh.model.mesh.getElements(2, sf)
                    n0 = len(tris)
                    for et, tags, nds in zip(etypes, etags, enodes):
                        if et != 2:
                            continue
                        for i in range(len(tags)):
                            tris.append(tuple(to_robot(node[nds[i*3+k]], off)
                                              for k in range(3)))
                    if len(tris) == n0:
                        empty += 1
            out = mesh_dir / f"{g}.stl"
            write_stl(out, tris)
            b = info[g]
            log(f"  {g:<12} 삼각형 {len(tris):>7,}  {out.stat().st_size/1e6:5.2f} MB   "
                f"X[{b[0]:6.3f},{b[3]:6.3f}] Y[{b[1]:6.3f},{b[4]:6.3f}] Z[{b[2]:6.3f},{b[5]:6.3f}]"
                f"   원점 {tuple(round(v,4) for v in origins[g])}"
                + (f"   ⚠ 메시 실패 면 {empty}" if empty else ""))

        # ── 5. URDF 에 넣을 치수 요약 ──
        wu, dk = info["wheel_unit"], info["deck"]
        fu, ru = info["front_lower"], info["rear_lower"]
        fup, rup = info["front_upper"], info["rear_upper"]
        ca, am = info["carriage"], info["arm"]
        travel = (rail[3] - rail[0]) - (sld[3] - sld[0])
        log("")
        log("params.xacro 에 넣을 값:")
        log(f"  전체        {whole[3]-whole[0]:.3f} x {whole[4]-whole[1]:.3f} "
            f"x {whole[5]-whole[2]:.3f} m")
        log(f"  바퀴부      x {wu[0]:+.3f}~{wu[3]:+.3f}  y ±{max(abs(wu[1]),wu[4]):.3f}  "
            f"z {wu[2]:.4f}~{wu[5]:.4f}")
        log(f"  끝단 전 아래 x {fu[0]:+.3f}~{fu[3]:+.3f}  z {fu[2]:.4f}~{fu[5]:.4f}  "
            f"회전축 x {turn_f_x:+.4f}")
        log(f"  끝단 전 위   x {fup[0]:+.3f}~{fup[3]:+.3f}  z {fup[2]:.4f}~{fup[5]:.4f}")
        log(f"  끝단 후 아래 x {ru[0]:+.3f}~{ru[3]:+.3f}  z {ru[2]:.4f}~{ru[5]:.4f}  "
            f"회전축 x {turn_r_x:+.4f}")
        log(f"  끝단 후 위   x {rup[0]:+.3f}~{rup[3]:+.3f}  z {rup[2]:.4f}~{rup[5]:.4f}")
        log(f"  시저 영역    z {fu[5]:.4f} ~ {fup[2]:.4f}  "
            f"(접힘 {fup[2]-fu[5]:.4f} m) — 도면 형상은 메시에서 뺐다")
        log(f"  바퀴부 체인 x {cm[0]:+.4f}~{cm[3]:+.4f}  y {cm[1]:+.4f}~{cm[4]:+.4f}  "
            f"z {cm[2]:.4f}~{cm[5]:.4f}")
        log(f"  끝단 체인   x {cf[0]:+.4f}~{cf[3]:+.4f}  y {cf[1]:+.4f}~{cf[4]:+.4f}  "
            f"z {cf[2]:.4f}~{cf[5]:.4f}")
        log(f"  상판        x {dk[0]:+.3f}~{dk[3]:+.3f}  z {dk[2]:.4f}~{dk[5]:.4f}")
        log(f"  리니어축    레일 x {rail[0]:+.4f}~{rail[3]:+.4f}  z {rail[5]:.4f}")
        log(f"              캐리지 중심 x {slider_cx:+.4f}  행정 {travel:.4f} "
            f"(중심 {(rail[0]+rail[3])/2:+.4f}, ±{travel/2:.4f})")
        log(f"  팔          x {am[0]:.4f}~{am[3]:.4f}  z {am[2]:.4f}~{am[5]:.4f}")

    finally:
        gmsh.finalize()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
