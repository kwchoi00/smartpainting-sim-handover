#!/usr/bin/env python3
"""블록 STEP을 열어 데크(주행 바닥)와 주행 가능 영역을 찾는다.

선박 블록 도면은 바닥판에 맨홀·경감공이 뚫려 있는 경우가 많다.
그걸 모르고 스폰하면 로봇이 구멍으로 빠지거나 애초에 바닥이 없는 높이에 뜬다.
이 스크립트는 실제로 서 있을 수 있는 높이와, 로봇 폭이 들어가는 영역을 알려준다.

좌표는 step_to_gazebo.py 와 **같은 변환**(스케일 + 원점 정렬)을 적용해 출력하므로
그대로 `ros2 launch ... x:= y:= z:=` 에 넣으면 된다.

사용:
    python3 inspect_block.py block.stp --robot-width 0.63

필요:
    pip install gmsh
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

try:
    import gmsh
except ImportError:
    sys.exit("gmsh 가 필요합니다:  pip install gmsh")


def guess_scale(span: float) -> tuple:
    if span > 1000.0:
        return 0.001, "mm"
    if span > 100.0:
        return 0.01, "cm"
    return 1.0, "m"


def erode(grid: list, nx: int, ny: int, r_cells: int) -> list:
    """r_cells 반경만큼 침식 — 로봇 반폭이 들어가는 셀만 남긴다."""
    if r_cells <= 0:
        return [row[:] for row in grid]
    out = [[0] * nx for _ in range(ny)]
    for gy in range(ny):
        for gx in range(nx):
            if not grid[gy][gx]:
                continue
            ok = True
            for dy in range(-r_cells, r_cells + 1):
                for dx in range(-r_cells, r_cells + 1):
                    yy, xx = gy + dy, gx + dx
                    if not (0 <= xx < nx and 0 <= yy < ny) or not grid[yy][xx]:
                        ok = False
                        break
                if not ok:
                    break
            out[gy][gx] = 1 if ok else 0
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", type=Path)
    ap.add_argument("--scale", type=float, default=None,
                    help="원본→m 스케일. step_to_gazebo.py 와 같은 값을 쓸 것")
    ap.add_argument("--up-axis", choices=["x", "y", "z"], default="z",
                    help="CAD 에서 '위'인 축. step_to_gazebo.py 와 같은 값을 줄 것 — "
                         "다르면 좌표가 안 맞아 스폰 위치가 틀어진다.")
    ap.add_argument("--origin", choices=["center", "floor-center", "min", "none"],
                    default="floor-center", help="step_to_gazebo.py 와 같은 값을 쓸 것")
    ap.add_argument("--robot-width", type=float, default=0.63,
                    help="로봇 전체 폭 [m]. params.xacro 의 overall_width")
    ap.add_argument("--robot-height", type=float, default=0.39,
                    help="로봇 전체 높이 [m]. 데크 위 여유 확인용")
    ap.add_argument("--cell", type=float, default=0.10, help="래스터 격자 [m]")
    ap.add_argument("--min-deck-area", type=float, default=2.0,
                    help="데크로 볼 최소 면적 [m²]")
    ap.add_argument("--no-map", action="store_true", help="ASCII 지도 생략")
    args = ap.parse_args()

    if not args.step.is_file():
        sys.exit(f"파일을 찾을 수 없습니다: {args.step}")

    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 0)
    try:
        gmsh.model.occ.importShapes(str(args.step))
        gmsh.model.occ.synchronize()

        bb = gmsh.model.getBoundingBox(-1, -1)
        span = max(bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2])
        guessed, unit = guess_scale(span)
        scale = args.scale if args.scale is not None else guessed
        print(f"단위 {unit} 추정 → 스케일 {scale}")

        if scale != 1.0:
            gmsh.model.occ.dilate(gmsh.model.getEntities(), 0, 0, 0, scale, scale, scale)
            gmsh.model.occ.synchronize()

        # step_to_gazebo.py 와 동일한 up 축 정렬 (원점 정렬보다 먼저)
        if args.up_axis == "y":
            gmsh.model.occ.rotate(gmsh.model.getEntities(), 0, 0, 0, 1, 0, 0, math.pi / 2)
            gmsh.model.occ.synchronize()
            print("up 축 정렬: CAD +Y → Gazebo +Z")
        elif args.up_axis == "x":
            gmsh.model.occ.rotate(gmsh.model.getEntities(), 0, 0, 0, 0, 1, 0, -math.pi / 2)
            gmsh.model.occ.synchronize()
            print("up 축 정렬: CAD +X → Gazebo +Z")

        bb = gmsh.model.getBoundingBox(-1, -1)
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
            gmsh.model.occ.translate(gmsh.model.getEntities(), dx, dy, dz)
            gmsh.model.occ.synchronize()
        print(f"원점 정렬({args.origin}) 적용 → 아래 좌표는 Gazebo 모델 좌표와 동일\n")

        bb = gmsh.model.getBoundingBox(-1, -1)
        print(f"블록 전체: {bb[3]-bb[0]:.2f} x {bb[4]-bb[1]:.2f} x {bb[5]-bb[2]:.2f} m   "
              f"X[{bb[0]:.2f},{bb[3]:.2f}] Y[{bb[1]:.2f},{bb[4]:.2f}] Z[{bb[2]:.2f},{bb[5]:.2f}]")

        # ── 수평 판재 = 데크 후보 ──
        decks = []
        for dim, tag in gmsh.model.getEntities(3):
            x0, y0, z0, x1, y1, z1 = gmsh.model.getBoundingBox(dim, tag)
            d = (x1-x0, y1-y0, z1-z0)
            if d.index(min(d)) != 2:        # 두께 방향이 Z 가 아니면 수평판 아님
                continue
            if d[0] * d[1] < args.min_deck_area:
                continue
            if d[2] > 0.12:                 # 너무 두꺼우면 판재가 아님 (형강 등)
                continue
            decks.append((tag, z1, (x0, y0, x1, y1)))
        decks.sort(key=lambda r: r[1])

        if not decks:
            print("\n수평 데크를 찾지 못했습니다. --min-deck-area 를 낮춰보세요.")
            return 1

        print(f"\n데크 후보 {len(decks)}개 (상면 높이 순)")
        r_cells = max(0, int(round(args.robot_width / 2.0 / args.cell)))

        for tag, ztop, (x0, y0, x1, y1) in decks:
            print(f"\n{'='*72}")
            print(f"데크 solid#{tag}  상면 z = {ztop:.3f} m   "
                  f"X[{x0:.2f},{x1:.2f}] Y[{y0:.2f},{y1:.2f}]")
            print("=" * 72)

            top_faces = []
            for _, ftag in gmsh.model.getBoundary([(3, tag)], oriented=False):
                f = gmsh.model.getBoundingBox(2, ftag)
                if abs(f[2] - ztop) < 1e-4 and abs(f[5] - ztop) < 1e-4:
                    top_faces.append(ftag)
            if not top_faces:
                print("  상면을 찾지 못했습니다.")
                continue

            gmsh.model.mesh.clear()
            gmsh.option.setNumber("Mesh.MeshSizeMax", args.cell * 0.7)
            gmsh.option.setNumber("Mesh.MeshSizeMin", args.cell * 0.2)
            gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 0)
            gmsh.option.setNumber("Mesh.MeshSizeFromPoints", 0)
            gmsh.model.mesh.generate(2)

            nx = int((x1 - x0) / args.cell) + 1
            ny = int((y1 - y0) / args.cell) + 1
            grid = [[0] * nx for _ in range(ny)]
            area = 0.0
            for ftag in top_faces:
                etypes, etags, enodes = gmsh.model.mesh.getElements(2, ftag)
                for et, tags, nodes in zip(etypes, etags, enodes):
                    if et != 2:
                        continue
                    for i in range(len(tags)):
                        p = [gmsh.model.mesh.getNode(nodes[i*3+k])[0][:2] for k in range(3)]
                        area += abs((p[1][0]-p[0][0])*(p[2][1]-p[0][1]) -
                                    (p[2][0]-p[0][0])*(p[1][1]-p[0][1])) / 2
                        cx = sum(q[0] for q in p) / 3
                        cy = sum(q[1] for q in p) / 3
                        gx, gy = int((cx-x0)/args.cell), int((cy-y0)/args.cell)
                        if 0 <= gx < nx and 0 <= gy < ny:
                            grid[gy][gx] = 1

            full = (x1-x0) * (y1-y0)
            hole = full - area
            print(f"  판재 면적 {area:.2f} m² / 외곽 {full:.2f} m²  ({area/full:.1%})")
            if hole > 0.5:
                print(f"  ⚠ 개구부 약 {hole:.2f} m² — 로봇이 빠질 수 있는 구멍이 있습니다")

            drive = erode(grid, nx, ny, r_cells)
            n_drive = sum(sum(r) for r in drive)
            drive_area = n_drive * args.cell ** 2
            print(f"  로봇 폭 {args.robot_width} m 기준 주행 가능 면적 "
                  f"{drive_area:.2f} m²  (전체 판재의 {drive_area/max(area,1e-9):.0%})")

            if n_drive == 0:
                print(f"  ✗ 이 데크에는 폭 {args.robot_width} m 로봇이 들어갈 자리가 없습니다.")
            else:
                # 가장 넓은 연속 영역의 중심을 스폰 후보로
                best = max(((gx, gy) for gy in range(ny) for gx in range(nx) if drive[gy][gx]),
                           key=lambda g: sum(drive[yy][xx]
                                             for yy in range(max(0, g[1]-5), min(ny, g[1]+6))
                                             for xx in range(max(0, g[0]-5), min(nx, g[0]+6))))
                sx = x0 + best[0] * args.cell
                sy = y0 + best[1] * args.cell
                print(f"  ✓ 스폰 후보:  x:={sx:.2f} y:={sy:.2f} z:={ztop+0.05:.2f}")

            if not args.no_map:
                print(f"\n  █=주행가능  ▒=판재(폭 부족)  ·=구멍     "
                      f"격자 {args.cell}m,  X: {x0:.1f} → {x1:.1f} m")
                for gy in range(ny-1, -1, -1):
                    row = "".join("█" if drive[gy][gx] else ("▒" if grid[gy][gx] else "·")
                                  for gx in range(nx))
                    print(f"  y={y0+gy*args.cell:5.2f} |{row}|")

        # ── 데크 위 통과 높이 ──
        print(f"\n{'='*72}\n데크 간 높이")
        print("=" * 72)
        for i, (tag, ztop, _) in enumerate(decks):
            nxt = decks[i+1][1] if i+1 < len(decks) else bb[5]
            clear = nxt - ztop
            ok = "✓" if clear > args.robot_height else "✗ 로봇 높이 초과"
            print(f"  z={ztop:6.3f} 위 여유 {clear:5.2f} m  {ok}")

    finally:
        gmsh.finalize()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
