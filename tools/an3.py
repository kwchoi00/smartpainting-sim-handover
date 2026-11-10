#!/usr/bin/env python3
"""rec2 CSV 의 [t0,t1] 구간 원시값 덤프 (step 줄마다 하나) — 가속도·각속도까지 본다.

    python3 an3.py rec.csv 120 180 10
"""
import csv, sys, math
rows=list(csv.DictReader(open(sys.argv[1])))
F=lambda r,k: float(r[k]) if r[k] not in ("",None) else float('nan')
t0,t1,step=float(sys.argv[2]),float(sys.argv[3]),int(sys.argv[4])
sel=[r for r in rows if t0<=F(r,'t')<=t1]
print(f"{'t':>7} {'z':>7} {'참yaw':>8} {'ESKFyaw':>8} {'ek_x':>8} {'ek_y':>8} "
      f"{'gt_x':>8} {'gt_y':>8} {'ax':>7} {'ay':>7} {'wz':>8}")
for r in sel[::step]:
    print(f"{F(r,'t'):7.1f} {F(r,'gt_z'):7.4f} {F(r,'gt_yaw'):8.3f} {F(r,'ek_yaw'):8.3f} "
          f"{F(r,'ek_x'):8.4f} {F(r,'ek_y'):8.4f} {F(r,'gt_x'):8.4f} {F(r,'gt_y'):8.4f} "
          f"{F(r,'ax'):7.3f} {F(r,'ay'):7.3f} {F(r,'wz'):8.4f}")
