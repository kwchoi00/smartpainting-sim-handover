#!/usr/bin/env python3
"""rec2 CSV 의 [t0,t1] 구간을 표로 — z, 참yaw, ESKFyaw, 오차, 오도메트리까지.

    python3 an2.py rec.csv 120 180
"""
import csv, sys, math
rows=list(csv.DictReader(open(sys.argv[1])))
F=lambda r,k: float(r[k]) if r[k] not in ("",None) else float('nan')
t0,t1=float(sys.argv[2]),float(sys.argv[3])
g0=(F(rows[0],'gt_x'),F(rows[0],'gt_y'),F(rows[0],'gt_yaw'))
e0=(F(rows[0],'ek_x'),F(rows[0],'ek_y'),F(rows[0],'ek_yaw'))
o0=(F(rows[0],'od_x'),F(rows[0],'od_y'),F(rows[0],'od_yaw'))
def loc(x,y,psi,ref):
    rx,ry,rp=ref; c,s=math.cos(-math.radians(rp)),math.sin(-math.radians(rp))
    dx,dy=x-rx,y-ry
    return (c*dx-s*dy, s*dx+c*dy, (psi-rp+180)%360-180)
print(f"{'t':>7} {'z':>7} {'참yaw':>8} {'ESKFyaw':>8} {'dyaw':>7} {'err':>7} "
      f"{'ax':>7} {'ay':>7} {'wz':>7} {'odom이동mm':>10} {'단계'}")
for r in rows:
    t=F(r,'t')
    if not (t0<=t<=t1): continue
    gx,gy,gp=loc(F(r,'gt_x'),F(r,'gt_y'),F(r,'gt_yaw'),g0)
    ex,ey,ep=loc(F(r,'ek_x'),F(r,'ek_y'),F(r,'ek_yaw'),e0)
    ox,oy,op=loc(F(r,'od_x'),F(r,'od_y'),F(r,'od_yaw'),o0)
    n=(r.get('note') or "").strip()
    if abs(t*20 - round(t*20)) < 1e-6 and round(t*4)%4==0 or "▶" in n:
        print(f"{t:7.1f} {F(r,'gt_z'):7.4f} {gp:8.3f} {ep:8.3f} {(ep-gp+180)%360-180:7.3f} "
              f"{math.hypot(ex-gx,ey-gy)*1000:7.1f} {F(r,'ax'):7.3f} {F(r,'ay'):7.3f} "
              f"{F(r,'wz'):7.4f} {math.hypot(ox,oy)*1000:10.1f} {n[:34]}")
