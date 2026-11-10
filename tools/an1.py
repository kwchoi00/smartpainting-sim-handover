#!/usr/bin/env python3
"""rec2 CSV → 시작점 기준 국소좌표로 옮겨 참값·ESKF 궤적을 나란히 찍는다.

    python3 an1.py rec.csv
"""
import csv, sys, math
rows=list(csv.DictReader(open(sys.argv[1])))
F=lambda r,k: float(r[k]) if r[k] not in ("",None) else float('nan')
g0=(F(rows[0],'gt_x'),F(rows[0],'gt_y'),F(rows[0],'gt_yaw'))
e0=(F(rows[0],'ek_x'),F(rows[0],'ek_y'),F(rows[0],'ek_yaw'))
def loc(x,y,psi,ref):
    rx,ry,rp=ref; c,s=math.cos(-math.radians(rp)),math.sin(-math.radians(rp))
    dx,dy=x-rx,y-ry
    return (c*dx-s*dy, s*dx+c*dy, (psi-rp+180)%360-180)
out=[]
for r in rows:
    gx,gy,gp=loc(F(r,'gt_x'),F(r,'gt_y'),F(r,'gt_yaw'),g0)
    ex,ey,ep=loc(F(r,'ek_x'),F(r,'ek_y'),F(r,'ek_yaw'),e0)
    out.append((F(r,'t'), (r.get('note') or "").strip(), F(r,'gt_z'), F(r,'gt_roll'),
                F(r,'gt_pitch'), gp, ep, (ep-gp+180)%360-180,
                math.hypot(ex-gx,ey-gy)*1000, F(r,'ax'), F(r,'ay'), F(r,'wz')))
print(f"{'t':>7} {'단계':<24} {'z':>7} {'roll':>7} {'pitch':>7} {'참yaw':>8} "
      f"{'ESKF':>8} {'dyaw':>7} {'err(mm)':>8} {'ax':>7} {'ay':>7}")
for o in out:
    if "▶" in o[1]:
        print(f"{o[0]:7.1f} {o[1][2:26]:<24} {o[2]:7.4f} {o[3]:7.3f} {o[4]:7.3f} "
              f"{o[5]:8.3f} {o[6]:8.3f} {o[7]:7.3f} {o[8]:8.1f} {o[9]:7.3f} {o[10]:7.3f}")
print("\n== 전 구간 극값 ==")
print("  참값 roll  %.3f ~ %.3f   pitch %.3f ~ %.3f   z %.4f ~ %.4f"%(
    min(o[3] for o in out),max(o[3] for o in out),
    min(o[4] for o in out),max(o[4] for o in out),
    min(o[2] for o in out),max(o[2] for o in out)))
print("  |dyaw| 최대 %.3f°   위치오차 최대 %.1f mm"%(
    max(abs(o[7]) for o in out), max(o[8] for o in out)))
print("  ax %.3f ~ %.3f   ay %.3f ~ %.3f"%(
    min(o[9] for o in out),max(o[9] for o in out),
    min(o[10] for o in out),max(o[10] for o in out)))
