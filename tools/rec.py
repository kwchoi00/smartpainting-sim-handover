#!/usr/bin/env python3
"""gz /joint_states 스트림을 파싱해 시계열 CSV 로 뽑는다."""
import re, sys, math

WANT = ("body_turn_joint","front_turn_joint","rear_turn_joint",
        "body_lift_joint","front_lift_joint","rear_lift_joint")

def blocks(txt):
    # 각 메시지는 최상위 'header {' 로 시작한다
    idx = [m.start() for m in re.finditer(r"(?m)^header \{", txt)]
    for a, b in zip(idx, idx[1:] + [len(txt)]):
        yield txt[a:b]

def parse(msg):
    t = re.search(r"header \{\n\s*stamp \{\n\s*sec: (\d+)\n\s*nsec: (\d+)", msg)
    if not t: return None
    ts = int(t.group(1)) + int(t.group(2))*1e-9
    pm = re.search(r"(?m)^pose \{\n  position \{\n(.*?)  \}\n  orientation \{\n(.*?)  \}", msg, re.S)
    if not pm: return None
    def num(s, k, d=0.0):
        r = re.search(rf"(?m)^\s*{k}: ([-\d.e+]+)", s); return float(r.group(1)) if r else d
    x, y, z = num(pm.group(1),"x"), num(pm.group(1),"y"), num(pm.group(1),"z")
    qx,qy,qz,qw = (num(pm.group(2),"x"), num(pm.group(2),"y"),
                   num(pm.group(2),"z"), num(pm.group(2),"w",1.0))
    yaw  = math.degrees(math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz)))
    roll = math.degrees(math.atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy)))
    pit  = math.degrees(math.asin(max(-1,min(1,2*(qw*qy-qz*qx)))))
    row = dict(t=ts,x=x,y=y,z=z,yaw=yaw,roll=roll,pitch=pit)
    for jm in re.finditer(r"(?m)^joint \{\n  name: \"([^\"]+)\"", msg):
        name = jm.group(1)
        if name not in WANT: continue
        seg = msg[jm.end():jm.end()+2000]
        p = re.search(r"axis1 \{\n(?:.*?\n)*?\s*position: ([-\d.e+]+)\n\s*velocity: ([-\d.e+]+)", seg)
        if p: row[name] = (float(p.group(1)), float(p.group(2)))
    return row

if __name__ == "__main__":
    txt = open(sys.argv[1]).read()
    rows = [r for r in (parse(b) for b in blocks(txt)) if r]
    print("msgs", len(rows))
    if not rows: sys.exit(1)
    t0 = rows[0]["t"]
    print("t,x,y,z,roll,pitch,base_yaw,body_turn,body_turn_v,deck_yaw,front_turn,front_turn_v,rear_turn")
    for r in rows:
        bt = r.get("body_turn_joint",(float('nan'),float('nan')))
        ft = r.get("front_turn_joint",(float('nan'),float('nan')))
        rt = r.get("rear_turn_joint",(float('nan'),float('nan')))
        print(f"{r['t']-t0:.3f},{r['x']:.4f},{r['y']:.4f},{r['z']:.4f},"
              f"{r['roll']:.3f},{r['pitch']:.3f},{r['yaw']:.3f},"
              f"{math.degrees(bt[0]):.3f},{math.degrees(bt[1]):.3f},"
              f"{r['yaw']+math.degrees(bt[0]):.3f},"
              f"{math.degrees(ft[0]):.3f},{math.degrees(ft[1]):.3f},{math.degrees(rt[0]):.3f}")
