#!/usr/bin/env python3
"""거대한 gz topic -e 덤프를 스트리밍으로 파싱한다 (메시지 단위, N개마다 하나씩)."""
import sys, math, re
path, step = sys.argv[1], int(sys.argv[2]) if len(sys.argv)>2 else 20
WANT = {"body_turn_joint","front_turn_joint","rear_turn_joint"}
out=[]
buf=[]; n=0
def flush(msg):
    global n
    n+=1
    if n % step: return
    txt="\n".join(msg)
    t=re.search(r"stamp \{\n\s*sec: (\d+)\n\s*nsec: (\d+)",txt)
    pm=re.search(r"(?m)^pose \{\n  position \{\n(.*?)  \}\n  orientation \{\n(.*?)  \}",txt,re.S)
    if not (t and pm): return
    ts=int(t.group(1))+int(t.group(2))*1e-9
    def num(s,k,d=0.0):
        r=re.search(rf"(?m)^\s*{k}: ([-\d.e+]+)",s); return float(r.group(1)) if r else d
    x,y,z=num(pm.group(1),"x"),num(pm.group(1),"y"),num(pm.group(1),"z")
    qx,qy,qz,qw=num(pm.group(2),"x"),num(pm.group(2),"y"),num(pm.group(2),"z"),num(pm.group(2),"w",1.0)
    yaw=math.degrees(math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz)))
    roll=math.degrees(math.atan2(2*(qw*qx+qy*qz),1-2*(qx*qx+qy*qy)))
    pit=math.degrees(math.asin(max(-1,min(1,2*(qw*qy-qz*qx)))))
    j={}
    for jm in re.finditer(r"(?m)^joint \{\n  name: \"([^\"]+)\"",txt):
        nm=jm.group(1)
        if nm not in WANT: continue
        seg=txt[jm.end():jm.end()+1500]
        p=re.search(r"axis1 \{\n(?:.*?\n)*?\s*position: ([-\d.e+]+)\n\s*velocity: ([-\d.e+]+)",seg)
        if p: j[nm]=(float(p.group(1)),float(p.group(2)))
    d=lambda k,i=0: math.degrees(j.get(k,(float('nan'),float('nan')))[i])
    out.append((ts,x,y,z,roll,pit,yaw,d("body_turn_joint"),d("body_turn_joint",1),
                d("front_turn_joint"),d("front_turn_joint",1),d("rear_turn_joint")))
with open(path) as f:
    for line in f:
        if line.startswith("header {") and buf:
            flush(buf); buf=[]
        buf.append(line.rstrip("\n"))
if buf: flush(buf)
print("msgs_total",n,"sampled",len(out))
t0=out[0][0]
print("t,x,y,z,roll,pitch,base_yaw,body_turn,body_turn_v,deck_yaw,front_turn,front_turn_v,rear_turn")
for r in out:
    print(f"{r[0]-t0:.3f},{r[1]:.4f},{r[2]:.4f},{r[3]:.4f},{r[4]:.3f},{r[5]:.3f},{r[6]:.3f},"
          f"{r[7]:.3f},{r[8]:.3f},{r[6]+r[7]:.3f},{r[9]:.3f},{r[10]:.3f},{r[11]:.3f}")
