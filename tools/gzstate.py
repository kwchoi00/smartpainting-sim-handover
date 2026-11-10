#!/usr/bin/env python3
"""gz /joint_states (gz.msgs.Model) 를 텍스트로 받아 관절값과 모델 포즈를 뽑는다."""
import re, subprocess, sys, math

def snapshot(n=1, timeout=10):
    out = subprocess.run(["gz","topic","-e","-t","/joint_states","-n",str(n)],
                         capture_output=True, text=True, timeout=timeout).stdout
    return out

def parse(txt):
    # 모델 포즈
    m = re.search(r"name: \"crawler\"\s*\nid: \d+\npose \{\n\s*position \{\n(.*?)\}\n\s*orientation \{\n(.*?)\}", txt, re.S)
    pose = {}
    if m:
        for k in "xyz":
            r = re.search(rf"{k}: ([-\d.e+]+)", m.group(1))
            pose[k] = float(r.group(1)) if r else 0.0
        q = {}
        for k in "xyzw":
            r = re.search(rf"{k}: ([-\d.e+]+)", m.group(2))
            q[k] = float(r.group(1)) if r else (1.0 if k=="w" else 0.0)
        pose["yaw"] = math.degrees(math.atan2(2*(q["w"]*q["z"]+q["x"]*q["y"]),
                                              1-2*(q["y"]**2+q["z"]**2)))
        pose["roll"] = math.degrees(math.atan2(2*(q["w"]*q["x"]+q["y"]*q["z"]),
                                               1-2*(q["x"]**2+q["y"]**2)))
        pose["pitch"] = math.degrees(math.asin(max(-1,min(1,2*(q["w"]*q["y"]-q["z"]*q["x"])))))
    # 관절: name 뒤 axis1 의 position
    joints = {}
    for jm in re.finditer(r"joint \{\n\s*name: \"([^\"]+)\"(.*?)\n\}\n", txt, re.S):
        name, body = jm.group(1), jm.group(2)
        p = re.search(r"axis1 \{\n\s*position: ([-\d.e+]+)", body)
        v = re.search(r"axis1 \{[^}]*?velocity: ([-\d.e+]+)", body, re.S)
        if p: joints[name] = (float(p.group(1)), float(v.group(1)) if v else 0.0)
    return pose, joints

if __name__ == "__main__":
    pose, j = parse(snapshot())
    print("pose", {k: round(v,4) for k,v in pose.items()})
    for k in sorted(j):
        if any(s in k for s in ("wheel","roller","scissor")): continue
        print(f"  {k:24s} q={j[k][0]:+.4f} v={j[k][1]:+.4f}")
