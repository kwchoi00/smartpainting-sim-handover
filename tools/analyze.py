#!/usr/bin/env python3
"""구간별 덱 yaw 흔들림을 뽑는다. patrol 로그(note)로 구간을 나눈다."""
import csv, sys, math

rows = list(csv.DictReader(open(sys.argv[1])))
def F(r, k):
    v = r[k]
    return float(v) if v not in ("", None) else float("nan")

# note 가 찍힌 지점에서 구간을 자른다 (▶ 로 시작하는 단계 로그만)
segs, cur = [], None
for r in rows:
    n = r.get("note", "") or ""
    if "▶" in n or "═══" in n or "──" in n:
        if cur: segs.append(cur)
        cur = {"label": n.strip(), "rows": []}
    if cur: cur["rows"].append(r)
if cur: segs.append(cur)

print(f"{'구간':<52} {'Δ덱yaw':>9} {'Δ바퀴yaw':>9} {'body_q범위':>11} {'Δroll':>7} {'Δpitch':>7} {'Δz(mm)':>8} {'s':>5}")
for s in segs:
    rr = s["rows"]
    if len(rr) < 3: continue
    def rng(k):
        v = [F(r, k) for r in rr if r[k] != ""]
        v = [x for x in v if x == x]
        return (max(v) - min(v)) if v else float("nan")
    dz = rng("z") * 1000
    dt = F(rr[-1], "t") - F(rr[0], "t")
    print(f"{s['label'][:52]:<52} {rng('deck_yaw'):9.3f} {rng('base_yaw'):9.3f} "
          f"{rng('body_turn_joint'):11.3f} {rng('roll'):7.3f} {rng('pitch'):7.3f} {dz:8.2f} {dt:5.0f}")
