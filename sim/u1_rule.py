"""U1 method A (rule-based): from the detection table, build a support graph (B rests on A if B.bottom ~ A.top and
their x-spans overlap) and repeatedly pick the highest box that nothing rests on (ties: left to right).
    python u1_rule.py 100 101 ...   -> plans/rule_<seed>.json
Optional perception noise (method A'): --noise drops 10 % of detections and adds +-3 cm to top/bottom."""
import argparse
import json
import os
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SC = os.environ.get("U1_SC", "")
SFX = f"_{SC}" if SC else ""
TOL, OVL = 0.04, 0.03


def plan(table):
    bx = {b["id"]: b for b in table}
    rest_on = {i: set() for i in bx}  # i rests on these
    for i, b in bx.items():
        for j, a in bx.items():
            if i == j:
                continue
            ov = min(b["x"] + b["w"] / 2, a["x"] + a["w"] / 2) - max(b["x"] - b["w"] / 2, a["x"] - a["w"] / 2)
            if ov > OVL and abs(b["bottom"] - a["top"]) < TOL:
                rest_on[i].add(j)
    left, seq = set(bx), []
    while left:
        free = [i for i in left if not any(i in rest_on[k] for k in left if k != i)]
        if not free:  # cycle / ambiguity: fall back to the highest box
            free = list(left)
        nxt = max(free, key=lambda i: (round(bx[i]["top"], 2), -bx[i]["x"]))
        seq.append(nxt)
        left.remove(nxt)
    return seq


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("seeds", nargs="+", type=int)
    ap.add_argument("--noise", action="store_true")
    a = ap.parse_args()
    for s in a.seeds:
        tab = json.load(open(os.path.join(HERE, "obs" + SFX, f"wall_{s}_table.json")))["boxes"]
        name = "rule"
        if a.noise:
            r = np.random.default_rng(1000 + s)
            tab = [dict(b, top=b["top"] + r.uniform(-0.03, 0.03), bottom=b["bottom"] + r.uniform(-0.03, 0.03))
                   for b in tab if r.random() > 0.10]
            name = "rule_noisy"
        t0 = time.perf_counter()
        seq = plan(tab)
        dt = time.perf_counter() - t0
        json.dump(dict(method=name, seed=s, sequence=seq, decision_latency_s=dt), open(os.path.join(HERE, "plans" + SFX, f"{name}_{s}.json"), "w"))
        print(s, name, len(seq), seq[:12], f"{dt * 1000:.2f} ms")
