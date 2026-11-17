"""U1: generate seeded single-layer box walls at a container door, stacked like a real floor-loaded trailer:
boxes of mixed sizes are dropped onto a height map, so many boxes rest on two or three boxes below (interlocking),
with a few small boxes and slight lean. Frame: x across the door, y into the container (front face y=0), z up.
    python u1_walls.py --n 20 --seed0 100
"""
import argparse
import json
import os

import numpy as np

W, H = 2.30, 2.50
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "walls")


def make_wall(seed):
    r = np.random.default_rng(seed)
    res = 0.01
    n = int(W / res)
    hm = np.zeros(n)
    boxes = []
    fails = 0
    while fails < 60:
        w = float(r.choice([0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.60, 0.70]))
        h = float(r.choice([0.18, 0.22, 0.26, 0.30, 0.35, 0.40, 0.48]))
        nw = int(w / res)
        # prefer the lowest region (like a loader filling the gaps)
        starts = np.arange(0, n - nw + 1, 5)
        if len(starts) == 0:
            fails += 1
            continue
        base = np.array([hm[s:s + nw].max() for s in starts])
        cand = starts[base <= base.min() + 0.06]
        s = int(r.choice(cand))
        z0 = float(hm[s:s + nw].max())
        support = (hm[s:s + nw] >= z0 - 0.015).mean()  # fraction of the bottom that touches something
        if z0 + h > H or support < 0.55:
            fails += 1
            continue
        boxes.append(dict(x=-W / 2 + (s + nw / 2) * res, z=z0 + h / 2, w=w - 0.01, h=h, d=float(r.choice([0.30, 0.40, 0.50])),
                          lean=0.0, support=round(float(support), 2)))
        hm[s:s + nw] = z0 + h + 0.002
        fails = 0 if len(boxes) < 60 else 60
    # slight lean on a few small top boxes
    for b in boxes:
        if b["h"] <= 0.22 and b["w"] <= 0.30 and r.random() < 0.4:
            b["lean"] = float(r.choice([-5.0, 5.0]))
    for k, b in enumerate(boxes):
        b["id"] = k + 1
        b["mass"] = round(float(np.clip(b["w"] * b["h"] * b["d"] * r.uniform(60, 160), 1.5, 18.0)), 1)  # scenario
    return dict(seed=seed, W=W, H=H, boxes=boxes)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--seed0", type=int, default=100)
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    for s in range(a.seed0, a.seed0 + a.n):
        w = make_wall(s)
        with open(os.path.join(OUT, f"wall_{s}.json"), "w", encoding="utf-8") as f:
            json.dump(w, f, indent=1)
        print(s, len(w["boxes"]), "boxes; partially supported:", sum(b["support"] < 0.95 for b in w["boxes"]),
              "leaning:", sum(b["lean"] != 0 for b in w["boxes"]))
