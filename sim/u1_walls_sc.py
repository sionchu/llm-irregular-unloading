"""U1 scenarios beyond the regular box wall (std = u1_walls.py, unchanged).
  mixed     : random mixed sizes (small to large), weaker support allowed, more lean, wider mass range
  irregular : mixed boxes + non-box items: polybags (flat, light, low friction), upright tubes/cans (cyl_v),
              lying tubes (cyl_h, nothing stacked on them). Polybag softness is NOT simulated (rigid approximation);
              its grip failures come from the U2 simple grip model (u1_sim.py, env U1_GRIP).
    python u1_walls_sc.py --sc mixed --seed0 200 --n 10   -> walls_mixed/wall_<seed>.json
"""
import argparse
import json
import os

import numpy as np

W, H = 2.30, 2.50
HERE = os.path.dirname(os.path.abspath(__file__))


def pick_item(r, sc):
    if sc == "irregular":
        kind = str(r.choice(["box", "bag", "cyl_v", "cyl_h"], p=[0.62, 0.22, 0.10, 0.06]))
    else:
        kind = "box"
    if kind == "bag":
        return kind, float(r.choice([0.35, 0.40, 0.45, 0.55])), float(r.choice([0.10, 0.12, 0.15, 0.18])), float(r.choice([0.30, 0.35, 0.45])), 0.70
    if kind == "cyl_v":
        dm = float(r.choice([0.20, 0.25, 0.30]))
        return kind, dm, float(r.choice([0.30, 0.35, 0.45])), dm, 0.75
    if kind == "cyl_h":
        dm = float(r.choice([0.15, 0.20, 0.25, 0.30]))
        return kind, dm, dm, float(r.choice([0.40, 0.50, 0.60])), 0.90
    w = float(r.choice([0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50, 0.60, 0.70]))
    h = float(r.choice([0.10, 0.14, 0.18, 0.22, 0.26, 0.30, 0.35, 0.40, 0.48, 0.55]))
    return kind, w, h, float(r.choice([0.25, 0.30, 0.40, 0.50, 0.60])), 0.45


def make_wall(seed, sc):
    r = np.random.default_rng(seed)
    res = 0.01
    n = int(W / res)
    hm = np.zeros(n)
    items = []
    fails = 0
    while fails < 60:
        kind, w, h, d, need = pick_item(r, sc)
        nw = int(w / res)
        starts = np.arange(0, n - nw + 1, 5)
        base = np.array([hm[s:s + nw].max() for s in starts])
        cand = starts[base <= base.min() + 0.08]
        s = int(r.choice(cand))
        z0 = float(hm[s:s + nw].max())
        support = (hm[s:s + nw] >= z0 - 0.015).mean()
        if z0 + h > H or support < need:
            fails += 1
            continue
        items.append(dict(kind=kind, x=-W / 2 + (s + nw / 2) * res, z=z0 + h / 2, w=w - 0.01, h=h, d=d, lean=0.0,
                          support=round(float(support), 2)))
        # nothing is stacked on a lying tube (it would roll): block that span
        hm[s:s + nw] = H + 1 if kind == "cyl_h" else z0 + h + 0.002
        fails = 0 if len(items) < 60 else 60
    for b in items:
        if b["kind"] == "box" and b["h"] <= 0.22 and b["w"] <= 0.30 and r.random() < 0.6:
            b["lean"] = float(r.choice([-8.0, -5.0, 5.0, 8.0]))
    for k, b in enumerate(items):
        b["id"] = k + 1
        vol = b["w"] * b["h"] * b["d"]
        if b["kind"] == "bag":
            m = np.clip(vol * r.uniform(40, 120), 0.3, 3.0)
        elif b["kind"].startswith("cyl"):
            m = np.clip(vol * 0.785 * r.uniform(80, 250), 0.5, 15.0)
        else:
            m = np.clip(vol * r.uniform(30, 220), 0.5, 25.0)
        b["mass"] = round(float(m), 1)  # scenario assumption
    return dict(seed=seed, sc=sc, W=W, H=H, boxes=items)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--sc", required=True, choices=["mixed", "irregular"])
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--seed0", type=int, default=200)
    a = ap.parse_args()
    out = os.path.join(HERE, f"walls_{a.sc}")
    os.makedirs(out, exist_ok=True)
    for s in range(a.seed0, a.seed0 + a.n):
        w = make_wall(s, a.sc)
        with open(os.path.join(out, f"wall_{s}.json"), "w", encoding="utf-8") as f:
            json.dump(w, f, indent=1)
        kinds = {}
        for b in w["boxes"]:
            kinds[b["kind"]] = kinds.get(b["kind"], 0) + 1
        print(s, len(w["boxes"]), kinds, "partial:", sum(b["support"] < 0.95 for b in w["boxes"]),
              "lean:", sum(b["lean"] != 0 for b in w["boxes"]))
