"""Loss breakdown of U1 results: each item ends in exactly one fate.
picked | fell (knocked down by another pick, never picked) | displaced (pushed out of wall, no fall flag)
| no_grip (last attempt failed to seal) | grip_lost (dropped in transit) | not_attempted.
    python u1_loss_breakdown.py  -> loss_breakdown.json + printed table"""
import collections
import glob
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = {}
for sfx, sc in (("", "std"), ("_mixed", "mixed"), ("_irregular", "irregular")):
    walls = {}
    for f in glob.glob(os.path.join(HERE, "walls" + sfx, "wall_*.json")):
        w = json.load(open(f, encoding="utf-8"))
        walls[w["seed"]] = {b["id"]: b.get("kind", "box") for b in w["boxes"]}
    for f in sorted(glob.glob(os.path.join(HERE, "results" + sfx, "*.json"))):
        r = json.load(open(f, encoding="utf-8"))
        m = re.match(r"(.+)_(\d+)\.json$", os.path.basename(f))
        method, seed = m.group(1), int(m.group(2))
        kinds = walls.get(seed, {})
        fate, fell_any, last = {}, set(), {}
        for e in r["log"]:
            fell_any.update(e.get("fell", []))
            if e.get("id") is not None:
                last[e["id"]] = e["result"]
        gone = set(kinds) - set(r["left"])
        for i in kinds:
            if i in gone:
                fate[i] = "picked"
            elif i in fell_any:
                fate[i] = "fell"
            elif last.get(i) == "skip_displaced":
                fate[i] = "displaced"
            elif last.get(i) in ("no_grip", "grip_lost"):
                fate[i] = last[i]
            elif i in r.get("on_floor", []):
                fate[i] = "fell"
            else:
                fate[i] = "not_attempted"
        key = f"{sc}/{method}"
        agg = OUT.setdefault(key, {"walls": [], "by_kind": {}})
        agg["walls"].append(seed)
        for i, k in kinds.items():
            agg["by_kind"].setdefault(k, collections.Counter())[fate[i]] += 1
FATES = ["picked", "fell", "displaced", "no_grip", "grip_lost", "not_attempted"]
for key, agg in OUT.items():
    tot = collections.Counter()
    for c in agg["by_kind"].values():
        tot.update(c)
    agg["total"] = tot
    n = sum(tot.values())
    print(f"{key:32s} walls={len(agg['walls']):2d} n={n:4d} " + " ".join(f"{f}={100 * tot[f] / n:5.1f}" for f in FATES))
    for k, c in sorted(agg["by_kind"].items()):
        nk = sum(c.values())
        print(f"    {k:6s} n={nk:4d} " + " ".join(f"{f}={100 * c[f] / nk:5.1f}" for f in FATES))
json.dump({k: {"walls": v["walls"], "total": dict(v["total"]), "by_kind": {a: dict(b) for a, b in v["by_kind"].items()}}
           for k, v in OUT.items()}, open(os.path.join(HERE, "loss_breakdown.json"), "w"), indent=1)
