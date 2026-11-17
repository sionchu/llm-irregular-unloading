"""Where do fallen/displaced items sit relative to the picked one (initial settled table)? rule results only."""
import collections, glob, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sfx = sys.argv[1] if len(sys.argv) > 1 else "_irregular"
meth = sys.argv[2] if len(sys.argv) > 2 else "rule"
C = collections.Counter(); PK = collections.Counter(); FK = collections.Counter()
for f in sorted(glob.glob(os.path.join(HERE, "results" + sfx, f"{meth}_*.json"))):
    r = json.load(open(f)); s = r["seed"]
    tab = {b["id"]: b for b in json.load(open(os.path.join(HERE, "obs" + sfx, f"wall_{s}_table.json")))["boxes"]}
    kind = {b["id"]: b.get("kind", "box") for b in json.load(open(os.path.join(HERE, "walls" + sfx, f"wall_{s}.json"), encoding="utf-8"))["boxes"]}
    for e in r["log"]:
        if not e.get("disturbed"): continue
        p = tab[e["id"]]
        PK[kind[e["id"]]] += 1
        for j in e["disturbed"]:
            q = tab[j]
            ov = min(p["x"] + p["w"] / 2, q["x"] + q["w"] / 2) - max(p["x"] - p["w"] / 2, q["x"] - q["w"] / 2)
            vo = min(p["top"], q["top"]) - max(p["bottom"], q["bottom"])
            if ov > 0.0 and q["bottom"] >= p["top"] - 0.06: rel = "above"
            elif ov > 0.0 and q["top"] <= p["bottom"] + 0.06: rel = "below"
            elif vo > 0.0 and ov > -0.05: rel = "beside"
            else: rel = "far"
            tag = "fell" if j in e["fell"] else "moved"
            C[(rel, tag)] += 1; FK[(kind[j], tag)] += 1
print("relation:", sorted(C.items()))
print("picked kind causing disturbance:", dict(PK))
print("disturbed kind:", sorted(FK.items()))
