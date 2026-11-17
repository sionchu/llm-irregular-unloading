"""Common-wall comparison (irregular 300-304, mixed 200-204): picked share, collapse events, attempts, time.
-> cl_compare.json + printed table"""
import json, os
H = os.path.dirname(os.path.abspath(__file__))
M = ["rule", "rule_noisy", "rule_quad", "qwen3vl8b_img", "smolvla", "cl_rule", "cl_stab", "cl_stab_look3"]
out = {}
for sc, seeds in (("irregular", range(300, 305)), ("mixed", range(200, 205))):
    for m in M:
        rs = []
        for s in seeds:
            p = os.path.join(H, f"results_{sc}", f"{m}_{s}.json")
            if os.path.exists(p):
                rs.append(json.load(open(p)))
        if len(rs) < 5:
            continue
        n = sum(r["n_boxes"] for r in rs); k = sum(r["picked"] for r in rs)
        att = sum(sum(1 for e in r["log"] if e.get("result") not in ("skip_displaced", "skip_invalid")) for r in rs)
        out[f"{sc}/{m}"] = dict(picked=k, n=n, pct=round(100 * k / n, 1), per_wall=[r["picked"] for r in rs],
                                collapse_events=sum(r["collapse_events"] for r in rs), attempts=att,
                                trials=sum(r.get("trials") or 0 for r in rs))
        print(f"{sc:9s} {m:15s} {k:3d}/{n} = {100*k/n:5.1f}%  per wall {[r['picked'] for r in rs]}  collapses {out[f'{sc}/{m}']['collapse_events']}  attempts {att}")
json.dump(out, open(os.path.join(H, "cl_compare.json"), "w"), indent=1)
