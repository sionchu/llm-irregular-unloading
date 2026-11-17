"""Aggregate U1 results -> summary.json (per method: walls, unload rate, collapse events per wall, boxes fallen,
decision latency) and a markdown table for the appendix card."""
import glob
import json
import os
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
SC = os.environ.get("U1_SC", "")
SFX = f"_{SC}" if SC else ""
LABEL = {"rule": "A 규칙 (3D 인식 좌표)", "rule_noisy": "A' 규칙 + 인식 노이즈", "opus_img": "B Claude Opus (이미지만, 대화형)",
         "qwen3vl8b_img": "B' Qwen3-VL-8B 로컬 (이미지만)", "qwen3vl8b_table": "B'' Qwen3-VL-8B 로컬 (이미지+좌표)",
         "smolvla": "C SmolVLA 미세조정 (폐루프)"}
agg = defaultdict(list)
for f in glob.glob(os.path.join(HERE, "results" + SFX, "*.json")):
    r = json.load(open(f))
    agg[r["method"]].append(r)
out = {}
for m, rs in agg.items():
    rs.sort(key=lambda r: r["seed"])
    nb = sum(r["n_boxes"] for r in rs)
    out[m] = dict(label=LABEL.get(m, m), walls=len(rs), seeds=[r["seed"] for r in rs], boxes=nb,
                  unloaded=sum(r["picked"] for r in rs), unload_rate=round(sum(r["picked"] for r in rs) / max(nb, 1), 3),
                  walls_clean=sum(1 for r in rs if r["boxes_fell"] == 0 and r["picked"] == r["n_boxes"]),
                  collapse_events_per_wall=round(sum(r["collapse_events"] for r in rs) / len(rs), 2),
                  boxes_fell=sum(r["boxes_fell"] for r in rs),
                  latency_s=[r.get("decision_latency_s") for r in rs][:1])
# fair comparison: walls that every method was run on (Opus was run on fewer walls)
common = set.intersection(*[set(s["seeds"]) for s in out.values()]) if out else set()
cm = {"seeds": sorted(common)}
for m, rs in agg.items():
    sub = [r for r in rs if r["seed"] in common]
    nb = sum(r["n_boxes"] for r in sub)
    cm[m] = round(sum(r["picked"] for r in sub) / max(nb, 1), 3)
out["_common"] = cm
print("common walls", cm)
json.dump(out, open(os.path.join(HERE, f"summary{SFX}.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
out.pop("_common")
for m, s in sorted(out.items()):
    print(f"{s['label']:<34} walls {s['walls']:>2}  unload {s['unload_rate'] * 100:5.1f}%  clean walls {s['walls_clean']:>2}  "
          f"collapse/wall {s['collapse_events_per_wall']:4.2f}  fell {s['boxes_fell']}")
