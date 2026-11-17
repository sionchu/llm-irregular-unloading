"""U1 closed-loop planners. Input: live table rows {id, kind, x, z, w, h, top, bottom, tilt_deg, displaced}.
Output: candidate ids, best first.
  rule  : same support-graph rule as u1_rule.py (highest box nothing rests on), re-run on every re-perception.
  stab  : stability-aware heuristic written by an LLM (Claude) offline from the U1 failure logs; deterministic at
          runtime (no model call). Rationale per term is in the comments; revision history in STAB_REV.
Kind (box/bag/cyl_v/cyl_h) is assumed known from the detector (stated assumption)."""
from u1_rule import plan as _rule_plan

STAB_REV = "r1"
UNSTABLE = {"cyl_v": 0.30, "cyl_h": 0.20, "bag": 0.15}  # loss log: these kinds fall/move most once neighbours go


def rule(tab):
    return _rule_plan(tab)


def _rests_on(tab, tol=0.05, ovl=0.03):
    ro = {b["id"]: set() for b in tab}
    for b in tab:
        for a in tab:
            if a is b:
                continue
            ov = min(b["x"] + b["w"] / 2, a["x"] + a["w"] / 2) - max(b["x"] - b["w"] / 2, a["x"] - a["w"] / 2)
            if ov > ovl and abs(b["bottom"] - a["top"]) < tol:
                ro[b["id"]].add(a["id"])
    return ro


def _beside(a, b, gap=0.04):
    """a and b touch side by side (x gap small, vertical spans overlap)."""
    gx = max(a["x"] - a["w"] / 2, b["x"] - b["w"] / 2) - min(a["x"] + a["w"] / 2, b["x"] + b["w"] / 2)
    vo = min(a["top"], b["top"]) - max(a["bottom"], b["bottom"])
    return gx < gap and vo > 0.05


def stab(tab):
    if not tab:
        return []
    bx = {b["id"]: b for b in tab}
    ro = _rests_on(tab)
    free = [i for i in bx if not any(i in ro[k] for k in bx if k != i)] or list(bx)
    score = {}
    for i in free:
        b = bx[i]
        s = b["top"]  # top-down remains the backbone (Pickle RL paper: height-weighted reward reduces collapse)
        s += UNSTABLE.get(b.get("kind", "box"), 0.0)  # take rollers/bags out while they are still reachable
        s += 0.02 * min(abs(b.get("tilt_deg", 0.0)), 20.0)  # a leaning item is already half-way to falling
        # do not pull the side prop of an unstable neighbour that is still standing (beside-falls in the log)
        for j, c in bx.items():
            if j == i or c.get("kind", "box") == "box" and abs(c.get("tilt_deg", 0.0)) < 8:
                continue
            if _beside(b, c) and c["top"] > b["top"] - 0.05:
                s -= 0.35
        score[i] = s
    order = sorted(free, key=lambda i: (-round(score[i], 3), bx[i]["x"]))
    rest = [i for i in _rule_plan(tab) if i not in order]
    return order + rest


PLANNERS = {"rule": rule, "stab": stab}
