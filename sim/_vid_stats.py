import json, re, os, glob
R = 'results_irregular'
# wall run time: last "done <tag> <s>s" per tag across logs
times = {}
for f in ['run.log', 'agent.log', 'v3.log', 'lift.log', 'astra3.log', 'preview.log']:
    if not os.path.exists(f):
        continue
    for line in open(f, encoding='utf-8', errors='ignore'):
        m = re.match(r'^\S+ done (.+?) (\d+)s\s*$', line.strip())
        if m:
            times[m.group(1).replace(' rerun', '')] = int(m.group(2))
def tkey(method, s):
    return {'ikrule': f'ikrule {s}', 'ikrule_lift': f'ikrule lift {s}', 'astra_v3lift': f'astra_v3lift {s}',
            'astra_v3prev': f'astra_v3prev {s}'}.get(method)
out = {}
for method, seeds in [('ikrule', range(300, 310)), ('ikrule_lift', range(300, 310)), ('opus', [300]), ('opus_v2', [300]),
                      ('astra_v3lift', range(300, 310)), ('astra_v3prev', range(300, 305))]:
    rows = []
    for s in seeds:
        p = f'{R}/agent_{method}_{s}.json'
        if not os.path.exists(p):
            continue
        d = json.load(open(p))
        tp = f'{R}/agent_{method}_{s}_trace.json'
        t = json.load(open(tp)) if os.path.exists(tp) else []
        k = tkey(method, s)
        rows.append(dict(seed=s, n=d['n_boxes'], picked=d['picked'], fell=d['boxes_fell'], handoff=d.get('n_handoff', 0),
                         lat=round(d.get('decision_latency_s') or 0, 1), decisions=len(t),
                         tok_in=sum((r.get('tok_in') or 0) for r in t), tok_cached=sum((r.get('tok_cached') or 0) for r in t),
                         tok_out=sum((r.get('tok_out') or 0) for r in t), cost=round(d.get('agent_cost_usd') or 0, 2),
                         n_preview=d.get('n_preview', 0), run_s=times.get(k) if k else None))
    out[method] = rows
json.dump(out, open('vid_stats.json', 'w'), indent=1)
for m, rows in out.items():
    tot = lambda k: sum((r[k] or 0) for r in rows)
    print(m, 'walls', len(rows), 'picked', tot('picked'), '/', tot('n'), 'fell', tot('fell'), 'ho', tot('handoff'),
          'dec', tot('decisions'), 'tok_in', tot('tok_in'), 'cached', tot('tok_cached'), 'out', tot('tok_out'), 'cost', round(tot('cost'), 2),
          'run_s', [r['run_s'] for r in rows], 'lat', [r['lat'] for r in rows])
print(sorted(times)[:60])
