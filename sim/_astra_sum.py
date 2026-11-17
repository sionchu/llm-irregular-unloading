import json
for s in range(300, 305):
    d = json.load(open(f'results_irregular/agent_astra_v3lift_{s}.json'))
    t = json.load(open(f'results_irregular/agent_astra_v3lift_{s}_trace.json'))
    ti = sum((r.get('tok_in') or 0) for r in t)
    tc = sum((r.get('tok_cached') or 0) for r in t)
    to = sum((r.get('tok_out') or 0) for r in t)
    nod = sum(1 for r in t if not r.get('decision'))
    print(s, 'picked', d['picked'], '/', d['n_boxes'], 'fell', d['boxes_fell'], 'handoff', d['n_handoff'], 'left', len(d.get('left', [])),
          'onfloor', len(d.get('on_floor', [])), 'lat', round(d['decision_latency_s'] or 0, 1), 'calls', len(t), 'tok_in', ti, 'cached', tc, 'out', to, 'nodecision', nod)
