# Build remotion_vid/src/data.json (stats + trace excerpts + grid events) and copy clip mp4s to remotion_vid/public
import json, os, re, shutil, subprocess
R = 'results_irregular'
OUT = 'remotion_vid'
subprocess.run(['python', '_vid_stats.py'], check=True, capture_output=True)
st = json.load(open('vid_stats.json'))


def agg(method, seeds):
    rows = [r for r in st[method] if r['seed'] in seeds]
    s = lambda k: sum((r[k] or 0) for r in rows)
    run = [r['run_s'] for r in rows if r['run_s']]
    lat = [r['lat'] for r in rows if r['lat']]
    return dict(walls=len(rows), n=s('n'), picked=s('picked'), fell=s('fell'), handoff=s('handoff'), decisions=s('decisions'),
                tok_in=s('tok_in'), tok_cached=s('tok_cached'), tok_out=s('tok_out'), cost=round(s('cost'), 2),
                run_min=round(min(run) / 60, 1) if run else None, run_max=round(max(run) / 60, 1) if run else None,
                run_sum_h=round(sum(run) / 3600, 2) if run else None,
                lat_min=min(lat) if lat else None, lat_max=max(lat) if lat else None)


W5, W10 = range(300, 305), range(300, 310)
stats = dict(
    w5=dict(rule=agg('ikrule', W5), rulelift=agg('ikrule_lift', W5), llm=agg('astra_v3lift', W5), prev=agg('astra_v3prev', W5)),
    w10=dict(rule=agg('ikrule', W10), rulelift=agg('ikrule_lift', W10), llm=agg('astra_v3lift', W10)),
    opus1=agg('opus', [300]), opus2=agg('opus_v2', [300]),
    rule_run_s=[r['run_s'] for r in st['ikrule'] if r['run_s']],
)


def calls(method, ts):
    t = json.load(open(f'{R}/agent_{method}_300_trace.json'))
    out = []
    for r in t:
        if r.get('t') in ts:
            cs = [dict(tool=c['tool'], item=(c.get('args') or {}).get('item'), face=(c.get('args') or {}).get('face'), ok=c.get('ok'))
                  for c in (r.get('calls') or [])]
            d = r.get('decision') or {}
            why = d.get('reason') or r.get('result') or ''
            why = re.sub(r'[*`#]', '', why).replace('−', '-')
            why = re.sub(r'\s+', ' ', why).strip()
            out.append(dict(t=r['t'], lat=r.get('latency_s'), calls=cs, decision=r.get('decision'), why=why[:600],
                            tok_in=r.get('tok_in'), tok_cached=r.get('tok_cached'), tok_out=r.get('tok_out'), cost=r.get('cost_usd')))
    return out


trace = dict(rule=calls('ikrule', [2]), opus1=calls('opus', [8]), llm=calls('astra_v3lift', [12, 13]),
             prev=calls('astra_v3prev', list(range(9, 17))), prevAll=calls('astra_v3prev', list(range(0, 200))),
             llmAll=calls('astra_v3lift', list(range(0, 200))))
grid = {}
for d in ['grid_rule300', 'grid_rulelift300', 'grid_astralift300', 'grid_astraprev300']:
    p = os.path.join(d, 'events.json')
    if os.path.exists(p):
        pngs = sorted(f for f in os.listdir(d) if f.endswith('.png'))
        grid[d] = dict(start=int(pngs[0][:5]), frames=len(pngs), events=json.load(open(p)))
clips = {}
os.makedirs(f'{OUT}/public/clips', exist_ok=True)
for d in ['vid_rule300', 'vid_astra300', 'vid_astralift300', 'show_fail_opus300', 'show_catch_opus300', 'grid_rule300',
          'grid_rulelift300', 'grid_astralift300', 'grid_astraprev300']:
    if os.path.exists(d + '.mp4'):
        shutil.copy(d + '.mp4', f'{OUT}/public/clips/{d}.mp4')
        pngs = [f for f in os.listdir(d) if f.endswith('.png')]
        ev = os.path.join(d, 'events.json')
        ph = os.path.join(d, 'phases.json')
        clips[d] = dict(frames=len(pngs), start=int(min(pngs)[:5]) if pngs else 0,
                        events=json.load(open(ev)) if os.path.exists(ev) else [],
                        phases=json.load(open(ph)) if os.path.exists(ph) else [])
json.dump(dict(stats=stats, trace=trace, clips=clips), open(f'{OUT}/src/data.json', 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
print(json.dumps(stats, indent=1))
print({k: (v['frames'], len(v['events'])) for k, v in clips.items()})
