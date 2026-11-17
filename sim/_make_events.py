# show/replay: write per-decision events (frame index, decision, cumulative picked/fell) to <frames_dir>/events.json
import shutil
p = 'u1_sim_cl.py'
s = open(p, encoding='utf-8').read()
if '_EVW' in s:
    print('already'); raise SystemExit
shutil.copy(p, p + '.pre_events')
a = '            d_ = _RP[t]\n'
assert s.count(a) == 1
s = s.replace(a, a + '            _EV0 = fcount[0] // max(1, args.show_every)\n')
b = '''            if d_["item"] in ids and d_["item"] not in gone and d_["item"] not in HANDED:
                handoff(d_["item"], d_.get("reason", ""))
            continue
'''
assert s.count(b) == 1
s = s.replace(b, '''            if d_["item"] in ids and d_["item"] not in gone and d_["item"] not in HANDED:
                handoff(d_["item"], d_.get("reason", ""))
            if args.agent == "replay":
                _EVW(t, d_, _EV0)
            continue
''')
c = '''        pick_g(g_, name)
        if LOG[-1]["result"] != "picked":
            AG_FAILS[d_["item"]] += 1
'''
assert s.count(c) == 1
s = s.replace(c, c + '''        if args.agent == "replay":
            _EVW(t, d_, _EV0)
''')
d = 'def handoff(bid, reason=""):\n'
assert s.count(d) == 1
s = s.replace(d, '''_EVS = []


def _EVW(t, d_, f0):
    if not frames_dir:
        return
    _EVS.append(dict(t=t, f0=f0, f1=fcount[0] // max(1, args.show_every), action=d_.get("action"), item=d_.get("item"),
                     face=d_.get("face"), reason=(d_.get("reason") or "")[:240], result=LOG[-1].get("result") if LOG else None,
                     fell_now=LOG[-1].get("fell", []) if LOG else [],
                     picked=sum(1 for e in LOG if e.get("result") == "picked"),
                     handed=sum(1 for e in LOG if e.get("result") == "handoff"),
                     fell=len({b for e in LOG for b in e.get("fell", [])})))
    json.dump(_EVS, open(os.path.join(frames_dir, "events.json"), "w"), indent=1, default=str)


''' + d)
open(p, 'w', encoding='utf-8').write(s)
print('ok')
