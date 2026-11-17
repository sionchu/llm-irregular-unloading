import json, os, collections
u = r"."
for d in ["vid_rule300", "show_fail_opus300", "show_catch_opus300", "vid_astralift300", "vid_astra300",
          "grid_rule300", "grid_rulelift300", "grid_astralift300", "grid_astraprev300"]:
    f = os.path.join(u, d, "armdiag.json")
    if not os.path.exists(f):
        print(d, "no diag"); continue
    r = json.load(open(f))
    bad = [x for x in r if x[2] > 10]
    print(d, len(r), dict(collections.Counter(x[1] for x in r)), "err>10mm", len(bad), "conv", sum(x[3] for x in r), bad[:6])
