"""U1 method B-local: Qwen3-VL-8B-Instruct (Q4_K_M GGUF) through llama.cpp llama-mtmd-cli, same prompt as Opus.
    python u1_qwen.py 100 101 ...   -> plans/qwen3vl8b_<seed>.json (sequence, raw answer, latency)"""
import json
import os
import re
import subprocess
import sys
import time

from u1_llm_prompt import SFX, HERE, prompt, prompt_cot, prompt_img
MODE = os.environ.get("U1_PROMPT", "img")  # img = image + IDs only (LLM does the perception); table = image + geometry table

LLAMA = r"D:\models\llama.cpp-b12202\llama-mtmd-cli.exe"
MODEL = r"D:\models\qwen3-vl-8b-instruct-gguf\Qwen3VL-8B-Instruct-Q4_K_M.gguf"
MMPROJ = r"D:\models\qwen3-vl-8b-instruct-gguf\mmproj-Qwen3VL-8B-Instruct-F16.gguf"

for s in map(int, sys.argv[1:]):
    img = os.path.join(HERE, "obs" + SFX, f"wall_{s}.png")
    t0 = time.perf_counter()
    r = subprocess.run([LLAMA, "-m", MODEL, "--mmproj", MMPROJ, "--image", img, "-p", {"img": prompt_img, "cot": prompt_cot}.get(MODE, prompt)(s), "-n", "2500", "--temp", "0",
                        "-ngl", "99", "-c", "8192"], capture_output=True, text=True, encoding="utf-8", errors="replace")
    dt = time.perf_counter() - t0
    out = r.stdout
    m = re.findall(r"\{[^{}]*\"sequence\"[^{}]*\}", out, re.S)
    seq = []
    if m:
        try:
            seq = [int(v) for v in json.loads(m[-1])["sequence"]]
        except Exception:
            seq = [int(v) for v in re.findall(r"\d+", m[-1].split("]")[0])]
    json.dump(dict(method=f"qwen3vl8b_{MODE}", seed=s, sequence=seq, decision_latency_s=dt, raw=out[-3000:], stderr_tail=r.stderr[-800:]),
              open(os.path.join(HERE, "plans" + SFX, f"qwen3vl8b_{MODE}_{s}.json"), "w"), indent=1)
    print(s, f"{dt:.1f}s", len(seq), seq[:15], flush=True)
