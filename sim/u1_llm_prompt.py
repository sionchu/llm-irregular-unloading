"""Shared prompt for the LLM methods (B): the same image + detection table is given to Claude Opus (in-session,
judged by reading the image) and to a local VLM (Qwen3-VL-8B via llama.cpp)."""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SC = os.environ.get("U1_SC", "")
SFX = f"_{SC}" if SC else ""

PROMPT = """You plan the unloading order for a robot that unloads a truck container.
The robot has a flat suction pad and pulls each box straight OUT of the wall toward the camera (horizontal pull).
The image shows the box wall seen from the container door; every box carries its ID number.
Detected boxes (metres; x = left/right centre, top/bottom = heights of the box top and bottom faces):
{table}
Physics: if a box is pulled out while another box rests on top of it, the box above loses support and falls.
Boxes often rest on two or three boxes below. Give the order that removes ALL boxes without making any fall.
Answer with JSON only: {{"sequence": [ids in pick order], "reason": "one short sentence"}}"""


PROMPT_IMG = """You plan the unloading order for a robot that unloads a truck container.
The robot has a flat suction pad and pulls each box straight OUT of the wall toward the camera (horizontal pull).
The image shows the box wall seen from the container door; every box carries its ID number (IDs 1 to {n}).
Physics: if a box is pulled out while another box rests on top of it, the box above loses support and falls.
Boxes often rest on two or three boxes below. Look at the image and give the order that removes ALL boxes
without making any box fall. Answer with JSON only: {{"sequence": [ids in pick order], "reason": "one short sentence"}}"""


PROMPT_COT = """You plan the unloading order for a robot that unloads a truck container.
The robot pulls each box straight OUT of the wall toward the camera with a suction pad.
The image shows the box wall; every box carries its ID number (IDs 1 to {n}).
If a box is pulled out while another box rests on top of it, the box above falls. Boxes often rest on two or three boxes.
Step 1: for every box ID, write which box IDs are directly on top of it (resting on it), one line per box.
Step 2: using that list, write the order that removes ALL boxes so that a box is only removed after every box on top of it.
End with JSON only on the last line: {{"sequence": [ids in pick order]}}"""


def prompt_cot(seed):
    n = len(json.load(open(os.path.join(HERE, "obs" + SFX, f"wall_{seed}_table.json")))["boxes"])
    return PROMPT_COT.format(n=n)


def prompt_img(seed):
    n = len(json.load(open(os.path.join(HERE, "obs" + SFX, f"wall_{seed}_table.json")))["boxes"])
    return PROMPT_IMG.format(n=n)


def table_text(seed, noisy=False):
    tab = json.load(open(os.path.join(HERE, "obs" + SFX, f"wall_{seed}_table.json")))["boxes"]
    rows = [f"id {b['id']}: x={b['x']:+.2f} width={b['w']:.2f} bottom={b['bottom']:.2f} top={b['top']:.2f}" for b in tab]
    return "\n".join(rows)


def prompt(seed):
    return PROMPT.format(table=table_text(seed))


if __name__ == "__main__":
    import sys

    print(prompt(int(sys.argv[1])))
