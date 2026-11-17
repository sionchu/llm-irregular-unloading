"""Minimal stdio MCP server for the U1 IK agent. Each tool call is forwarded to the running Isaac sim through a file
handshake (req_N.json -> resp_N.json in the io dir given as argv[1]). No third-party packages."""
import itertools
import json
import os
import sys
import time

IO = sys.argv[1]
CNT = itertools.count()
NUM = {"type": "number"}
TOOLS = [
    dict(name="get_scene", description="Current state of every item still in the container (re-perceived after each pick).",
         inputSchema={"type": "object", "properties": {}}),
    dict(name="check_grasp", description="Check one grasp: KR210 inverse kinematics on all waypoints, pad sweep collisions, pad coverage on the face, "
                                         "items resting on the target. Does not move the robot.",
         inputSchema={"type": "object", "properties": {"item": {"type": "integer"}, "face": {"type": "string", "enum": ["front", "top"]},
                                                        "du": NUM, "dv": NUM, "base_y": NUM}, "required": ["item", "face"]}),
    dict(name="execute_grasp", description="Commit this grasp as the action for this turn (re-checked; refused if it does not pass).",
         inputSchema={"type": "object", "properties": {"item": {"type": "integer"}, "face": {"type": "string", "enum": ["front", "top"]},
                                                        "du": NUM, "dv": NUM, "base_y": NUM}, "required": ["item", "face"]}),
    dict(name="preview_grasp", description="Physics preview: run this grasp in a twin of the current wall, report what would fall or move, "
                                           "then restore. Same arguments as check_grasp.",
         inputSchema={"type": "object", "properties": {"item": {"type": "integer"}, "face": {"type": "string", "enum": ["front", "top"]},
                                                        "du": NUM, "dv": NUM, "base_y": NUM}, "required": ["item", "face"]}),
    dict(name="hand_off", description="Leave this item to a person (it is removed by hand). Use when no grasp is safe.",
         inputSchema={"type": "object", "properties": {"item": {"type": "integer"}, "reason": {"type": "string"}}, "required": ["item"]}),
    dict(name="finish", description="Stop: nothing left that the robot should take.", inputSchema={"type": "object", "properties": {}}),
]


def call(tool, args):
    n = next(CNT)
    rq = os.path.join(IO, f"req_{os.getpid()}_{n:04d}.json")
    json.dump(dict(tool=tool, args=args), open(rq + ".tmp", "w"))
    os.replace(rq + ".tmp", rq)
    rp = rq.replace("req_", "resp_")
    t0 = time.time()
    while not os.path.exists(rp):
        if time.time() - t0 > 120:
            return {"ok": False, "error": "sim did not answer"}
        time.sleep(0.03)
    time.sleep(0.02)
    r = json.load(open(rp))
    os.remove(rp)
    return r


def send(o):
    sys.stdout.write(json.dumps(o) + "\n")
    sys.stdout.flush()


if "preview" not in sys.argv[2:]:
    TOOLS = [x for x in TOOLS if x["name"] != "preview_grasp"]

for line in sys.stdin:
    line = line.strip()
    if not line:
        continue
    m = json.loads(line)
    mid, meth = m.get("id"), m.get("method")
    if mid is None:
        continue
    if meth == "initialize":
        send(dict(jsonrpc="2.0", id=mid, result=dict(protocolVersion=m.get("params", {}).get("protocolVersion", "2024-11-05"),
                                                     capabilities={"tools": {}}, serverInfo={"name": "cell", "version": "1"})))
    elif meth == "tools/list":
        send(dict(jsonrpc="2.0", id=mid, result={"tools": TOOLS}))
    elif meth == "tools/call":
        p = m.get("params", {})
        r = call(p.get("name"), p.get("arguments") or {})
        send(dict(jsonrpc="2.0", id=mid, result={"content": [{"type": "text", "text": json.dumps(r)}], "isError": False}))
    elif meth == "ping":
        send(dict(jsonrpc="2.0", id=mid, result={}))
    else:
        send(dict(jsonrpc="2.0", id=mid, error={"code": -32601, "message": "method not found"}))
