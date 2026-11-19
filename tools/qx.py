# usage: python qx.py <tool> '<json-args>'   (or args from file: @path)
import json, sys, urllib.request, os
URL = "http://localhost:3030/mcp"
SF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "qx.session")
def post(body, sid=None):
    h = {"Content-Type": "application/json", "Accept": "application/json, text/event-stream"}
    if sid: h["Mcp-Session-Id"] = sid
    r = urllib.request.urlopen(urllib.request.Request(URL, json.dumps(body).encode(), h), timeout=600)
    return r.headers.get("Mcp-Session-Id"), r.read().decode("utf-8")
def session(new=False):
    if not new and os.path.exists(SF): return open(SF).read().strip()
    sid, _ = post({"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-03-26","capabilities":{},"clientInfo":{"name":"qx","version":"1"}}})
    post({"jsonrpc":"2.0","method":"notifications/initialized"}, sid)
    open(SF, "w").write(sid); return sid
tool = sys.argv[1]; a = sys.argv[2] if len(sys.argv) > 2 else "{}"
args = json.load(open(a[1:], encoding="utf-8")) if a.startswith("@") else json.loads(a)
for attempt in (0, 1):
    try:
        _, txt = post({"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":tool,"arguments":args}}, session(attempt == 1)); break
    except Exception as e:
        if attempt: raise
d = json.loads(txt)
if "error" in d: print("ERROR:", d["error"]); sys.exit(1)
for c in d["result"].get("content", []):
    print(c.get("text", c))
