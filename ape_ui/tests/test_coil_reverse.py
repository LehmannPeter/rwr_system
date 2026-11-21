"""The 'Reverse arm directions' switch writes the teleop node's coil station file (mock_station.py, test domain only)."""
import sys, os, threading, time, json, urllib.request, urllib.error
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); sys.argv = ["server.py", "--port", "8766"]
import server as S
FILE = os.environ["APE_COIL_ADJUST_FILE"]
def api(path, body=None, post=True):
    req = urllib.request.Request("http://127.0.0.1:8766" + path, data=json.dumps(body or {}).encode() if post else None, headers={"Content-Type": "application/json"})
    try: return json.load(urllib.request.urlopen(req, timeout=15))
    except urllib.error.HTTPError as e: return {"ERR": e.read().decode()[:200]}
op = lambda: api("/api/status", post=False)["operate"]
saved = lambda: open(FILE).read().replace("\n", " ") if os.path.exists(FILE) else "no file"
def run():
    try:
        time.sleep(8)
        print("at first: yaw", op()["coil_yaw_deg"], "| file:", saved())
        r = api("/api/ctl/coil_reverse", {"on": True}); print("on  -> yaw", r.get("operate", {}).get("coil_yaw_deg"), "| file:", saved(), "| note:", r.get("note"))
        r = api("/api/ctl/coil_reverse", {"on": False}); print("off -> yaw", r.get("operate", {}).get("coil_yaw_deg"), "| file:", saved(), "| note:", r.get("note"))
        Monitor, running = type(S.monitor), type(S.monitor).teleop_running
        Monitor.teleop_running = property(lambda self: True)   # the mock station sends no arm commands, so pretend teleop runs
        print("on while teleop runs ->", api("/api/ctl/coil_reverse", {"on": True}).get("ERR", "ACCEPTED (wrong)"), "| file:", saved())
        Monitor.teleop_running = running
        S.operate.rec = "review"
        print("on with an unsaved demo ->", api("/api/ctl/coil_reverse", {"on": True}).get("ERR", "ACCEPTED (wrong)"), "| file:", saved())
        S.operate.rec = "idle"
        print("on while paused ->", api("/api/ctl/coil_reverse", {"on": True}).get("ERR", "accepted"), "| yaw", op()["coil_yaw_deg"])
        for content in ('{"yaw_deg": 150}', '{"yaw_deg": 999}', 'not json'):
            open(FILE, "w").write(content); print(f"file {content!r} -> yaw", op()["coil_yaw_deg"])
    except Exception:
        import traceback; traceback.print_exc()
    os._exit(0)
threading.Thread(target=run, daemon=True).start(); S.main()
