"""Checks the smooth move to center/start and Stop move against mock_station.py (test domain only)."""
import sys, os, threading, time, json, math, urllib.request, urllib.error
OUT = os.environ["MOCK_DIR"]
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); sys.argv = ["server.py", "--port", "8766"]
import server as S
def api(path, post=True):
    req = urllib.request.Request("http://127.0.0.1:8766" + path, data=b"{}" if post else None, headers={"Content-Type": "application/json"})
    try: return json.load(urllib.request.urlopen(req, timeout=15))
    except urllib.error.HTTPError as e: return {"ERR": e.read().decode()[:200]}
cmd = lambda: json.load(open(OUT + "/mock_state.json"))["cmd"]
op = lambda: api("/api/status", False)["operate"]
def watch(seconds):
    pts, t0 = [], time.time()
    while time.time() - t0 < seconds:
        try: pts.append(cmd())
        except Exception: pass
        time.sleep(0.1)
    return [p for p in pts if p]
def run():
    try:
        time.sleep(6)
        o = op(); print("center:", o["center"], "| start saved:", o["start_pose"]["saved"])
        print("move2center ->", api("/api/ctl/move2center").get("note"))
        time.sleep(0.5); o = op(); print("while moving: moving=%r teleop_running=%r left=%.1f" % (o["moving"], o["teleop_running"], o["move_left"]))
        print("second move refused ->", api("/api/ctl/move2center").get("ERR"))
        print("teleop start refused ->", api("/api/ctl/teleop_start").get("ERR"))
        pts = watch(7.5); steps = [math.dist(a, b) for a, b in zip(pts, pts[1:])]
        print("first target", pts[0], "last target", pts[-1], "| largest step in 0.1 s: %.1f mm" % (max(steps) * 1000))
        o = op(); print("after: moving=%r teleop_running=%r" % (o["moving"], o["teleop_running"]))
        print("set start ->", api("/api/ctl/set_start").get("note"))
        print("move2init ->", api("/api/ctl/move2init").get("note")); time.sleep(0.6)
        print("stop ->", api("/api/ctl/move_stop").get("note")); time.sleep(0.4); a = cmd(); time.sleep(0.6); b = cmd()
        print("targets frozen after stop:", a == b, "| moving:", op()["moving"])
    except Exception:
        import traceback; traceback.print_exc()
    os._exit(0)
threading.Thread(target=run, daemon=True).start(); S.main()
