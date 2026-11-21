"""Hand torque follows teleop and pose moves; start with recording (mock_station.py, test domain only)."""
import sys, os, threading, time, json, urllib.request, urllib.error
OUT = os.environ["MOCK_DIR"]
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); sys.argv = ["server.py", "--port", "8766"]
import server as S
S.operate.dataset = "syn_test"; S.operate.relax_after_start = True   # as after starting the launch from the page
def api(path, post=True):
    req = urllib.request.Request("http://127.0.0.1:8766" + path, data=b"{}" if post else None, headers={"Content-Type": "application/json"})
    try: return json.load(urllib.request.urlopen(req, timeout=15))
    except urllib.error.HTTPError as e: return {"ERR": e.read().decode()[:200]}
log = lambda: json.load(open(OUT + "/mock_state.json"))["log"]
op = lambda: api("/api/status", False)["operate"]
def run():
    try:
        time.sleep(8); n = 0
        def new():
            nonlocal n; time.sleep(0.4); l = log(); out = l[n:]; n = len(l); return out
        print("after launch start:", new(), "| torque shown:", op()["torque"])
        print("start + recording ->", api("/api/ctl/teleop_start_rec").get("note", "ERR")); time.sleep(0.8)
        o = op(); print("  calls:", new(), "| running", o["teleop_running"], "| rec", o["rec"], "| torque", o["torque"])
        print("stop recording ->", api("/api/ctl/rec_stop").get("note", "ERR")); o = op(); print("  calls:", new(), "| rec", o["rec"], "| torque", o["torque"])
        api("/api/ctl/rec_discard")
        print("plain start ->", api("/api/ctl/teleop_start").get("note", "ERR")); time.sleep(0.6); o = op(); print("  calls:", new(), "| rec", o["rec"], "| torque", o["torque"])
        print("pause ->", api("/api/ctl/teleop_pause").get("note", "ERR")); print("  calls:", new(), "| torque", op()["torque"]); time.sleep(0.6)
        print("move to center ->", api("/api/ctl/move2center").get("note")); print("  calls:", new(), "| torque", op()["torque"])
        time.sleep(9.5); o = op(); print("  after the move: calls", new(), "| torque", o["torque"], "| message:", o["message"])
        print("near limit (mock joints at 0):", o["near_limit"])
        open(OUT + "/ready", "w").close(); time.sleep(float(os.environ.get("HOLD", "0")))
    except Exception:
        import traceback; traceback.print_exc()
    os._exit(0)
threading.Thread(target=run, daemon=True).start(); S.main()
