"""After 'Stop recording and pause' the arm returns to the chosen pose (mock_station.py, test domain only)."""
import sys, os, threading, time, json, urllib.request, urllib.error
OUT = os.environ["MOCK_DIR"]
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); sys.argv = ["server.py", "--port", "8766"]
import server as S
S.operate.dataset = "syn_test"
def api(path, body=None, post=True):
    req = urllib.request.Request("http://127.0.0.1:8766" + path, data=json.dumps(body or {}).encode() if post else None, headers={"Content-Type": "application/json"})
    try: return json.load(urllib.request.urlopen(req, timeout=15))
    except urllib.error.HTTPError as e: return {"ERR": e.read().decode()[:200]}
log = lambda: json.load(open(OUT + "/mock_state.json"))["log"]
op = lambda: api("/api/status", post=False)["operate"]
def run():
    try:
        time.sleep(8); n = len(log())
        def new():
            nonlocal n; l = log(); out = l[n:]; n = len(l); return out
        print("start pose option without a saved pose ->", api("/api/ctl/after_stop", {"value": "start"}).get("ERR", "accepted"))
        for mode in ("off", "center"):
            print(f"--- after stop = {mode}:", api("/api/ctl/after_stop", {"value": mode}).get("operate", {}).get("after_stop"))
            api("/api/ctl/teleop_start_rec"); time.sleep(1.0); new()
            api("/api/ctl/rec_stop"); time.sleep(1.6); o = op()
            print("   1.6 s after stop: calls", new(), "| moving:", repr(o["moving"]), "| torque", o["torque"], "| rec", o["rec"])
            t0 = time.time()
            while time.time() - t0 < 16 and op()["moving"]: time.sleep(0.3)
            time.sleep(4); o = op(); print("   at the end: calls", new(), "| torque", o["torque"], "| distance to center %.0f mm" % (o["center"]["distance"] * 1000), "| message:", o["message"][:80])
            api("/api/ctl/rec_discard")
        print("setting kept in file:", open(os.environ["APE_SETTINGS_FILE"]).read().replace("\n", " "))
    except Exception:
        import traceback; traceback.print_exc()
    os._exit(0)
threading.Thread(target=run, daemon=True).start(); S.main()
