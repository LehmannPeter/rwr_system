"""Drives APE's start pose and teleop-start checks against mock_station.py (test domain only)."""
import sys, os, threading, time, json, urllib.request, urllib.error
OUT = os.environ["MOCK_DIR"]
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); sys.argv = ["server.py", "--port", "8766"]
import server as S
def api(path, post=True):
    req = urllib.request.Request("http://127.0.0.1:8766" + path, data=b"{}" if post else None, headers={"Content-Type": "application/json"})
    try: return json.load(urllib.request.urlopen(req, timeout=15))
    except urllib.error.HTTPError as e: return {"ERR": e.read().decode()[:200]}
def run():
    try:
        time.sleep(6)
        mode = api("/api/status", False)["ros"]["robot"]["mode"]
        print("robot:", mode)
        if mode != "ready":
            print("teleop start ->", api("/api/ctl/teleop_start").get("ERR", "STARTED (wrong)")); os._exit(0)
        print("start pose at first:", api("/api/status", False)["operate"]["start_pose"])
        print("move without saved pose ->", api("/api/ctl/move2init"))
        print("set start ->", api("/api/ctl/set_start").get("note")); time.sleep(0.3)
        print("file:", open(os.environ["APE_START_POSE_FILE"]).read().replace("\n", " "))
        print("start pose now:", api("/api/status", False)["operate"]["start_pose"])
        print("move ->", api("/api/ctl/move2init").get("note")); time.sleep(0.5)
        print("arm target sent:", json.load(open(OUT + "/mock_state.json"))["cmd"])
        json.dump({"position": [0.45, -0.36, 0.2], "quat_xyzw": [1, 0, 0, 0]}, open(os.environ["APE_START_POSE_FILE"], "w"))
        print("far pose:", api("/api/status", False)["operate"]["start_pose"]["distance"], "->", api("/api/ctl/move2init"))
    except Exception:
        import traceback; traceback.print_exc()
    os._exit(0)
threading.Thread(target=run, daemon=True).start(); S.main()
