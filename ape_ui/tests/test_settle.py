"""Move to center with different turn values against a fake arm that stops 7 deg short (test domain only)."""
import sys, os, threading, time, json, math, urllib.request, urllib.error
import numpy as np
from scipy.spatial.transform import Rotation as R
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); sys.argv = ["server.py", "--port", "8766"]
import server as S
def post(path, body=None):
    try: return json.load(urllib.request.urlopen(urllib.request.Request("http://127.0.0.1:8766" + path, data=json.dumps(body or {}).encode(), headers={"Content-Type": "application/json"}), timeout=15))
    except urllib.error.HTTPError as e: return {"ERR": e.read().decode()[:200]}
def yaw():
    x = R.from_quat(S.monitor.tele["arm"][3:]).as_matrix(); return math.degrees(math.atan2(x[1, 0], x[0, 0])), math.degrees(math.acos(max(-1, min(1, -x[2, 2]))))
def run():
    try:
        time.sleep(7)
        for turn in (0, -20, -5, 10, 30, 0, 45, -5):
            post("/api/ctl/center_yaw", {"value": turn}); r = post("/api/ctl/move2center"); t0 = time.time()
            while time.time() - t0 < 14:
                time.sleep(0.2)
                if not json.load(urllib.request.urlopen("http://127.0.0.1:8766/api/status"))["operate"]["moving"]: break
            time.sleep(0.5); y, tilt = yaw()
            print(f"turn {turn:4d}: hand yaw {y:6.1f} (error {y - turn:5.1f}), tilt from straight down {tilt:4.1f} deg, took {time.time() - t0:4.1f} s | {r.get('note', r)}", flush=True)
            time.sleep(3.6); m = json.load(urllib.request.urlopen("http://127.0.0.1:8766/api/status"))["operate"]["message"]
            if m: print("   message:", m[:110])
    except Exception:
        import traceback; traceback.print_exc()
    os._exit(0)
threading.Thread(target=run, daemon=True).start(); S.main()
