"""Live data, cameras, processing job and demo player against mock_station.py (test domain only).
Expects the synthetic recording in $RWR_WS/data/raw/syn_test/ (before 2026-11-21: $RWR_WS/datasets/syn_test/)."""
import sys, os, threading, time, json, urllib.request, urllib.error, subprocess
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); sys.argv = ["server.py", "--port", "8766"]
import server as S
B = "http://127.0.0.1:8766"
def get(path, raw=False):
    try:
        r = urllib.request.urlopen(B + path, timeout=20); d = r.read(); return d if raw else json.loads(d)
    except urllib.error.HTTPError as e: return {"ERR": e.code, "detail": e.read().decode()[:160]}
def post(path):
    try: return json.load(urllib.request.urlopen(urllib.request.Request(B + path, data=b"{}", headers={"Content-Type": "application/json"}), timeout=20))
    except urllib.error.HTTPError as e: return {"ERR": e.code, "detail": e.read().decode()[:200]}
def run():
    try:
        time.sleep(7)
        live = S.monitor.live()
        print("live keys with data:", [k for k, v in live.items() if v])
        print("arm", live["arm"][:3], "| hand temp max", max(live["hand_temp"]), "| current[0]", live["hand_current"][0], "| franka q len", len(live["franka"]["q"]))
        st = get("/api/status"); print("rates:", {k: v["hz"] for k, v in st["rates"].items()})
        print("cameras publishing:", st["ros"]["checks"]["cameras"])
        j = get("/api/cam/oakd_front_view.jpg", raw=True); print("camera jpeg:", len(j), "bytes", j[:3] == b"\xff\xd8\xff")
        print("bad camera ->", get("/api/cam/nope.jpg").get("ERR"))
        cpu = subprocess.run(["ps", "-o", "%cpu=", "-p", str(os.getpid())], capture_output=True, text=True).stdout.strip(); print("server cpu % (average since start):", cpu)
        ds = get("/api/datasets")["datasets"]; print("datasets:", [(d["name"], len(d["demos"]), d["todo"]) for d in ds])
        print("process ->", post("/api/datasets/syn_test/process").get("running"))
        print("second job refused ->", post("/api/datasets/syn_test/process").get("ERR"))
        for _ in range(120):
            time.sleep(0.5)
            if not get("/api/status")["procs"]["job"]["running"]: break
        print("job exit:", get("/api/status")["procs"]["job"]["exit_code"])
        d = get("/api/datasets")["datasets"][0]["demos"][0]; print("after job:", d)
        m = get("/api/demo/syn_test/20270101_120000/meta"); print("meta:", m["frames"], m["freq"], m["cameras"])
        f = get("/api/demo/syn_test/20270101_120000/frame/5.jpg", raw=True); print("frame jpeg:", len(f), f[:3] == b"\xff\xd8\xff")
        print("plot:", len(get("/api/demo/syn_test/20270101_120000/trajectory.png", raw=True)), "bytes")
        print("path trick ->", get("/api/demo/..%2f..%2fetc/passwd/meta").get("ERR"))
        print("recalibrate while hand runs ->", post("/api/hand/recalibrate").get("detail"))
        print("process again (nothing to do) ->", post("/api/datasets/syn_test/process").get("running")); time.sleep(6)
        print("job exit:", get("/api/status")["procs"]["job"]["exit_code"])
        open(os.environ["MOCK_DIR"] + "/ready", "w").close(); time.sleep(float(os.environ.get("HOLD", "0")))
    except Exception:
        import traceback; traceback.print_exc()
    os._exit(0)
threading.Thread(target=run, daemon=True).start(); S.main()
