"""Evaluation of policy runs (card 5), without ROS and without the robot: the server runs with a stand-in monitor, the policy
launch is tests/mock_inference_stdin.py, and camera frames and arm/hand samples are fed in by this script.
Env: RWR_WS = an empty folder, APE_POLICY_DIR = a folder with one run (config.yaml + checkpoints/*.pth),
APE_INFERENCE_TEST_CMD = '["python", ".../tests/mock_inference_stdin.py"]'. Optional: APE_TEST_DEMO = a processed demo (.h5)
with one relocation; the workspace camera then shows its first image, and the demo itself during the first live run:
the live count and the recount of that run must both be 1."""
import sys, os, threading, time, json, math, types, urllib.request, urllib.error
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); sys.argv = ["server.py", "--port", "8767"]
import server as S
import eval_trials; S.eval_trials_module = eval_trials
assert os.environ.get("APE_INFERENCE_TEST_CMD"), "the mock inference command is required"
S.external_launches = lambda own: []
B = "http://127.0.0.1:8767"
def api(path, body=None, post=True):
    req = urllib.request.Request(B + path, data=json.dumps(body or {}).encode() if post else None, headers={"Content-Type": "application/json"})
    try: return json.load(urllib.request.urlopen(req, timeout=20))
    except urllib.error.HTTPError as e:
        text = e.read().decode()
        try: return {"ERR": json.loads(text).get("detail", "")[:200]}
        except ValueError:   # a fault inside the server: show it
            try: S.status()
            except Exception: import traceback; traceback.print_exc()
            raise RuntimeError(f"{path} -> {e.code} {text[:200]}")
ev = lambda: api("/api/status", post=False)["eval"]
def wait(cond, seconds=15):
    end = time.time() + seconds
    while time.time() < end:
        if cond(): return True
        time.sleep(0.1)
    return False
feeding = {"on": True, "robot": "ready", "move": True, "demo": None}
DEMO = None
if os.environ.get("APE_TEST_DEMO"):
    import h5py
    with h5py.File(os.environ["APE_TEST_DEMO"]) as f: DEMO = f["observations/images/oakd_front_view/color"][:]
def feed():   # what the ROS callbacks do: 30 camera frames per second, arm at 50 Hz, hand at 20 Hz
    m, k = S.monitor, 0
    while feeding["on"]:
        now = time.time(); k += 1
        m.robot, m.robot_stamp, m.checks = {"mode": feeding["robot"], "mode_id": 2, "success_rate": 1.0, "current_errors": [], "last_motion_errors": []}, now, {"controller_listening": 1}
        a = now if feeding["move"] else 0.0
        m.tele["arm"] = [0.45 + 0.1 * math.sin(a), 0.1 * math.cos(a), 0.3, 1.0, 0.0, 0.0, 0.0]; m.arm_pos, m.arm_stamp = tuple(m.tele["arm"][:3]), now
        S.evals.feed("arm", m.tele["arm"]); S.evals.feed("policy_arm", m.tele["arm"])
        if k % 3 == 0: S.evals.feed("hand", [0.6 + 0.5 * math.sin(a * 2)] * 17 if feeding["move"] else [0.6] * 17); S.evals.feed("hand_temp", [40.0] * 17)
        if k % 2 == 0:
            for cam, (h, w) in (("oakd_front_view", (360, 640)), ("oakd_wrist_view", (640, 360))):
                img = np.full((h, w, 3), 60, np.uint8); img[:, : int(w * (now % 1.0))] = (200, 40, 40)
                if DEMO is not None and cam == "oakd_front_view":   # the demo's first image while nothing runs, then the demo at its own speed
                    img = DEMO[min(len(DEMO) - 1, int((now - feeding["demo"]) * 10))] if feeding["demo"] is not None else DEMO[0]; h, w = img.shape[:2]
                msg = types.SimpleNamespace(height=h, width=w, step=w * 3, encoding="rgb8", data=img.tobytes())
                S.evals.feed_frame(cam, msg)
                if cam == S.live.cam: S.live.feed(msg)
        time.sleep(1 / 60)
def run():
    try:
        time.sleep(3)
        m = S.monitor; print("ROS available:", m.ok, "(expected False here)")
        m.policy_command = lambda text: S.inference.write(text.encode() + b"\n")   # the mock takes commands on its terminal
        S.inference.node_ready = lambda: True
        threading.Thread(target=feed, daemon=True).start()
        st = api("/api/status", post=False); print("features has eval:", "eval" in st["features"], "| state:", st["eval"]["state"])
        pl = api("/api/policies", post=False); run_, ck = pl["runs"][0]["name"], pl["runs"][0]["checkpoints"][0]["name"]
        print("marking by hand is gone ->", api("/api/eval/event/placed").get("ERR", "still there"))

        print("--- a preview run is not evaluated")
        r = api("/api/inference/start", {"run": run_, "ckpt": ck, "preview": True}); print("launch preview:", r.get("ERR") or r["running"]); time.sleep(1.0)
        print("start policy:", api("/api/inference/cmd/start").get("ERR", "accepted"), "| policy on:", wait(lambda: S.inference.policy_on)); time.sleep(1.5)
        print("state:", ev()["state"])
        api("/api/inference/cmd/stop"); time.sleep(1.0); print("nothing saved:", api("/api/eval", post=False)["total"] == 0 and not S.evals.folder.exists())
        print("stop launch:", api("/api/inference/stop")["stopping"], wait(lambda: not S.inference.running, 25))

        print("live count stopped with the launch:", wait(lambda: not S.live.running, 5))
        print("--- live runs: Start policy and Stop policy of card 4")
        r = api("/api/inference/start", {"run": run_, "ckpt": ck, "preview": False, "override": True}); print("launch live:", r.get("ERR") or r["running"]); time.sleep(1.0)
        print("nothing recorded before the policy starts:", ev()["state"] == "idle" and ev()["current"] is None, "| live count running with the launch:", wait(lambda: ev()["live"]["fresh"], 15))
        feeding["robot"] = "reflex"; time.sleep(0.3)
        print("start while the robot is not Ready ->", api("/api/inference/cmd/start", {"override": True}).get("ERR"), "| state:", ev()["state"], "| nothing kept:", not S.evals.folder.exists() or not any(S.evals.folder.iterdir()))
        done = lambda: (api("/api/eval", post=False)["trials"][0].get("detection") or {}).get("state") == "done"
        for i, (how, seconds) in enumerate([("operator", len(DEMO) / 10 + 0.5 if DEMO is not None else 7.0), ("robot", 2.0), ("launch_stopped", 2.0)]):
            feeding["robot"] = "ready"; time.sleep(1.2)   # run ids are per second
            if i == 0 and DEMO is not None: feeding["demo"] = time.time()
            r = api("/api/inference/cmd/start", {"override": True}); assert "ERR" not in r, r
            assert wait(lambda: ev()["state"] == "running"); time.sleep(0.5)
            if i == 0:
                print("processing during a run ->", api("/api/datasets/nope/process").get("ERR")); feeding["move"] = False   # arm and hand stand still: the automatic measure must see it
            if i == 0 and DEMO is not None:   # the count must go up while the demo plays, not after it
                seen = []
                for _ in range(int(seconds * 4)):
                    time.sleep(0.25); lv = ev()["live"]; seen.append((len(lv["placed"]), lv["colour"], lv["cube"] is not None, lv["problem"]))
                first = next((k for k, x in enumerate(seen) if x[0] == 1), None)
                print("   live while the demo plays: count went", seen[0][0], "->", seen[-1][0], "| after", None if first is None else round(first * 0.25, 1), "s of", round(len(seen) * 0.25, 1), "| colour", seen[-1][1], "| cube seen in", sum(x[2] for x in seen), "of", len(seen), "polls | problems", sorted({x[3] for x in seen}))
                lv = ev()["live"]; img = np.zeros((540, 960, 3), np.uint8); S._draw_tracking(img); print("   overlay drawn:", int((img > 0).any(axis=2).sum()), "pixels | hull points", len(lv["hull"] or []), "| size", lv["size"], "| trail", sum(p is not None for p in lv["trail"]))
            else: time.sleep(seconds)
            if i == 0: e = ev(); print("current:", sorted(e["current"]), "| elapsed > 5:", e["current"]["elapsed"] > 5, "| state (no time limit):", e["state"])
            if how == "operator": api("/api/inference/cmd/stop")
            if how == "robot": feeding["robot"] = "reflex"
            if how == "launch_stopped": api("/api/inference/stop")
            assert wait(lambda: ev()["state"] == "idle", 30); feeding["move"], feeding["demo"] = True, None
            t = api("/api/eval", post=False)["trials"][0]; print(f"live run {i + 1}: end {t['end_reason']} (wanted {how}) | duration {t['duration_s']} | detection right after saving: {t['detection']['state']}")
            print("   counted:", wait(done, 120), end=" | "); t = api("/api/eval", post=False)["trials"][0]
            print("live", t.get("live"), "| recount", t["detection"], "| result", t["result"], "| events", t["events"])
            if i == 0:
                tid = t["id"]; print("   message:", repr(ev()["message"]), "| video starts at", {c: v["starts_at_s"] for c, v in t["video"].items()}, "| robot at end", t["robot_mode_at_end"], "| still", t["auto"].get("still"))
                print("   video:", {c: (v["frames"], v["video_frames"], v["codec"], v["size"], v["error"]) for c, v in t["video"].items()}, "| frames lost:", t["video_frames_lost"], "| problems:", t["recording_problems"])
                import h5py, cv2
                with h5py.File(S.evals.folder / t["folder"] / "signals.h5") as f:
                    print("   h5:", {k: f[k]["data"].shape for k in f if k != "video"}, "| video frames", {c: len(f["video"][c]["t"]) for c in f["video"]})
                probe = lambda path: (lambda c: (int(c.get(cv2.CAP_PROP_FRAME_COUNT)), c.read()[0]))(cv2.VideoCapture(str(path)))
                copy = S.evals.video(tid, "oakd_wrist_view"); rec = S.evals.folder / t["folder"] / "oakd_wrist_view.mp4"
                print("   recorded with the archive's video settings:", t["video"]["oakd_wrist_view"]["video_args"] == S.eval_trials_module.VIDEO_ARGS and "yuv444p" in t["video"]["oakd_wrist_view"]["video_args"],
                      "| recording readable (frames, first frame):", probe(rec), "| the page gets a 4:2:0 copy outside data/eval:", copy != rec and S.evals.folder not in copy.parents, probe(copy))
                print("   video endpoint:", urllib.request.urlopen(B + f"/api/eval/trial/{tid}/oakd_wrist_view.mp4").status, "| bad id ->", api("/api/eval/trial/..%2f..%2fx/oakd_wrist_view.mp4", post=False).get("ERR"))
                n0 = t["result"]["placed"]; u = api(f"/api/eval/trial/{tid}", {"add": "placed", "t": 2.5}); u = api(f"/api/eval/trial/{tid}", {"add": "placed"}); u = api(f"/api/eval/trial/{tid}", {"remove": "placed"}); u = api(f"/api/eval/trial/{tid}", {"note": "checked on video"})
                print("   after corrections (+2, -1):", u["result"]["placed"] - n0, "| sources", [e["source"] for e in u["events"]], "| note", u["note"], "| unknown kind ->", api(f"/api/eval/trial/{tid}", {"add": "dropped"}).get("ERR"))
        print("csv:"); print(open(S.evals.folder / "trials.csv").read())
        t = api("/api/eval", post=False); want = f"{run_}/{ck.rsplit('/', 1)[-1].rsplit('.', 1)[0]}/{t['trials'][0]['id']}"
        print("folder layout <policy run>/<checkpoint>/<run id>:", t["trials"][0]["folder"] == want and (S.evals.folder / want / "trial.json").is_file(), "| nothing left directly in eval/:", sorted(x.name for x in S.evals.folder.iterdir()) == sorted([run_, "trials.csv"]))
        print("upload state per run:", {x["upload"] for x in t["trials"]}, "| summary:", t["upload"], "| upload without a Training machine ->", api("/api/eval/upload").get("ERR"))
        print("--- a live policy started outside the page is picked up")
        wait(lambda: not S.inference.running, 25); feeding["robot"] = "ready"; api("/api/inference/start", {"run": run_, "ckpt": ck, "preview": False, "override": True}); time.sleep(1.5)
        S.inference.write(b"start\n"); print("evaluating:", wait(lambda: ev()["state"] == "running")); time.sleep(1.5); S.inference.write(b"stop\n")
        print("saved:", wait(lambda: ev()["state"] == "idle", 30), "| flagged:", api("/api/eval", post=False)["trials"][0].get("started_outside_page"), "| end", api("/api/eval", post=False)["trials"][0]["end_reason"])
    except Exception:
        import traceback; traceback.print_exc()
    finally:
        feeding["on"] = False; S.cleanup(); os._exit(0)
threading.Thread(target=run, daemon=True).start()
S.uvicorn.run(S.app, host="127.0.0.1", port=8767, log_level="critical")
