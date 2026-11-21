"""Policy card: listing, checks, preview and live launch, policy commands and their guards.
Runs against mock_station.py (start it with MOCK_CONTROLLER_LISTENS=1) and mock_inference.py on a test ROS domain.
Env: APE_ROS_DOMAIN_ID (not 7), APE_POLICY_DIR = a folder with one run that has config.yaml and checkpoints/*.pth,
APE_INFERENCE_TEST_CMD = '["python", ".../tests/mock_inference.py"]'."""
import sys, os, threading, time, json, urllib.request, urllib.error, urllib.parse
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); sys.argv = ["server.py", "--port", "8766"]
import server as S
assert S.ROS_ENV["ROS_DOMAIN_ID"] != "7" and os.environ.get("APE_INFERENCE_TEST_CMD"), "test domain and the mock inference command are required"
S.external_launches = lambda own: []   # launches of the real station (another ROS domain) are not this test's business
nuc_calls = []; S.nuc.start_controller = lambda: nuc_calls.append("start")   # never open a connection to the real NUC from a test
B = "http://127.0.0.1:8766"
def api(path, body=None, post=True):
    req = urllib.request.Request(B + path, data=json.dumps(body or {}).encode() if post else None, headers={"Content-Type": "application/json"})
    try: return json.load(urllib.request.urlopen(req, timeout=20))
    except urllib.error.HTTPError as e: return {"ERR": json.loads(e.read().decode()).get("detail", "")[:230]}
inf = lambda: api("/api/status", post=False)["procs"]["inference"]
def wait(cond, seconds=12):
    end = time.time() + seconds
    while time.time() < end:
        if cond(): return True
        time.sleep(0.2)
    return False
torque = lambda: [l for l in json.load(open(os.environ.get("MOCK_DIR", "/tmp") + "/mock_state.json"))["log"] if l.startswith("torque")]
def run():
    try:
        time.sleep(7)
        st = api("/api/status", post=False); print("features:", st["features"], "| inference running:", st["procs"]["inference"]["running"])
        pl = api("/api/policies", post=False); run_, ck = pl["runs"][0]["name"], pl["runs"][0]["checkpoints"][0]["name"]
        print("runs:", [(r["name"], r["dataset"], r["algo"], [c["name"] for c in r["checkpoints"]]) for r in pl["runs"]])
        q = urllib.parse.urlencode({"run": run_, "ckpt": ck})
        c = api("/api/policies/check?" + q, post=False); print("check:", {i["id"]: i["status"] for i in c["items"]}, "| bad", c["bad"], "warn", c["warn"])
        print("check of a missing checkpoint ->", api("/api/policies/check?run=nope&ckpt=x.pth", post=False).get("ERR"))
        print("path escape ->", api("/api/policies/check?" + urllib.parse.urlencode({"run": run_, "ckpt": "../../x.pth"}), post=False).get("ERR"))
        print("command without a launch ->", api("/api/inference/cmd/start").get("ERR"))
        # the mock controller reports no end-effector values (all zero), the demos have real ones: the check must fail and block a live launch
        ee = next(i for i in c["items"] if i["id"] == "ee"); print("ee check:", ee["status"], "|", ee["text"][:110])
        print("live launch with a failed check ->", api("/api/inference/start", {"run": run_, "ckpt": ck, "preview": False}).get("ERR"))

        print("--- preview")
        r = api("/api/inference/start", {"run": run_, "ckpt": ck, "preview": True}); print("start preview:", r.get("ERR") or (r["running"], r["preview"], r["arm_init"], r["command"][-60:]))
        print("second launch ->", api("/api/inference/start", {"run": run_, "ckpt": ck, "preview": True}).get("ERR"))
        print("station launch while a policy launch runs ->", api("/api/recording/start", {"dataset_name": "s99_test"}).get("ERR"))
        print("command while loading ->", api("/api/inference/cmd/start").get("ERR"))
        print("node ready:", wait(lambda: inf()["node_ready"]))
        print("init_arm in preview ->", api("/api/inference/cmd/init_arm").get("ERR"))
        print("start policy:", api("/api/inference/cmd/start").get("ERR", "accepted"), "| policy_on:", wait(lambda: inf()["policy_on"]), "| producing:", wait(lambda: inf()["producing"]))
        op = api("/api/status", post=False)["operate"]; print("operate: policy_running", op["policy_running"], "teleop_running", op["teleop_running"])
        live = S.monitor.live(); print("live: policy", live["policy"], "arm_cmd", live["arm_cmd"][:3], "hand_cmd[0]", live["hand_cmd"][0])
        print("arm commands reach the controller topic in preview:", S.monitor.commands_flowing, "| torque calls:", torque())
        print("start twice ->", api("/api/inference/cmd/start").get("ERR"))
        print("processing while the policy runs ->", api("/api/datasets/no_such_session/process").get("ERR"))
        time.sleep(1.5); i = inf(); print("pauses:", i["pauses"], "| last:", i["last_pause"] and i["last_pause"]["text"])
        print("stop policy:", api("/api/inference/cmd/stop").get("ERR", "accepted"), "| policy_on:", not wait(lambda: not inf()["policy_on"]), "| producing:", not wait(lambda: not inf()["producing"]))
        c2 = api("/api/policies/check?" + q, post=False); print("previewed check:", next(i for i in c2["items"] if i["id"] == "preview")["status"], "|", api("/api/policies", post=False)["runs"][0]["checkpoints"][0]["previewed"])
        print("stop launch:", api("/api/inference/stop")["stopping"], "| stopped:", wait(lambda: not inf()["running"], 25), "| exit", inf()["exit_code"])

        print("--- live (with the failed check confirmed)")
        r = api("/api/inference/start", {"run": run_, "ckpt": ck, "preview": False, "override": True}); print("start live:", r.get("ERR") or (r["running"], r["preview"], r["arm_init"]), "|", r.get("command", "")[-120:])
        print("node ready:", wait(lambda: inf()["node_ready"]), "| torque calls:", torque())
        print("start policy without confirming the failed check ->", api("/api/inference/cmd/start").get("ERR"))
        n0 = len(torque())
        print("init_arm:", api("/api/inference/cmd/init_arm").get("note") or "refused", "| init:", inf()["init"])
        print("start during the move ->", api("/api/inference/cmd/start", {"override": True}).get("ERR"))
        print("move done:", wait(lambda: inf()["init"] == "done"), "| init:", inf()["init"])
        print("log tail:", b"".join(d for _, d in S.inference.read_since(0)).decode(errors="replace")[-700:].replace("\r", ""))
        print("start policy (confirmed):", api("/api/inference/cmd/start", {"override": True}).get("ERR", "accepted"), "| policy_on:", wait(lambda: inf()["policy_on"]),
              "| commands reach the controller topic:", wait(lambda: S.monitor.commands_flowing), "| torque calls since:", torque()[n0:])
        op = api("/api/status", post=False)["operate"]; print("operate: policy_running", op["policy_running"], "teleop_running", op["teleop_running"], "hand_following", op["hand_following"])
        print("NUC start while the policy runs ->", api("/api/nuc/start").get("ERR"), "| reached the NUC:", nuc_calls)
        # the robot drops out of Ready: APE must stop the policy by itself
        Monitor, info = type(S.monitor), type(S.monitor).info
        def reflex(self):
            d = info(self)
            if d["robot"]: d["robot"] = dict(d["robot"], mode="reflex")
            return d
        Monitor.info = reflex
        print("robot in reflex -> policy stopped by APE:", wait(lambda: not inf()["policy_on"], 6), "| note:", inf()["note"])
        print("start while in reflex ->", api("/api/inference/cmd/start", {"override": True}).get("ERR"))
        Monitor.info = info
        time.sleep(4); print("torque calls after the stop:", torque()[n0:])
        print("stop launch:", api("/api/inference/stop")["stopping"], "| stopped:", wait(lambda: not inf()["running"], 25))
        print("station launch possible again:", "launch guard passed" if S._launch_guard() is None else "")
    except Exception:
        import traceback; traceback.print_exc()
    os._exit(0)
threading.Thread(target=run, daemon=True).start(); S.main()
