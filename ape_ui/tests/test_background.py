"""Background encoding, raw check, View from raw/archive, Transfer new demos and Clear (card 3), without ROS and without
a Training machine: the server runs with its stand-in monitor, demos are synthetic raw recordings dropped into
data/raw/<session>/ (as the logger does after a save), and a local folder stands in for the Training machine.
Env: RWR_WS = an empty folder with src/rwr_system -> this checkout; APE_RSYNC = an rsync >= 3.2.3.
Prints one line per check; "FAIL" marks a check that did not hold."""
import json, os, shutil, sys, threading, time, urllib.error, urllib.request
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")); sys.argv = ["server.py", "--port", "8768"]
os.environ["APE_ARCHIVE_REQUIRE_HF_BACKUP"] = "0"   # no Hugging Face markers in a local stand-in
import server as S
import h5py
S.external_launches = lambda own: []
B = "http://127.0.0.1:8768"
SES = "s99_2026-11-26_test"
RAW = S.DATA / "raw" / SES
DEST = S.WORKSPACE / "training_machine"
results = []


def check(name, ok, extra=""):
    results.append(bool(ok))
    print(("ok   " if ok else "FAIL ") + name + (f"  ({extra})" if extra else ""), flush=True)


def api(path, body=None, post=True):
    req = urllib.request.Request(B + path, data=json.dumps(body or {}).encode() if post else None, headers={"Content-Type": "application/json"})
    try:
        r = urllib.request.urlopen(req, timeout=30)
        return json.load(r) if r.headers.get_content_type() == "application/json" else r.read()
    except urllib.error.HTTPError as e:
        return {"ERR": e.code, "detail": e.read().decode()[:200]}


def wait(cond, seconds=60, step=0.2):
    end = time.time() + seconds
    while time.time() < end:
        if cond():
            return True
        time.sleep(step)
    return False


def make_raw(stem, seconds=3.0, fps=30, size=(180, 320)):
    """A raw recording in the logger's layout, written as .h5.part and renamed when complete (like the logger)."""
    RAW.mkdir(parents=True, exist_ok=True)
    part = RAW / f"{stem}.h5.part"
    t0 = 1_800_000_000_000_000_000 + int(stem[-6:]) * 10**9
    rng = np.random.default_rng(int(stem[-6:]))
    with h5py.File(part, "w") as h:
        def stream(topic, hz, fn):
            g = h.create_group(topic)
            for k in range(int(seconds * hz)):
                g.create_dataset(str(t0 + int(k * 1e9 / hz)), data=fn(k / hz))
        pose = lambda t: np.array([0.4 + 0.05 * np.sin(t), 0.1 * np.cos(t), 0.3, 0, 0, 0, 1.0])
        stream("/right/arm/end_effector_pose", 50, pose)
        stream("/right/arm/end_effector_pose_cmd", 50, pose)
        stream("/right/hand/joint_position_from_motor", 50, lambda t: np.full(17, 0.3 * np.sin(t), np.float32))
        stream("/right/hand/policy_output", 50, lambda t: np.full(17, 0.3 * np.sin(t), np.float32))
        y, x = np.mgrid[0:size[0], 0:size[1]]   # smooth, camera-like content (noise is very slow to encode)
        base = np.stack([x * 255 // size[1], y * 255 // size[0], (x + y + int(rng.integers(0, 99))) * 255 // (size[0] + size[1] + 99)], -1).astype(np.uint8)
        for i, cam in enumerate(("oakd_front_view", "oakd_wrist_view")):
            def img(t, i=i):
                a = base.copy(); a[..., i] //= 2; x = int(t * 60) % (size[1] - 40); a[60:100, x:x + 40] = 255
                return a
            stream(f"/{cam}/color", fps, img)
            g = h.create_group(f"/{cam}/intrinsics"); g.create_dataset(str(t0), data=np.eye(3))
    if part.with_suffix("").exists():
        part.with_suffix("").unlink()
    part.rename(RAW / f"{stem}.h5")


def demos():
    d = next((x for x in api("/api/datasets", post=False)["datasets"] if x["name"] == SES), None)
    return {m["name"]: m for m in d["demos"]} if d else {}


def run():
    try:
        local = lambda cmd: [a for a in cmd] + ["--local-dest", str(DEST)]
        orig_t, orig_c = S.station.transfer_command, S.station.clear_command
        S.station.transfer_command = lambda ds: local(orig_t(ds))
        S.station.clear_command = lambda ds: local(S.station._archive_job_args("clear", ds))
        time.sleep(2)
        st = api("/api/status", post=False)
        check("status has the background worker", "background" in st["features"] and "task" in st["background"])
        check("Upload to Hugging Face endpoint is gone", api("/api/upload").get("ERR") in (404, 405))
        check("old process endpoint is gone", api(f"/api/datasets/{SES}/process").get("ERR") in (404, 405))

        print("--- three demos saved one after the other: each is encoded, then checked")
        (RAW).mkdir(parents=True, exist_ok=True)
        (RAW / "20261126_150000.h5.part").write_bytes(b"half written")
        stems = ["20261126_150101", "20261126_150202", "20261126_150303"]
        seen_pct, seen_wait = set(), False
        make_raw(stems[0], size=(360, 640)); S.bg.wake()   # a bigger first one, so its progress can be seen
        threading.Thread(target=lambda: (time.sleep(1.0), make_raw(stems[1]), make_raw(stems[2]), S.bg.wake()), daemon=True).start()
        def watch():
            nonlocal seen_wait
            for m in demos().values():
                w = m["work"] or {}
                if w.get("task") == "encode" and w.get("pct") is not None:
                    seen_pct.add(w["pct"])
                if w.get("task") == "wait_encode":
                    seen_wait = True
            ds = demos()
            return len(ds) == 3 and all(m["archive"] and m["archive"]["state"] == "archived" for m in ds.values())
        t0 = time.time()
        check("all three encoded", wait(watch, 240, 0.1), f"{time.time() - t0:.0f} s")
        check("the .h5.part file is ignored", "20261126_150000" not in demos())
        check("encoding progress was visible between 0 and 100 %", any(0 < p < 100 for p in seen_pct), f"seen {sorted(seen_pct)[:8]}")
        check("a demo waited for its turn", seen_wait)
        check("all three checked on the raw file", wait(lambda: all(m["check"] for m in demos().values()), 120),
              f"{[(m['check'] or {}).get('ok') for m in demos().values()]}")
        m = demos()[stems[0]]
        check("check result: ok, length about 3 s", m["check"]["ok"] and 2.5 < (m["check"]["duration_s"] or 0) < 3.5, f"{m['check']}")
        check("trajectory plot from the raw check", isinstance(api(f"/api/demo/{SES}/{stems[0]}/trajectory.png", post=False), bytes))

        print("--- View from the raw file")
        meta = api(f"/api/demo/{SES}/{stems[0]}/meta", post=False)
        check("source raw, 90 frames at 30 Hz, both cameras", meta.get("source") == "raw" and meta["frames"] == 90 and abs(meta["freq"] - 30) < 0.5 and len(meta["cameras"]) == 2, f"{meta}")
        raw_jpg = api(f"/api/demo/{SES}/{stems[0]}/frame/45.jpg", post=False)
        check("frame 45 from the raw file", isinstance(raw_jpg, bytes) and raw_jpg[:2] == b"\xff\xd8")

        print("--- a policy starts while a demo is being encoded: encoding stops, and is redone after")
        make_raw("20261126_150404"); S.bg.wake()
        check("encoding started", wait(lambda: (demos().get("20261126_150404", {}).get("work") or {}).get("task") == "encode", 30, 0.05))
        S.inference.policy_on = True
        check("stopped while the policy runs", wait(lambda: S.bg.current is None and "paused" in S.bg.info()["note"], 10))
        time.sleep(4)
        w = demos()["20261126_150404"]
        check("nothing encoded meanwhile, shown as waiting", (w["work"] or {}).get("task") == "wait_encode" and not w["archive"], f"{w['work']}, {w['archive']}")
        check("no leftover progress or partial files", wait(lambda: not list((S.DATA / "archive" / SES).glob(".20261126_150404*")), 5),
              [x.name for x in (S.DATA / "archive" / SES).glob(".20261126_150404*")])
        S.inference.policy_on = False; S.bg.wake()
        check("encoded after the policy stopped", wait(lambda: (demos()["20261126_150404"]["archive"] or {}).get("state") == "archived", 120))

        print("--- a recording starts while a demo is being encoded: the encode is suspended, and continues after the save")
        pstate = lambda pid: open(f"/proc/{pid}/stat").read().rsplit(")", 1)[1].split()[0]
        group = lambda pgid: [int(p) for p in os.listdir("/proc") if p.isdigit() and os.path.exists(f"/proc/{p}/stat") and int(open(f"/proc/{p}/stat").read().rsplit(")", 1)[1].split()[2]) == pgid]
        make_raw("20261126_150450", size=(360, 640)); S.bg.wake()
        check("encoding started", wait(lambda: S.bg.current is not None and S.bg.current["stem"] == "20261126_150450", 30, 0.05))
        time.sleep(0.5)
        pid = S.bg.current["proc"].pid
        S.operate.rec_starting = True; S.bg.suspend_now(); S.operate.rec, S.operate.rec_starting = "recording", False
        time.sleep(0.3)
        states = {p: pstate(p) for p in group(pid)}
        check("every process of the encode is stopped (T) while recording", states and set(states.values()) == {"T"}, f"{states}")
        pct = (demos()["20261126_150450"]["work"] or {}).get("pct")
        time.sleep(3)
        check("no progress while recording", (demos()["20261126_150450"]["work"] or {}).get("pct") == pct and S.bg.current and S.bg.current["stem"] == "20261126_150450", f"{pct}")
        check("page says why", "recorded" in S.bg.info()["note"], S.bg.info()["note"])
        S.operate.rec = "saving"; time.sleep(1)
        check("still held while saving", set(pstate(p) for p in group(pid)) == {"T"})
        S.operate.rec = "idle"; S.bg.wake()
        check("continued after the save and finished", wait(lambda: (demos()["20261126_150450"]["archive"] or {}).get("state") == "archived", 120))
        log = open(S.bg.log_file).read()
        check("log shows suspended and continued, encoded once", "150450 suspended" in log and "150450 continued" in log and log.count(f"encode {SES}/20261126_150450:") == 1)
        S.operate.rec = "recording"
        make_raw("20261126_150460"); S.bg.wake(); time.sleep(4)
        c = S.bg.current
        check("no new task starts during a recording (a running one stays suspended)", (c is None or (c.get("held") and c["stem"] != "20261126_150460"))
              and not demos()["20261126_150460"]["archive"] and not list((S.DATA / "archive" / SES).glob(".20261126_150460*")), f"{c and (c['task'], c['stem'], c.get('held'))}")
        S.operate.rec = "idle"; S.bg.wake()
        check("it starts after the recording", wait(lambda: (demos()["20261126_150460"]["archive"] or {}).get("state") == "archived", 120))

        print("--- Transfer new demos while a demo is being encoded: the transfer waits for it")
        make_raw("20261126_150505"); S.bg.wake()
        wait(lambda: S.bg.current is not None and S.bg.current["stem"] == "20261126_150505", 30, 0.05)
        r = api(f"/api/datasets/{SES}/transfer")
        check("transfer asked for", r.get("queued") is True, f"{r}")
        check("queued, not started while encoding", SES in S.bg.info()["transfer_wanted"] and not S.job.running)
        check("second press while queued is accepted", api(f"/api/datasets/{SES}/transfer").get("queued") is True)
        check("transfer job started after the encode", wait(lambda: S.job.running and S.job.dataset == "transfer " + SES, 120, 0.05))
        mid = demos()
        check("demos shown as copying (or already verified) during the transfer", all(m["archive"]["state"] in ("archived", "not_verified", "verified") for m in mid.values()))
        make_raw("20261126_150606"); S.bg.wake()
        time.sleep(1)
        check("a demo saved during the transfer waits for the job", S.bg.current is None and "card 3" in S.bg.info()["note"], S.bg.info()["note"])
        check("transfer job ended", wait(lambda: not S.job.running, 120), f"exit {S.job.exit_code}")
        ds = demos()
        n_ok = sum(1 for m in ds.values() if m["archive"]["state"] == "verified")
        check("all demos encoded before or by the job are verified on the stand-in Training machine", n_ok >= 7,
              f"{ {k: (m['archive'] or {}).get('state') for k, m in ds.items()} }")
        check("only archives copied, no processed files", (DEST / "archive" / SES).is_dir() and not (DEST / "processed").exists())
        check("the demo saved during the transfer: taken by the job, or encoded afterwards", wait(lambda: (demos()["20261126_150606"]["archive"] or {}).get("state") in ("archived", "verified"), 120),
              (demos()["20261126_150606"]["archive"] or {}).get("state"))
        make_raw("20261126_150808"); S.bg.wake()
        check("a demo saved after the transfer is encoded, not transferred", wait(lambda: (demos()["20261126_150808"]["archive"] or {}).get("state") == "archived", 120))

        print("--- a damaged recording: reported, not retried in the background")
        (RAW / "20261126_150707.h5").write_bytes(b"not an hdf5 file")
        S.bg.wake()
        check("encoding failed is shown", wait(lambda: (demos().get("20261126_150707", {}).get("archive") or {}).get("state") == "archive_failed", 60),
              f"{(demos().get('20261126_150707') or {}).get('archive')}")
        time.sleep(5)
        tries = open(S.bg.log_file).read().count(f"encode {SES}/20261126_150707")
        check("not retried in the background", tries == 1 and demos()["20261126_150707"]["work"] is None, f"{tries} encode attempt(s)")

        print("--- View from the archive when the raw file is not there (moved aside here)")
        ok = wait(lambda: not S.job.running and S.bg.current is None, 60)
        aside = S.WORKSPACE / "aside.h5"; shutil.move(RAW / f"{stems[0]}.h5", aside)
        m = demos()[stems[0]]
        check("demo without raw file still viewable from its archive", m["raw_gb"] is None and m["viewable"], f"{m}")
        meta2 = api(f"/api/demo/{SES}/{stems[0]}/meta", post=False)
        check("source archive, same frames and rate", meta2.get("source") == "archive" and meta2["frames"] == meta["frames"] and abs(meta2["freq"] - meta["freq"]) < 0.01, f"{meta2}")
        arc_jpg = api(f"/api/demo/{SES}/{stems[0]}/frame/45.jpg", post=False)
        import cv2
        a = cv2.imdecode(np.frombuffer(raw_jpg, np.uint8), 1).astype(float); b = cv2.imdecode(np.frombuffer(arc_jpg, np.uint8), 1).astype(float)
        psnr = 10 * np.log10(255 ** 2 / max(1e-9, ((a - b) ** 2).mean()))
        check("frame 45 from the archive looks like the raw one", psnr > 30, f"PSNR {psnr:.1f} dB")
        seq = [api(f"/api/demo/{SES}/{stems[0]}/frame/{i}.jpg", post=False) for i in (0, 1, 2, 3, 80, 10)]
        check("sequential playback and jumps decode", all(isinstance(x, bytes) and x[:2] == b"\xff\xd8" for x in seq))
        shutil.move(aside, RAW / f"{stems[0]}.h5")
        check("back on the raw file once it is there again", api(f"/api/demo/{SES}/{stems[0]}/meta", post=False).get("source") == "raw")

        print("--- Clear")
        r = api(f"/api/datasets/{SES}/clear")
        check("clear started", "ERR" not in r and ok, f"{r}")
        check("clear ended", wait(lambda: not S.job.running, 120), f"exit {S.job.exit_code}")
        m = demos()[stems[0]]
        check("verified demo cleared: raw file gone, archive kept, still listed with its check", m["raw_gb"] is None and m["archive"]["state"] == "cleared" and m["viewable"] and bool(m["check"])
              and (S.DATA / "archive" / SES / f"{stems[0]}.archive.json").is_file(), f"{m}")
        check("a cleared demo plays from its archive", api(f"/api/demo/{SES}/{stems[0]}/meta", post=False).get("source") == "archive"
              and isinstance(api(f"/api/demo/{SES}/{stems[0]}/frame/45.jpg", post=False), bytes))
        d = next(x for x in api("/api/datasets", post=False)["datasets"] if x["name"] == SES)
        check("nothing left to clear; disk use counts the kept archives", d["clearable"] == 0 and d["total_gb"] >= 0, f"clearable {d['clearable']}, {d['total_gb']} GB")
        check("demo not yet transferred is still on this PC", demos()["20261126_150808"]["raw_gb"] is not None and demos()["20261126_150808"]["archive"]["state"] in ("archived", "not_verified"),
              demos()["20261126_150808"]["archive"])
        check("Frames column counts workspace camera frames", wait(lambda: (demos()["20261126_150808"]["check"] or {}).get("frames") == 90, 60), demos()["20261126_150808"]["check"])
    except Exception:
        import traceback; traceback.print_exc(); results.append(False)
    finally:
        print(f"=== {sum(results)} of {len(results)} checks ok", flush=True)
        S.cleanup(); os._exit(0 if all(results) else 1)


threading.Thread(target=run, daemon=True).start()
S.uvicorn.run(S.app, host="127.0.0.1", port=8768, log_level="critical")
