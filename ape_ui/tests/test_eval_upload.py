"""Folder layout of the evaluated runs and their upload, without ROS, the robot or a second machine:
    python tests/test_eval_upload.py        (needs numpy and rsync 3.2.3 or newer; a local folder stands in for the Training machine)
Checks: new runs are saved in <policy run>/<checkpoint>/<run id>/, old flat runs are moved there, trials.csv names the
folder, the upload copies and verifies complete runs only, a changed trial.json is sent again, an existing video is
never overwritten, a file with other content on the Training machine is reported, and each run gets policy_log.h5:
the inference node's log without the images (needs h5py)."""
import csv, json, os, shutil, sys, tempfile, time
from pathlib import Path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import eval_trials as E
import eval_upload as U

RSYNC = os.environ.get("APE_RSYNC") or shutil.which("rsync") or str(Path.home() / ".local/rsync/rsync")
FAILED = []
def check(name, ok, extra=""):
    print(("ok    " if ok else "FAIL  ") + name + (f"  [{extra}]" if extra and not ok else ""))
    if not ok: FAILED.append(name)

def old_run(folder, tid, run, ckpt):   # a run as APE saved it before the grouped layout
    d = folder / tid; d.mkdir(parents=True)
    for name in ("oakd_front_view.mp4", "oakd_wrist_view.mp4", "signals.h5"): (d / name).write_bytes(os.urandom(2000))
    (d / "trial.json").write_text(json.dumps({"id": tid, "started": "x", "run": run, "ckpt": ckpt, "events": [], "duration_s": 10.0, "result": {"placed": 0}}, indent=1))

def upload(data, dest):
    return U.main(["--data", str(data), "--local-dest", str(dest), "--rsync", RSYNC, "--log-root", str(Path(data).parent / "ws")])

def node_log(path, steps=40, chunk=10, every=5):   # as experiments/model_inference_node.py writes it: step_<i>/{actions,observations,dbg}/<key>
    import h5py, numpy as np
    path.parent.mkdir(parents=True, exist_ok=True); rng = np.random.default_rng(1)
    with h5py.File(path, "w") as f:
        for i in range(steps):
            n = min(i // every + 1, chunk)   # the ensembling buffer grows until it holds `chunk` chunks
            f.create_dataset(f"step_{i}/actions/actions_arm", data=rng.normal(size=7).astype(np.float32))
            f.create_dataset(f"step_{i}/observations/qpos_hand", data=rng.normal(size=(1, 17)))
            f.create_dataset(f"step_{i}/observations/wrist_view_images", data=rng.random((1, 3, 224, 224)))
            f.create_dataset(f"step_{i}/dbg/aggr_buffer_actions_arm", data=rng.normal(size=(1, n, chunk, 7)).astype(np.float32))
            f.create_dataset(f"step_{i}/dbg/aggr_mask_actions_arm", data=rng.random((1, n, chunk)) > 0.5)
            if i % every == 0: f.create_dataset(f"step_{i}/dbg/step_raw_actions_arm", data=rng.normal(size=(1, chunk, 7)).astype(np.float32))

def main():
    tmp = Path(tempfile.mkdtemp(prefix="ape_eval_upload_")); data, dest = tmp / "data", tmp / "training_machine"
    ev = data / "eval"
    old_run(ev, "20261125-114115", "Ampere_20261121-180315_s01_v1_act", "checkpoints/best_model.pth")
    old_run(ev, "20261125-114543", "Ampere_20261121-180315_s01_v1_act", "checkpoints/checkpoint_100.pth")
    old_run(ev, "20261125-115017", "Avogadro_20261123-224214_s01s02s03_v1_act", "checkpoints/best_model.pth")
    logname = "inference_logs/Avogadro_20261123-224214_s01s02s03_v1_act-best_model/20261125-115017.hdf5"   # this run's inference log is still there
    t = json.loads((ev / "20261125-115017/trial.json").read_text()); t["inference_log"] = logname; (ev / "20261125-115017/trial.json").write_text(json.dumps(t, indent=1))
    node_log(tmp / "ws" / logname); log_bytes = (tmp / "ws" / logname).read_bytes()
    (ev / "20261125-120000").mkdir(); (ev / "20261125-120000" / "oakd_front_view.mp4").write_bytes(b"half")   # a run that was never saved
    T = E.EvalTrials(ev, ("oakd_front_view", "oakd_wrist_view"))
    check("old flat runs are listed before the move", len(T.trials()) == 3)
    check("the upload refuses to send flat runs", upload(data, dest) == 1 and not (dest / "eval").exists())
    moved, problems = T.migrate()
    check("3 runs moved, no problems", len(moved) == 3 and not problems, f"{moved} {problems}")
    want = ev / "Ampere_20261121-180315_s01_v1_act" / "checkpoint_100" / "20261125-114543"
    check("checkpoint folder is the file name without .pth", (want / "trial.json").is_file() and (want / "signals.h5").is_file())
    check("trial.json names its folder", T.get("20261125-114543").get("folder") == "Ampere_20261121-180315_s01_v1_act/checkpoint_100/20261125-114543")
    check("the unsaved folder stays where it is", (ev / "20261125-120000").is_dir())
    check("moving again changes nothing", T.migrate() == ([], []))
    check("video is found by run id", T.video("20261125-115017", "oakd_front_view").parent.parent.name == "best_model")
    rows = list(csv.DictReader(open(ev / "trials.csv")))
    check("trials.csv has the folder of each run", [r["folder"] for r in rows] == sorted(moved, key=lambda m: m.split("/")[-1]), str(rows))

    # a new run goes straight into the grouped layout
    tid = T.begin({"run": "Bohr_20261123-224905_s01s02s03_v1_act", "ckpt": "checkpoints/best_model.pth", "note": ""})
    new = ev / "Bohr_20261123-224905_s01s02s03_v1_act" / "best_model" / tid
    check("a new run is recorded in <policy run>/<checkpoint>/<run id>", new.is_dir() and T.current["folder"] == f"Bohr_20261123-224905_s01s02s03_v1_act/best_model/{tid}")
    T.cancel(); check("a cancelled run leaves no folder behind", not (ev / "Bohr_20261123-224905_s01s02s03_v1_act").exists())
    tid = T.begin({"run": "Bohr_20261123-224905_s01s02s03_v1_act", "ckpt": "checkpoints/best_model.pth", "note": ""})
    T.started(time.time()); T.feed("arm", [0.0] * 7); time.sleep(0.2)
    (new.parent / tid / "oakd_front_view.mp4").write_bytes(b"being recorded")
    check("the run being recorded is not sent", upload(data, dest) == 0 and not (dest / "eval" / "Bohr_20261123-224905_s01s02s03_v1_act").exists())
    T.finish("operator"); end = time.time() + 60
    while time.time() < end and (T.busy or (T.trials() and (T.trials()[0].get("detection") or {}).get("state") != "done")): time.sleep(0.2)
    saved = T.get(tid)
    check("the new run is saved with its folder", saved.get("folder") == f"Bohr_20261123-224905_s01s02s03_v1_act/best_model/{tid}" and len(T.trials()) == 4, str(saved)[:200])

    # upload
    check("upload of 4 runs succeeds", upload(data, dest) == 0)
    st = U.states(ev, T.trials())
    check("all 4 runs are uploaded and verified", sorted(st.values()) == ["uploaded"] * 4, str(st))
    check("same folders on the Training machine", (dest / "eval" / want.relative_to(ev) / "signals.h5").read_bytes() == (want / "signals.h5").read_bytes())
    check("trials.csv arrived, the status file and the unsaved folder did not", (dest / "eval/trials.csv").is_file() and not (dest / "eval" / U.STATUS_FILE).exists() and not (dest / "eval/20261125-120000").exists())
    import h5py, numpy as np
    pl = ev / "Avogadro_20261123-224214_s01s02s03_v1_act/best_model/20261125-115017/policy_log.h5"
    check("policy_log.h5 is written for the run with an inference log, and only for it", pl.is_file() and len(list(ev.glob("*/*/*/policy_log.h5"))) == 1)
    check("policy_log.h5 arrived on the Training machine", (dest / "eval" / pl.relative_to(ev)).read_bytes() == pl.read_bytes())
    with h5py.File(pl) as f, h5py.File(tmp / "ws" / logname) as src:
        check("images are left out, the other observations kept", sorted(f["observations"]) == ["qpos_hand", "qpos_hand_step"] and "wrist_view_images" in f.attrs["left_out_images"])
        check("actions of all steps, unchanged", f["actions/actions_arm"].shape == (40, 7) and all(np.array_equal(f["actions/actions_arm"][i], src[f"step_{i}/actions/actions_arm"][()]) for i in range(40)))
        check("predicted chunks only for the steps in which the model ran", list(f["dbg/step_raw_actions_arm_step"][:]) == list(range(0, 40, 5)) and np.array_equal(f["dbg/step_raw_actions_arm"][3], src["step_15/dbg/step_raw_actions_arm"][()]))
        shp = f["dbg/aggr_buffer_actions_arm_shape"][:]
        check("the growing ensembling buffer keeps its real shape per step", f["dbg/aggr_buffer_actions_arm"].shape == (40, 1, 8, 10, 7) and list(shp[0]) == [1, 1, 10, 7] and list(shp[39]) == [1, 8, 10, 7])
        check("buffer and mask values unchanged", all(np.array_equal(f[f"dbg/{k}"][i][:, :shp[i][1]], src[f"step_{i}/dbg/{k}"][()]) for i in (0, 7, 39) for k in ("aggr_buffer_actions_arm", "aggr_mask_actions_arm"))
              and bool(np.isnan(f["dbg/aggr_buffer_actions_arm"][0][:, 1:]).all()))
    check("the node's own log is untouched", (tmp / "ws" / logname).read_bytes() == log_bytes)
    time.sleep(0.05); T.update("20261125-114543", {"note": "corrected after the upload"})
    st = U.states(ev, T.trials())
    check("a note changed after the upload shows as changed", st["20261125-114543"] == "changed" and list(st.values()).count("uploaded") == 3, str(st))
    video = dest / "eval" / want.relative_to(ev) / "oakd_front_view.mp4"; before = video.stat().st_mtime_ns
    check("second upload succeeds", upload(data, dest) == 0)
    check("the changed trial.json was replaced on the Training machine", json.loads((video.parent / "trial.json").read_text())["note"] == "corrected after the upload"
          and "corrected after the upload" in (dest / "eval/trials.csv").read_text())
    check("the video there was not touched", video.stat().st_mtime_ns == before and U.states(ev, T.trials())["20261125-114543"] == "uploaded")
    video.write_bytes(video.read_bytes() + b"x")   # another content under the same name on the Training machine
    check("a different file on the Training machine fails the upload", upload(data, dest) == 1)
    st = U.states(ev, T.trials())
    check("that run is marked, the others stay uploaded", st["20261125-114543"] == "different" and list(st.values()).count("uploaded") == 3, str(st))
    check("and it was not overwritten", video.read_bytes().endswith(b"x"))
    check("nothing was deleted here", len(T.trials()) == 4 and (want / "oakd_front_view.mp4").is_file())
    shutil.rmtree(tmp)
    print("\n" + ("ALL PASSED" if not FAILED else f"{len(FAILED)} FAILED: " + "; ".join(FAILED)))
    return 1 if FAILED else 0

if __name__ == "__main__":
    sys.exit(main())
