#!/usr/bin/env python3
"""Rebuild the 10 Hz processed files from the video archive (Training machine, archive-only transfer).

For every demo in <data>/archive/<session>/ whose archive is complete and verified (its .archive.json says ok
and every file it lists is there with the listed size and sha256) and that has no <data>/processed/<session>_10hz/<demo>.h5:

  1. reconstruct_raw.py: archive -> raw-format .h5 in <work>/<session>/<demo>/raw/ (images lzf; non-image data
     bit for bit, images decoded from the MP4s)
  2. syncronize_data.py <that folder> --cfg_file experiments/cfgs/logger/datacollection_right.yaml --compress
     (the same command APE runs on the Robot PC; its default rate is 10 Hz)
  3. adds the root attribute "rebuilt_from_video_archive" (JSON: sha256 of the .archive.json and of every archive
     file, the codec settings, the logger config's sha256, the rwr_system commit, the time)
  4. copies it next to the target as .<demo>.h5.rebuild.part, checks the copy's sha256, and hard-links it to
     <demo>.h5. The link fails if <demo>.h5 exists, so an existing processed file is never replaced (dataset
     versions hard-link them); the rebuilt file is made read-only
  5. deletes everything in <work>/<session>/<demo>/

Safe to re-run at any time, also after an instance restart: finished demos are skipped (their processed file
exists), half-done work only lives in /tmp (wiped by a restart) or as .<demo>.h5.rebuild.part (removed at the
start of the next run). Only one run at a time (a lock in <work>); a second run exits at once with code 0.

Sessions still transferred WITH processed files are left alone: if a session's processed folder holds files that
were not rebuilt here (they came from the Robot PC) and demos are missing, the Robot PC is probably still
copying them, and a rebuilt file would make its --ignore-existing copy skip and its verification fail. Such a
"mixed" session is listed and skipped; --include-mixed rebuilds it anyway.

    python src/logger/rebuild_processed.py                          # all sessions in ~/rwr_task2/data/archive
    python src/logger/rebuild_processed.py --session s05_2026-11-26_Clarence --dry-run
    python src/logger/rebuild_processed.py --data /tmp/test_data --work /tmp/rebuild_test

--jobs N rebuilds N demos at a time (default 1). One 3-minute demo peaks at about 6.2 GiB, so 2 jobs fit in a
16 GiB cap. Plain Python (h5py, numpy, scipy, opencv, yaml, ffmpeg); no ROS. Exit code 1 if any demo failed or has a damaged
archive, else 0. Demos whose archive is not complete yet (still arriving) are skipped quietly and retried next time.
"""
import argparse
import concurrent.futures
import datetime
import fcntl
import json
import multiprocessing
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(HERE))
from video_archive_common import find_ffmpeg, sha256  # noqa: E402

PROCESSED_SUFFIX = "_10hz"
ATTR = "rebuilt_from_video_archive"
PART = ".rebuild.part"
DEFAULT_CFG = REPO / "experiments" / "cfgs" / "logger" / "datacollection_right.yaml"


def log(msg):
    print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {msg}", flush=True)


def git_commit(repo):
    try:
        sha = subprocess.run(["git", "-C", str(repo), "rev-parse", "HEAD"], capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", str(repo), "status", "--porcelain", "--untracked-files=no"],
                               capture_output=True, text=True).stdout.strip()
        return sha + ("-dirty" if dirty else "")
    except Exception:
        return "unknown"


def archive_state(session_dir, stem):
    """('ok', summary, None) | ('incomplete', None, why) | ('bad', None, why). Incomplete = files still missing
    (e.g. the transfer is running); bad = a file is there but differs from the .archive.json, or the archive is not ok."""
    try:
        summary = json.loads((session_dir / f"{stem}.archive.json").read_text())
    except Exception as exc:
        return "incomplete", None, f".archive.json unreadable ({exc})"
    if not summary.get("ok"):
        return "bad", None, "archive marked as not ok: " + "; ".join(summary.get("errors", []))
    for name, info in summary.get("files", {}).items():
        p = session_dir / name
        if not p.is_file():
            return "incomplete", None, f"{name} not there yet"
        if p.stat().st_size != info["bytes"]:
            return "incomplete", None, f"{name} has {p.stat().st_size} of {info['bytes']} bytes"
    for name, info in summary.get("files", {}).items():
        if sha256(session_dir / name) != info["sha256"]:
            return "bad", None, f"{name}: sha256 differs from its .archive.json"
    if not any(n.endswith(".mp4") for n in summary.get("files", {})) or f"{stem}.h5" not in summary.get("files", {}):
        return "bad", None, "the .archive.json lists no .h5 or no video"
    return "ok", summary, None


def is_rebuilt(path):
    import h5py
    try:
        with h5py.File(path, "r") as f:
            return ATTR in f.attrs
    except Exception:
        return False


def rebuild_one(session_dir, stem, summary, target, work, cfg, ffmpeg, threads, python):
    from reconstruct_raw import reconstruct_file
    import h5py

    demo_work = work / session_dir.name / stem
    if demo_work.exists():
        shutil.rmtree(demo_work)
    raw_dir, out_dir = demo_work / "raw", demo_work / "out"
    out_dir.mkdir(parents=True)
    part = target.parent / f".{target.name}{PART}"
    try:
        t0 = time.time()
        reconstruct_file(session_dir / f"{stem}.h5", raw_dir, ffmpeg, threads=threads, image_compression="lzf")
        (raw_dir / f"{stem}.meta.json").unlink(missing_ok=True)   # only the .h5 is an input of syncronize_data.py
        t1 = time.time()
        # exactly APE's command: no --sampling_freq, so the default 10 Hz is used and the file gets the same
        # root attribute sampling_frequency (int64 10; passing "--sampling_freq 10" would store float64 10.0)
        cmd = [python, "-u", str(HERE / "syncronize_data.py"), str(raw_dir), "--cfg_file", str(cfg),
               "--compress", "--max_parallel", "1", "--output_folder", str(out_dir)]
        res = subprocess.run(cmd, capture_output=True, text=True)
        built = out_dir / f"{stem}.h5"
        if res.returncode or not built.is_file():
            raise RuntimeError(f"syncronize_data.py failed (code {res.returncode}): {(res.stderr or res.stdout).strip()[-500:]}")
        t2 = time.time()
        own = session_dir / f"{stem}.archive.json"
        mark = {"archive": f"archive/{session_dir.name}/{stem}", "archive_json_sha256": sha256(own),
                "files": {n: i["sha256"] for n, i in summary["files"].items()},
                "format_version": summary.get("format_version"), "video_args": summary.get("video_args"),
                "image_compression_raw": "lzf",
                "sync": {"script": "src/logger/syncronize_data.py", "cfg_file": cfg.name, "cfg_sha256": sha256(cfg),
                         "sampling_freq": 10, "compress": True},
                "rwr_system_commit": git_commit(REPO),
                "rebuilt_at": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")}
        with h5py.File(built, "r+") as f:
            f.attrs[ATTR] = json.dumps(mark, sort_keys=True)
        digest = sha256(built)
        target.parent.mkdir(parents=True, exist_ok=True)
        part.unlink(missing_ok=True)
        shutil.copyfile(built, part)
        with open(part, "rb") as f:
            os.fsync(f.fileno())
        if sha256(part) != digest:
            raise RuntimeError("the copy in the processed folder differs from the rebuilt file")
        os.chmod(part, 0o444)
        try:
            os.link(part, target)          # fails if the target exists: never replaces a processed file
        except FileExistsError:
            return "exists", "a processed file appeared meanwhile; left it as it is"
        return "ok", (f"{target.stat().st_size / 1e9:.2f} GB, reconstruct {t1 - t0:.0f} s, synchronise {t2 - t1:.0f} s, "
                      f"copy {time.time() - t2:.0f} s")
    finally:
        part.unlink(missing_ok=True)
        shutil.rmtree(demo_work, ignore_errors=True)
        try:
            demo_work.parent.rmdir()   # the session folder, once empty
        except OSError:
            pass


def _job(a):
    sd, st = a[0], a[1]
    try:
        state, text = rebuild_one(*a)
    except Exception as exc:
        state, text = "failed", str(exc)
    return f"{sd.name}/{st}", state, text


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--data", default=os.environ.get("RWR_DATA_DIR", "~/rwr_task2/data"), help="data root (default ~/rwr_task2/data)")
    p.add_argument("--session", action="append", help="only these sessions (repeatable); default all in <data>/archive")
    p.add_argument("--work", default="/tmp/rwr_rebuild", help="scratch folder for the reconstructed raw files (default /tmp/rwr_rebuild)")
    p.add_argument("--cfg-file", default=str(DEFAULT_CFG), help="logger config for syncronize_data.py (default datacollection_right.yaml)")
    p.add_argument("--include-mixed", action="store_true", help="also rebuild into sessions that hold processed files from the Robot PC")
    p.add_argument("--jobs", type=int, default=1, help="demos rebuilt at the same time (default 1; a 3-minute demo needs about 6.2 GiB)")
    p.add_argument("--limit", type=int, default=0, help="rebuild at most this many demos in this run (0 = all)")
    p.add_argument("--threads", type=int, default=2, help="ffmpeg decoding threads (default 2)")
    p.add_argument("--ffmpeg")
    p.add_argument("--dry-run", action="store_true", help="only list what would be rebuilt")
    args = p.parse_args(argv)

    data = Path(args.data).expanduser().resolve()
    archive, processed, work = data / "archive", data / "processed", Path(args.work).expanduser()
    cfg = Path(args.cfg_file).expanduser().resolve()
    if not cfg.is_file():
        raise SystemExit(f"logger config not found: {cfg}")
    if not archive.is_dir():
        log(f"No archive folder in {data}: nothing to do.")
        return 0
    work.mkdir(parents=True, exist_ok=True)
    lock = open(work / "rebuild.lock", "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        log("Another rebuild_processed.py is running: nothing to do.")
        return 0

    sessions = args.session or sorted(d.name for d in archive.iterdir() if d.is_dir() and not d.name.startswith("."))
    todo, bad, waiting, mixed, have = [], [], [], [], 0
    for s in sessions:
        sd, pd = archive / s, processed / (s + PROCESSED_SUFFIX)
        if not sd.is_dir():
            bad.append(f"{s}: no archive folder")
            continue
        if pd.is_dir() and not args.dry_run:
            for stale in pd.glob(f".*{PART}"):    # left by a run that was killed between copy and link
                stale.unlink(missing_ok=True)
        stems = sorted(f.name[: -len(".archive.json")] for f in sd.glob("*.archive.json"))
        missing = [st for st in stems if not (pd / f"{st}.h5").exists()]
        have += len(stems) - len(missing)
        if not missing:
            continue
        from_robot = [f.name for f in sorted(pd.glob("*.h5"))] if pd.is_dir() else []
        from_robot = [n for n in from_robot if not is_rebuilt(pd / n)]
        if from_robot and not args.include_mixed:
            mixed.append(f"{s}: {len(missing)} demo(s) without processed file, but {len(from_robot)} processed file(s) "
                         "came from the Robot PC (still transferring with processed files?) - skipped, --include-mixed to force")
            continue
        for st in missing:
            state, summary, why = archive_state(sd, st)
            if state == "ok":
                todo.append((sd, st, summary, pd / f"{st}.h5"))
            elif state == "incomplete":
                waiting.append(f"{s}/{st}: {why}")
            else:
                bad.append(f"{s}/{st}: {why}")
    log(f"{data}: {len(todo)} demo(s) to rebuild, {have} already processed, {len(waiting)} archive(s) not complete yet, "
        f"{len(bad)} damaged, {len(mixed)} session(s) skipped as mixed")
    for line in mixed:
        log("  SKIPPED " + line)
    for line in waiting:
        log("  WAITING " + line)
    for line in bad:
        log("  DAMAGED " + line)
    if args.limit:
        todo = todo[: args.limit]
    if args.dry_run:
        for sd, st, _, target in todo:
            log(f"  would rebuild {target.relative_to(data)}")
        return 1 if bad else 0

    ffmpeg = find_ffmpeg(args.ffmpeg)
    failed, jobs = 0, max(1, min(args.jobs, len(todo)))
    if jobs > 1:
        log(f"Rebuilding {jobs} demos at a time")
    jobs_args = [(sd, st, summary, target, work, cfg, ffmpeg, args.threads, sys.executable) for sd, st, summary, target in todo]
    # separate processes (spawn: nothing of this process' HDF5 state is inherited); each demo also runs
    # syncronize_data.py as its own subprocess
    ctx = multiprocessing.get_context("spawn")
    with concurrent.futures.ProcessPoolExecutor(max_workers=jobs, mp_context=ctx) as ex:
        for i, (name, state, text) in enumerate(ex.map(_job, jobs_args) if jobs > 1 else map(_job, jobs_args), 1):
            failed += state == "failed"
            log(f"[{i}/{len(todo)}] {name}: {state}: {text}")
    log(f"Done: {len(todo) - failed} of {len(todo)} rebuilt or already there, {failed} failed.")
    return 1 if failed or bad else 0


if __name__ == "__main__":
    sys.exit(main())
