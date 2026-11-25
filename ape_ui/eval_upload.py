#!/usr/bin/env python3
"""Copy the evaluated policy runs to the Training machine (run by APE's "Upload to the Training machine" button in
card 5; also usable by hand).

    eval_upload.py --data DATA --host <ssh alias>      copy <DATA>/eval/ to <dest-root>/eval/ and verify by checksum

Same destination settings as the demo archive (archive_job.py): --host (ssh target; for tests --local-dest instead),
--dest-root (data root there, default ~/rwr_task2/data), --rsync-path (rsync on the Training machine), --ssh.
The folders arrive as they are here: eval/<policy run>/<checkpoint>/<run id>/ (eval_trials.py).

What is sent, and how:
- only runs that are saved completely (their trial.json exists; it is written last), never the run being recorded;
- the videos and signals.h5 never change after a run, so they are sent with rsync --ignore-existing: a file that is
  already on the Training machine is not sent again and never overwritten. If it is there with other content, the
  run shows up as "different on the Training machine";
- trial.json (notes and corrected counts can change after a run) and trials.csv are replaced there whenever they
  differ: this PC is where runs are corrected, so its copy wins.
Nothing is deleted on either machine.

The videos are not compressed again: APE encodes them while the run is recorded, with the video settings of the
demo archive (eval_trials.VIDEO_ARGS), so there are no raw images to compress, unlike the demos (encode_archive.py).

policy_log.h5: before sending, each run gets a small copy of the inference node's log of that policy start
(trial.json "inference_log", a path under --log-root): the published actions, the debug data (the action chunk the
model predicted, raw and translated, and the temporal-ensembling buffer with its mask) and the non-image observations
of every step, gzip-compressed. The images the network was given (nearly all of the log's size) are left out.
The node's own log is neither changed nor deleted nor sent. A run whose log is gone is sent without policy_log.h5.
    eval_upload.py --data DATA --log-root WS --policy-logs-only     only write the missing policy_log.h5 files

Result per run: <DATA>/eval/.upload_status.json (APE shows it in the list of runs; never sent).
Needs rsync on both machines (3.2.3 or newer for --mkpath), ssh keys without a password prompt (BatchMode), and
h5py and numpy for policy_log.h5.
"""
import argparse
import json
import os
import re
import shlex
import subprocess
import sys
import tempfile
import time
from pathlib import Path

TRIAL_ID = re.compile(r"^\d{8}-\d{6}(_\d+)?$")   # as in eval_trials.py
STATUS_FILE = ".upload_status.json"
MUTABLE = ("trial.json",)   # replaced on the Training machine when changed here; every other file is sent once
DEFAULT_SSH = "ssh -o BatchMode=yes -o ConnectTimeout=15"


def now():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def command(data, settings, log_root=""):
    """The command line APE starts, from station_data.archive_settings(). log_root: where the inference node was
    launched (its logs are in <log_root>/inference_logs/)."""
    cmd = [sys.executable, "-u", str(Path(__file__).resolve()), "--data", str(data), "--dest-root", settings["data_root"]]
    if log_root:
        cmd += ["--log-root", str(log_root)]
    for flag, key in (("--host", "host"), ("--rsync-path", "remote_rsync"), ("--ssh", "ssh")):
        if settings.get(key):
            cmd += [flag, settings[key]]
    return cmd


def saved_runs(folder):
    """{run id: its folder relative to `folder`} of every completely saved run in the grouped layout."""
    out = {}
    for f in sorted(Path(folder).glob("*/*/*/trial.json")):
        if TRIAL_ID.match(f.parent.name):
            out.setdefault(f.parent.name, f.parent.relative_to(folder).as_posix())
    return out


def flat_runs(folder):
    """Run ids still in the old layout (<eval>/<run id>/): APE moves them when it starts."""
    return sorted(f.parent.name for f in Path(folder).glob("*/trial.json") if TRIAL_ID.match(f.parent.name))


def run_files(folder, rel):
    return sorted(f"{rel}/{f.name}" for f in (Path(folder) / rel).iterdir()
                  if f.is_file() and not f.name.startswith(".") and not f.name.endswith((".tmp", ".part")))


def read_status(folder):
    try:
        return json.loads((Path(folder) / STATUS_FILE).read_text())
    except Exception:
        return {}


def states(folder, trials):
    """{run id: "uploaded" | "changed" | "different" | "no"} for the page. "changed": trial.json was edited after
    the upload; "different": the last upload found another file under the same name on the Training machine."""
    runs = read_status(folder).get("runs", {})
    out = {}
    for t in trials:
        s = runs.get(t["id"])
        if not s:
            out[t["id"]] = "no"
            continue
        try:
            same = abs((Path(folder) / s["folder"] / "trial.json").stat().st_mtime - s.get("json_mtime", 0)) < 1e-3
        except OSError:
            same = False
        out[t["id"]] = ("uploaded" if same else "changed") if s.get("ok") else "different"
    return out


POLICY_LOG = "policy_log.h5"
LOG_GROUPS = ("actions", "dbg", "observations")
IMAGE_VALUES = 20000   # an observation with more values than this per step is an image: left out


def make_policy_log(source, target):
    """Write `target` from the inference node's log `source` (one group step_<i>/{actions,observations,dbg}/<key> per
    published policy step): the same keys, one array per key over all steps, without the image observations.
    <group>/<key>          (n, ...) the values of the n steps that have this key, in step order
    <group>/<key>_step     (n,)     their step numbers (dbg/step_raw_* exist only in steps in which the model ran)
    <group>/<key>_shape    (n, d)   only if the shape changes between steps (the ensembling buffer grows at the start):
                                    the real shape per step; the array is filled up with NaN (False for masks)
    Returns a short text for the job log."""
    import h5py
    import numpy as np

    source, target = Path(source), Path(target)
    series, skipped = {}, {}   # (group, key) -> [(step, array)]; image keys -> shape
    with h5py.File(source, "r") as f:
        steps = sorted(int(k[5:]) for k in f if k.startswith("step_") and k[5:].isdigit())
        for i in steps:
            for group in LOG_GROUPS:
                g = f.get(f"step_{i}/{group}")
                if g is None:
                    continue
                for key, d in g.items():
                    if not isinstance(d, h5py.Dataset):
                        continue
                    if group == "observations" and d.size > IMAGE_VALUES:
                        skipped[key] = list(d.shape)
                        continue
                    series.setdefault((group, key), []).append((i, d[()]))
    tmp = target.with_name(target.name + ".tmp")
    with h5py.File(tmp, "w") as out:
        out.attrs["source"] = str(source)
        out.attrs["steps"] = len(steps)
        out.attrs["left_out_images"] = json.dumps(skipped)
        out.attrs["about"] = ("From the inference node's log, without the images. One array per key over the steps that have it; "
                              "<key>_step = their step numbers; <key>_shape = real shape per step where it changes (filled with NaN/False). "
                              "A step is one published action (10 per second); the log has no times: signals.h5 policy_arm has one sample per published step.")
        for (group, key), rows in series.items():
            arrays = [np.asarray(a) for _, a in rows]
            shapes = {a.shape for a in arrays}
            if len(shapes) > 1:
                if len({a.ndim for a in arrays}) > 1:
                    continue   # not seen in the node's logs: nothing sensible to stack
                full = tuple(max(a.shape[k] for a in arrays) for k in range(arrays[0].ndim))
                fill = False if arrays[0].dtype == bool else np.nan
                dtype = arrays[0].dtype if arrays[0].dtype.kind in "fb" else np.float64
                data = np.full((len(arrays), *full), fill, dtype)
                for n, a in enumerate(arrays):
                    data[(n, *(slice(0, k) for k in a.shape))] = a
                out.create_dataset(f"{group}/{key}_shape", data=np.array([a.shape for a in arrays], np.int32))
            else:
                data = np.stack(arrays)
            if data.dtype == np.float64:
                data = data.astype(np.float32)   # the model works in float32
            out.create_dataset(f"{group}/{key}", data=data, compression="gzip", compression_opts=4, shuffle=True)
            out.create_dataset(f"{group}/{key}_step", data=np.array([i for i, _ in rows], np.int32))
    tmp.replace(target)
    return f"{len(steps)} steps, {target.stat().st_size / 1e6:.1f} MB (the node's log: {source.stat().st_size / 1e6:.0f} MB)"


def write_policy_logs(folder, runs, log_root):
    """policy_log.h5 for every run that has none yet and whose inference log is still on this PC."""
    made = missing = 0
    for tid, rel in runs.items():
        target = Path(folder) / rel / POLICY_LOG
        if target.is_file():
            continue
        try:
            name = json.loads((Path(folder) / rel / "trial.json").read_text()).get("inference_log") or ""
        except Exception:
            name = ""
        source = Path(name) if os.path.isabs(name) else Path(log_root).expanduser() / name
        if not name or not source.is_file():
            missing += 1
            continue
        try:
            print(f"{rel}/{POLICY_LOG}: " + make_policy_log(source, target), flush=True)
            made += 1
        except Exception as exc:   # e.g. the node still has the file open: the next upload tries again
            print(f"{rel}: {POLICY_LOG} not written ({type(exc).__name__}: {exc})", flush=True)
            target.with_name(target.name + ".tmp").unlink(missing_ok=True)
    if made or missing:
        print(f"{POLICY_LOG}: {made} written" + (f"; {missing} run(s) have no inference log on this PC and are sent without it" if missing else ""), flush=True)


class Upload:
    def __init__(self, args):
        self.args = args
        self.eval = Path(args.data).expanduser() / "eval"
        root = args.dest_root
        if args.host:   # rsync over ssh starts in the remote home directory: "~/x" and "x" are the same place
            root = root[2:] if root.startswith("~/") else root
            self.dest = f"{args.host}:{root.rstrip('/')}/eval"
        elif args.local_dest:   # tests only: a folder on this machine stands in for the Training machine
            self.dest = str(Path(args.local_dest).expanduser()).rstrip("/") + "/eval"
        else:
            self.dest = ""
        self.rsync = [args.rsync, "-a", "--mkpath"]
        if args.host:
            self.rsync += ["-e", args.ssh or DEFAULT_SSH]
        if args.rsync_path:
            self.rsync += ["--rsync-path", args.rsync_path]

    def _rsync(self, names, *options, capture=False):
        """rsync the listed files (paths relative to the eval folder) with the folders they are in."""
        with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
            f.write("\n".join(names) + "\n")
        try:
            cmd = self.rsync + [*options, f"--files-from={f.name}", f"{self.eval}/", f"{self.dest}/"]
            if not capture:
                print("$ " + shlex.join(cmd), flush=True)
            return subprocess.run(cmd, capture_output=capture, text=True)
        finally:
            os.unlink(f.name)

    def run(self):
        runs = saved_runs(self.eval)
        old = flat_runs(self.eval)
        if old:
            print(f"{len(old)} run(s) are still in the old folder layout and are not sent: restart APE (it moves them), "
                  f"or run eval_trials.py migrate {self.eval}", flush=True)
        if not runs:
            print("No evaluated run to send.", flush=True)
            return 1 if old else 0
        if self.args.log_root:
            print("== Actions and debug data from the inference logs ==", flush=True)
            write_policy_logs(self.eval, runs, self.args.log_root)
        files = {tid: run_files(self.eval, rel) for tid, rel in runs.items()}
        mtimes = {tid: (self.eval / rel / "trial.json").stat().st_mtime for tid, rel in runs.items()}   # before sending
        once = [n for names in files.values() for n in names if Path(n).name not in MUTABLE]
        again = [n for names in files.values() for n in names if Path(n).name in MUTABLE]
        if (self.eval / "trials.csv").is_file():
            again.append("trials.csv")
        code = 0
        print(f"== Sending {len(runs)} evaluated run(s) to {self.dest} ==", flush=True)
        for names, options, what in ((once, ["--ignore-existing", "--info=stats1,name1"], "videos and signals"),
                                     (again, ["--checksum", "--info=name1"], "trial.json files and trials.csv")):
            if names:
                res = self._rsync(names, *options)
                if res.returncode:
                    print(f"rsync of the {what} ended with code {res.returncode}", flush=True)
                    code = res.returncode
        # full checksum comparison of every file of every run with what is on the Training machine now
        res = self._rsync(once + again, "--checksum", "--dry-run", "--out-format=%i %n", capture=True)
        bad = None
        if res.returncode:
            print(f"Verification failed (rsync code {res.returncode}): {res.stderr.strip()[-300:]}", flush=True)
            code = code or res.returncode
        else:   # '<' = content would be sent (missing or different there); '.' = attributes only (same content)
            bad = {name for flags, _, name in (line.partition(" ") for line in res.stdout.splitlines())
                   if name and not name.endswith("/") and flags[:1] in "<>c" and flags[1:2] == "f"}
        status = read_status(self.eval)
        status.update(dest=self.dest, checked_at=now())
        ok_runs = 0
        for tid, rel in runs.items():
            if bad is None:
                continue   # could not compare: keep what was known before
            problems = sorted(Path(n).name for n in files[tid] if n in bad)
            status.setdefault("runs", {})[tid] = {"ok": not problems, "at": now(), "folder": rel, "json_mtime": mtimes[tid],
                                                  "problems": [f"{p} missing or different on the Training machine" for p in problems]}
            ok_runs += not problems
            if problems:
                print(f"NOT verified: {rel}: " + ", ".join(problems) + " missing or different on the Training machine", flush=True)
        tmp = self.eval / (STATUS_FILE + ".tmp")
        tmp.write_text(json.dumps(status, indent=1))
        tmp.replace(self.eval / STATUS_FILE)
        if bad is not None:
            print(f"Verified on the Training machine: {ok_runs} of {len(runs)} run(s).", flush=True)
            code = code or (0 if ok_runs == len(runs) else 1)
        return code or (1 if old else 0)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--data", required=True, help="local data root (holds eval/)")
    p.add_argument("--host", default=os.environ.get("APE_ARCHIVE_HOST", ""), help="ssh target of the Training machine")
    p.add_argument("--dest-root", default=os.environ.get("APE_ARCHIVE_DATA_ROOT", "~/rwr_task2/data"),
                   help="data root on the Training machine (default ~/rwr_task2/data)")
    p.add_argument("--local-dest", default="", help="tests only: copy to this local folder instead of a Training machine")
    p.add_argument("--ssh", default=os.environ.get("APE_ARCHIVE_SSH", ""), help=f"remote shell for rsync (default: {DEFAULT_SSH})")
    p.add_argument("--rsync", default=os.environ.get("APE_RSYNC", "rsync"), help="local rsync binary")
    p.add_argument("--rsync-path", default=os.environ.get("APE_REMOTE_RSYNC", ""),
                   help="rsync binary on the Training machine, e.g. ~/.local/rsync/rsync")
    p.add_argument("--log-root", default="", help="folder the policy launch was started in (holds inference_logs/): "
                   "each run gets policy_log.h5, the node's log without the images. Empty: no policy_log.h5 is made")
    p.add_argument("--policy-logs-only", action="store_true", help="write the missing policy_log.h5 files and send nothing")
    args = p.parse_args(argv)
    job = Upload(args)
    if args.policy_logs_only:
        if not args.log_root:
            raise SystemExit("--policy-logs-only needs --log-root")
        write_policy_logs(job.eval, saved_runs(job.eval), args.log_root)
        return 0
    if not job.dest:
        raise SystemExit("No Training machine is set (archive_host in static/local/station.json).")
    if not job.eval.is_dir():
        print("No evaluated run to send.", flush=True)
        return 0
    return job.run()


if __name__ == "__main__":
    sys.exit(main())
