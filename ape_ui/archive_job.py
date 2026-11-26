#!/usr/bin/env python3
"""Archive, transfer and clear recorded demos (run by APE's processing job; also usable by hand).

    archive_job.py transfer --data DATA --session S   encode new raw recordings into data/archive/S/
                                                       (MP4 per camera + HDF5, src/logger/encode_archive.py),
                                                       copy archive (and processed) files to the Training
                                                       machine with rsync --ignore-existing, verify by checksum
    archive_job.py verify   --data DATA --session S   only the checksum verification (and the HF markers)
    archive_job.py clear    --data DATA --session S   verify again, then delete the local raw and processed
                                                       files of every demo whose archive matches on the Training
                                                       machine and (by default) that is backed up on Hugging Face
                                                       (course rule: clear the Robot PC after each session). The
                                                       archive (about 2 MB per second of recording), <demo>.meta.json
                                                       and manifest.csv stay, so APE can still show the demo.
                                                       Demos marked keep=discard in manifest.csv lose their raw and
                                                       processed files too, copied or not.

Discarded demos (keep=discard in this PC's manifest.csv) are not encoded and not copied by `transfer`; one that was
copied before stays on the Training machine, where the merged manifest leaves it out of dataset versions.
`transfer --no-encode --demo STEM [--demo STEM ...]` copies and verifies only those demos (APE's background worker
runs it as soon as a demo is encoded); manifest.csv is always sent.

Destination: --host (ssh target, e.g. an alias from ~/.ssh/config; for tests --local-dest instead) and
--dest-root (data root there, default ~/rwr_task2/data). Files go to <dest-root>/archive/<session>/ and, unless
--no-processed, <dest-root>/processed/<session>_10hz/; manifest.csv goes to <dest-root>/manifest_inbox/robot_pc.csv
(always replaced; the Training machine merges it with `rwr_data.py merge-manifest`). Existing archive and
processed files there are never overwritten; a file that
already exists with other content shows up as "different on the Training machine" and blocks clearing.

Hugging Face backup: src/logger/hf_backup_archive.py on the Training machine uploads archive/<session>/ and writes
<demo>.hf.json next to the archive files, with the sha256 of every file as the Hub reports it. `clear` fetches these
markers and only deletes a demo whose marker lists exactly the local archive files (skip with --no-require-hf-backup).

After a transfer (with --after-transfer CMD, from "archive_after_transfer" in station.json; empty = off): `transfer`
starts CMD on the Training machine over ssh once verification is done, without waiting for it (15 s timeout). The
Training machine's ~/rwr_task2/after_transfer.sh detaches itself and runs merge-manifest, the Hugging Face backup and
the rebuild of the processed files there. If ssh fails it prints one line; the exit code of the transfer is not changed.

Status per demo: <status-dir>/<session>/<demo>.json (APE shows it in the demo list).
Plain Python (stdlib); needs rsync on both machines (3.2.3 or newer for --mkpath) and ssh keys without a
password prompt (BatchMode).
"""
import argparse
import csv
import fcntl
import hashlib
import json
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path

PROCESSED_SUFFIX = "_10hz"
LOGGER_DIR = Path(__file__).resolve().parent.parent / "src" / "logger"
EXCLUDES = ["--exclude=.*", "--exclude=*.part", "--exclude=*.tmp", "--exclude=*.hf.json"]
DEFAULT_SSH = "ssh -o BatchMode=yes -o ConnectTimeout=15"


def now():
    return time.strftime("%Y-%m-%d %H:%M:%S")


def sha256(path, block=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(block):
            h.update(chunk)
    return h.hexdigest()


def save_status(status_dir, stem, **parts):
    """Update parts of <status_dir>/<stem>.json. APE's background encoder and this job both write these files, so
    the read-modify-write runs under a lock."""
    status_dir = Path(status_dir)
    status_dir.mkdir(parents=True, exist_ok=True)
    with open(status_dir / ".lock", "a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            s = json.loads((status_dir / f"{stem}.json").read_text())
        except Exception:
            s = {}
        s.update(parts)
        s["updated_at"] = now()
        tmp = status_dir / f".{stem}.json.tmp"
        tmp.write_text(json.dumps(s, indent=1))
        tmp.replace(status_dir / f"{stem}.json")


def record_archive(status_dir, archive_dir, raw_dir, stem, failure=None):
    """Write the archive part of a demo's status from <archive_dir>/<stem>.archive.json (or `failure`)."""
    try:
        summary = json.loads((Path(archive_dir) / f"{stem}.archive.json").read_text())
    except Exception:
        summary = None
    if summary is None:
        if failure or (Path(raw_dir) / f"{stem}.h5").is_file():
            save_status(status_dir, stem, archive={"ok": False, "errors": [failure or "not archived (see the job log)"]})
        return
    save_status(status_dir, stem, archive={
        "ok": bool(summary.get("ok")), "errors": summary.get("errors", []),
        "bytes": summary.get("archive_bytes"), "source_bytes": summary.get("source_bytes"),
        "crf": next((a for i, a in enumerate(summary.get("video_args", [])) if i and summary["video_args"][i - 1] == "-crf"), None),
        "psnr_mean": {k: round(v.get("psnr_mean", 0), 2) for k, v in summary.get("cameras", {}).items()},
        "psnr_min": {k: round(v.get("psnr_min", 0), 2) for k, v in summary.get("cameras", {}).items()}})


class Job:
    def __init__(self, args):
        self.args = args
        self.data = Path(args.data).expanduser()
        self.session = args.session
        self.raw = self.data / "raw" / self.session
        self.processed = self.data / "processed" / (self.session + PROCESSED_SUFFIX)
        self.archive = self.data / "archive" / self.session
        self.status_dir = Path(args.status_dir).expanduser() / self.session
        self.status_dir.mkdir(parents=True, exist_ok=True)
        self.hf_dir = self.status_dir / "hf"   # copies of the Hugging Face markers from the Training machine
        self.with_processed = not args.no_processed
        self.only = set(args.demo or [])   # --demo: only these demos (transfer and verify)
        root = args.dest_root
        if args.host:
            # rsync over ssh starts in the remote home directory, so "~/x" and "x" are the same place;
            # a relative path avoids relying on tilde expansion through rsync's argument protection.
            root = root[2:] if root.startswith("~/") else root
            self.dest = f"{args.host}:{root.rstrip('/')}"
        elif args.local_dest:   # tests only: a folder on this machine stands in for the Training machine
            self.dest = str(Path(args.local_dest).expanduser()).rstrip("/")
        else:
            self.dest = ""
        self.rsync_base = [args.rsync, "-a"]
        if args.host:
            self.rsync_base += ["-e", args.ssh or DEFAULT_SSH]
        if args.rsync_path:
            self.rsync_base += ["--rsync-path", args.rsync_path]
        self.rsync = self.rsync_base + ["--mkpath", *EXCLUDES]

    # -- status files ---------------------------------------------------------------------
    def status(self, stem):
        try:
            return json.loads((self.status_dir / f"{stem}.json").read_text())
        except Exception:
            return {}

    def save(self, stem, **parts):
        save_status(self.status_dir, stem, **parts)

    # -- what belongs to a demo -----------------------------------------------------------
    def discarded(self):
        """Stems of this session marked keep=discard in this PC's manifest.csv."""
        try:
            with open(self.data / "manifest.csv", newline="") as f:
                return {Path(r.get("file", "")).stem for r in csv.DictReader(f)
                        if r.get("session") == self.session and (r.get("keep") or "").strip().lower() == "discard"}
        except FileNotFoundError:
            return set()

    def only_filter(self):
        """rsync filter rules limiting a transfer or comparison to the --demo stems (none = the whole session)."""
        if not self.only:
            return []
        return [f"--include=/{stem}.*" for stem in sorted(self.only)] + ["--exclude=*"]

    def stems(self):
        found = {f.stem for f in self.raw.glob("*.h5")} if self.raw.is_dir() else set()
        found |= {f.stem for f in self.processed.glob("*.h5")} if self.processed.is_dir() else set()
        found |= {f.name[: -len(".archive.json")] for f in self.archive.glob("*.archive.json")} if self.archive.is_dir() else set()
        return sorted(found)

    def archive_summary(self, stem):
        try:
            return json.loads((self.archive / f"{stem}.archive.json").read_text())
        except Exception:
            return None

    def archive_files(self, stem):
        """Names of the archive files of a demo (from its .archive.json), or None if it has no complete archive."""
        summary = self.archive_summary(stem)
        return None if summary is None else sorted(summary.get("files", {})) + [f"{stem}.archive.json"]

    def pairs(self):
        """(local folder, remote folder, kind) that are transferred."""
        out = [(self.archive, f"{self.dest}/archive/{self.session}", "archive")]
        if self.with_processed:
            out.append((self.processed, f"{self.dest}/processed/{self.session}{PROCESSED_SUFFIX}", "processed"))
        return out

    # -- steps ------------------------------------------------------------------------------
    def encode(self):
        if not self.raw.is_dir():
            print("No raw folder: nothing to encode.", flush=True)
            return 0
        dropped = self.discarded()
        files = [f for f in sorted(self.raw.glob("*.h5")) if f.stem not in dropped and (not self.only or f.stem in self.only)]
        if dropped & {f.stem for f in self.raw.glob("*.h5")}:
            print(f"Not encoded (discarded): {', '.join(sorted(dropped & {f.stem for f in self.raw.glob('*.h5')}))}", flush=True)
        if not files:
            print("Nothing to encode.", flush=True)
            return 0
        cmd = [sys.executable, "-u", str(LOGGER_DIR / "encode_archive.py"), *map(str, files), "-o", str(self.archive),
               "--skip-existing", "--jobs", str(self.args.jobs), "--threads", str(self.args.threads)]
        if self.args.ffmpeg:
            cmd += ["--ffmpeg", self.args.ffmpeg]
        print("$ " + shlex.join(cmd), flush=True)
        code = subprocess.run(["nice", "-n", "10", *cmd]).returncode
        for stem in self.stems():
            record_archive(self.status_dir, self.archive, self.raw, stem)
        return code

    def transfer(self):
        code = 0
        dropped = sorted(self.discarded())
        if dropped:
            print(f"Not copied (discarded): {', '.join(dropped)}", flush=True)
        for local, remote, kind in self.pairs():
            if not local.is_dir():
                continue
            skip = [f"--exclude=/{stem}.*" for stem in dropped]   # before the --demo rules: first match wins
            cmd = self.rsync + skip + self.only_filter() + ["--ignore-existing", "--info=stats1,name1", f"{local}/", f"{remote}/"]
            print("$ " + shlex.join(cmd), flush=True)
            res = subprocess.run(cmd)
            if res.returncode:
                print(f"rsync of the {kind} files ended with code {res.returncode}", flush=True)
                code = res.returncode
        return code

    def send_manifest(self):
        """Copy this PC's manifest.csv to <dest-root>/manifest_inbox/<name>.csv on the Training machine. It never
        touches the manifest there: `rwr_data.py merge-manifest` folds it in (the Training machine's values win,
        except keep: a demo discarded here is discarded there too)."""
        manifest = self.data / "manifest.csv"
        if not manifest.is_file():
            return 0
        cmd = self.rsync_base + ["--mkpath", "--checksum", str(manifest), f"{self.dest}/manifest_inbox/{self.args.inbox_name}.csv"]
        print("$ " + shlex.join(cmd), flush=True)
        code = subprocess.run(cmd).returncode
        if code:
            print(f"rsync of manifest.csv ended with code {code}", flush=True)
        return code

    def mismatches(self):
        """{local folder: set of file names that are missing or different on the Training machine}, by a full
        checksum comparison (rsync --checksum --dry-run). None for a folder when rsync failed."""
        out = {}
        for local, remote, kind in self.pairs():
            if not local.is_dir():
                out[local] = set()
                continue
            cmd = self.rsync + self.only_filter() + ["--checksum", "--dry-run", "--out-format=%i %n", f"{local}/", f"{remote}/"]
            res = subprocess.run(cmd, capture_output=True, text=True)
            if res.returncode:
                print(f"Verification of the {kind} files failed (rsync code {res.returncode}): {res.stderr.strip()[-300:]}", flush=True)
                out[local] = None
                continue
            bad = set()
            for line in res.stdout.splitlines():
                flags, _, name = line.partition(" ")
                # '<' or '>' = content would be sent (missing or different); '.' = attributes only (same content)
                if name and not name.endswith("/") and flags[:1] in "<>c" and flags[1:2] == "f":
                    bad.add(name)
            out[local] = bad
        return out

    def fetch_hf_markers(self):
        """Copy <demo>.hf.json from the Training machine's archive folder into the status folder."""
        self.hf_dir.mkdir(exist_ok=True)
        cmd = self.rsync_base + ["--include=*.hf.json", "--exclude=*", f"{self.dest}/archive/{self.session}/", f"{self.hf_dir}/"]
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode not in (0, 23):   # 23: the remote folder does not exist yet = no markers
            print(f"Could not fetch the Hugging Face markers (rsync code {res.returncode}): {res.stderr.strip()[-300:]}", flush=True)
            return False
        return True

    def hf_state(self, stem):
        """Is this demo's archive on Hugging Face with exactly the local content? (ok, text, marker)"""
        try:
            marker = json.loads((self.hf_dir / f"{stem}.hf.json").read_text())
        except Exception:
            return False, "not backed up on Hugging Face yet", None
        summary = self.archive_summary(stem)
        if summary is None:
            return False, "no local archive to compare with", marker
        want = {name: info["sha256"] for name, info in summary.get("files", {}).items()}
        own = self.archive / f"{stem}.archive.json"
        if own.is_file():
            want[own.name] = sha256(own)
        have = {name: (info or {}).get("sha256") for name, info in marker.get("files", {}).items()}
        missing = [n for n, h in want.items() if have.get(n) != h]
        if missing:
            return False, "Hugging Face copy differs or is incomplete: " + ", ".join(missing[:3]), marker
        return True, f"on Hugging Face ({marker.get('repo', '?')}, {marker.get('at', '')})", marker

    def trigger_after_transfer(self):
        """Start the Training machine's after-transfer command (detached there), never waiting for it to finish."""
        cmd = self.args.after_transfer.strip()
        if not cmd:
            return
        remote = f"{cmd} --session {shlex.quote(self.session)} </dev/null >/dev/null 2>&1"
        if self.args.host:
            argv = shlex.split(self.args.ssh or DEFAULT_SSH) + [self.args.host, remote]
        else:   # tests: --local-dest stands in for the Training machine, so the command runs here
            argv = ["bash", "-c", remote]
        try:
            res = subprocess.run(argv, capture_output=True, text=True, timeout=15)
            if res.returncode:
                print(f"After-transfer step on the Training machine not started (code {res.returncode}): "
                      f"{(res.stderr or res.stdout).strip()[-200:]}", flush=True)
            else:
                print(f"After-transfer step started on the Training machine: {cmd}", flush=True)
        except subprocess.TimeoutExpired:
            print(f"After-transfer step on the Training machine not started: no answer within 15 s ({cmd})", flush=True)
        except OSError as exc:
            print(f"After-transfer step on the Training machine not started: {exc}", flush=True)

    def verify(self):
        """Mark every demo verified or not, and refresh its Hugging Face state. Returns the verified stems."""
        mism = self.mismatches()
        have_markers = self.fetch_hf_markers()
        ok_stems = []
        stems = [s for s in self.stems() if not self.only or s in self.only]
        for stem in stems:
            problems, files = [], {}
            names = self.archive_files(stem)
            if names is None:
                problems.append("no complete archive")
            groups = [(self.archive, names or [])]
            if self.with_processed:
                groups.append((self.processed, [f"{stem}.h5"]))
            for folder, wanted in groups:
                for name in wanted:
                    path = folder / name
                    if not path.is_file():
                        problems.append(f"{folder.name}/{name} missing on this PC")
                        continue
                    bad = mism.get(folder)
                    if bad is None:
                        problems.append("could not compare with the Training machine")
                    elif name in bad:
                        problems.append(f"{folder.name}/{name} missing or different on the Training machine")
                    files[str(path)] = path.stat().st_size
            ok = not problems
            hf_ok, hf_text, _ = self.hf_state(stem) if have_markers else (False, "Hugging Face markers could not be fetched", None)
            self.save(stem, dest=self.dest, verified={"ok": ok, "at": now(), "problems": sorted(set(problems))[:10],
                                                      "files": files, "processed_included": self.with_processed},
                      hf={"ok": hf_ok, "text": hf_text, "checked_at": now()})
            if ok:
                ok_stems.append(stem)
        n_hf = sum(1 for s in ok_stems if self.status(s).get("hf", {}).get("ok"))
        print(f"Verified on the Training machine: {len(ok_stems)} of {len(stems)} demo(s); "
              f"{n_hf} of them backed up on Hugging Face.", flush=True)
        return ok_stems

    def clear(self):
        """Delete local files of demos whose every file matches on the Training machine right now (and, unless
        --no-require-hf-backup, whose archive is on Hugging Face with the same content)."""
        verified = self.verify()   # fresh checksum comparison and fresh markers, never an old result
        freed, cleared, kept = 0, 0, []
        for stem in verified:
            s = self.status(stem)
            if self.args.require_hf_backup and not s.get("hf", {}).get("ok"):
                kept.append(f"{stem}: {s.get('hf', {}).get('text', 'not backed up on Hugging Face')}")
                continue
            # The raw recording is covered by the archive; the processed file is either compared byte for byte
            # (default) or, with --no-processed, can be rebuilt from the archive with reconstruct_raw.py. The archive
            # itself stays on this PC (small; APE plays cleared demos from it), and so does <demo>.meta.json.
            files = [Path(p) for p in s["verified"]["files"] if Path(p).parent != self.archive]
            for extra in (self.raw / f"{stem}.h5", self.processed / f"{stem}.h5"):
                if extra.is_file() and extra not in files:
                    files.append(extra)
            removed = []
            for f in files:
                if f.name.endswith(".meta.json"):
                    continue   # the settings of the demo stay on this PC (APE's policy checks read them)
                if f.is_file():
                    freed += f.stat().st_size
                    f.unlink()
                    removed.append(str(f))
            self.save(stem, cleared={"at": now(), "files": removed})
            cleared += 1
            print(f"{stem}: removed {len(removed)} file(s)", flush=True)
        # Discarded demos: their raw and processed files go even if they never reached the Training machine (the
        # person discarded them in APE); the archive, if any, and <demo>.meta.json stay (undo is still possible).
        for stem in sorted(self.discarded() - set(verified)):
            removed = []
            for f in (self.raw / f"{stem}.h5", self.processed / f"{stem}.h5"):
                if f.is_file():
                    freed += f.stat().st_size
                    f.unlink()
                    removed.append(str(f))
            if removed:
                self.save(stem, cleared={"at": now(), "files": removed, "discarded": True})
                cleared += 1
                print(f"{stem}: discarded, removed {len(removed)} file(s) (not verified on the Training machine)", flush=True)
        if kept:
            print("Kept (verified on the Training machine, but not backed up on Hugging Face):\n  " + "\n  ".join(kept), flush=True)
        print(f"Cleared {cleared} demo(s), {freed / 1e9:.2f} GB freed.", flush=True)
        return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("action", choices=["transfer", "verify", "clear"])
    p.add_argument("--data", required=True, help="local data root (raw/, processed/, archive/)")
    p.add_argument("--session", required=True)
    p.add_argument("--status-dir", required=True, help="where the per-demo status files go")
    p.add_argument("--host", default=os.environ.get("APE_ARCHIVE_HOST", ""), help="ssh target of the Training machine")
    p.add_argument("--dest-root", default=os.environ.get("APE_ARCHIVE_DATA_ROOT", "~/rwr_task2/data"),
                   help="data root on the Training machine (default ~/rwr_task2/data)")
    p.add_argument("--local-dest", default="", help="tests only: copy to this local folder instead of a Training machine")
    p.add_argument("--ssh", default=os.environ.get("APE_ARCHIVE_SSH", ""), help=f"remote shell for rsync (default: {DEFAULT_SSH})")
    p.add_argument("--rsync", default=os.environ.get("APE_RSYNC", "rsync"), help="local rsync binary")
    p.add_argument("--rsync-path", default=os.environ.get("APE_REMOTE_RSYNC", ""),
                   help="rsync binary on the Training machine, e.g. ~/.local/rsync/rsync")
    p.add_argument("--no-processed", action="store_true", help="transfer and verify the archive only, not the processed files")
    p.add_argument("--no-encode", action="store_true", help="transfer: copy what is already encoded, encode nothing")
    p.add_argument("--demo", action="append", default=[], help="transfer, verify: only this demo (file stem); repeatable")
    p.add_argument("--no-require-hf-backup", dest="require_hf_backup", action="store_false",
                   help="clear demos that are verified on the Training machine even if not backed up on Hugging Face")
    p.add_argument("--after-transfer", default=os.environ.get("APE_ARCHIVE_AFTER_TRANSFER", ""),
                   help="command started on the Training machine after a transfer (detached), e.g. ~/rwr_task2/after_transfer.sh")
    p.add_argument("--inbox-name", default="robot_pc", help="manifest.csv is sent as manifest_inbox/<name>.csv")
    p.add_argument("--jobs", type=int, default=2, help="recordings encoded at the same time")
    p.add_argument("--threads", type=int, default=4, help="ffmpeg threads per recording")
    p.add_argument("--ffmpeg", default="")
    args = p.parse_args(argv)
    if not args.session or "/" in args.session or args.session.startswith("."):
        raise SystemExit("invalid session name")
    job = Job(args)
    if args.action == "transfer":
        code = 0 if args.no_encode else job.encode()
        if not job.dest:
            print("No Training machine is set (archive_host in static/local/station.json): archived only, nothing transferred.", flush=True)
            return code
        code = job.transfer() or code
        code = job.send_manifest() or code
        job.verify()
        job.trigger_after_transfer()   # never changes the exit code
        return code
    if not job.dest:
        raise SystemExit("No Training machine is set (archive_host in static/local/station.json).")
    if args.action == "verify":
        job.verify()
        return 0
    return job.clear()


if __name__ == "__main__":
    sys.exit(main())
