"""Recorded data, hand configuration and hand calibration helpers for APE UI."""
import csv
import importlib.util
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,80}$")
PROCESSED_SUFFIX = "_10hz"  # data/processed/<session>_10hz, same as rwr_il/scripts/rwr_data.py
CAMERAS = ("oakd_front_view", "oakd_wrist_view")
FINGERS = ("thumb", "index", "middle", "ring", "pinky")
# How to turn a camera image for DISPLAY only (live view and demo player). The recorded
# data is never changed. "ccw" = 90 degrees anticlockwise, "cw" = 90 degrees clockwise, "180".
DISPLAY_ROTATION = {"oakd_wrist_view": "ccw"}


def rotate_for_display(cam, img):
    import cv2

    code = {"ccw": cv2.ROTATE_90_COUNTERCLOCKWISE, "cw": cv2.ROTATE_90_CLOCKWISE, "180": cv2.ROTATE_180}.get(DISPLAY_ROTATION.get(cam))
    return img if code is None else cv2.rotate(img, code)


HERE = Path(__file__).resolve().parent
STATION_FILE = HERE / "static" / "local" / "station.json"   # git-ignored, station-specific (also holds robot_ip)
ARCHIVE_JOB = HERE / "archive_job.py"


def archive_settings():
    """Where and how demos are copied to the Training machine. Set in static/local/station.json, e.g.
    {"archive_host": "<ssh alias>", "archive_data_root": "~/rwr_task2/data",
     "archive_remote_rsync": "~/.local/rsync/rsync",
     "archive_require_hf_backup": true, "archive_after_transfer": "~/rwr_task2/after_transfer.sh"}
    or with $APE_ARCHIVE_HOST, $APE_ARCHIVE_DATA_ROOT, $APE_REMOTE_RSYNC, $APE_ARCHIVE_TRANSFER_PROCESSED,
    $APE_ARCHIVE_REQUIRE_HF_BACKUP (0/1), $APE_ARCHIVE_AFTER_TRANSFER, $APE_ARCHIVE_SSH (remote shell, rarely needed).
    archive_after_transfer is empty by default (off): nothing is started on the Training machine.
    archive_transfer_processed is still read but no longer used: since 2026-11-26 APE transfers the archive only."""
    try:
        cfg = json.loads(STATION_FILE.read_text())
    except Exception:
        cfg = {}
    pick = lambda env, key, default="": str(os.environ.get(env) or cfg.get(key) or default).strip()

    def flag(env, key, default=True):
        value = os.environ.get(env, cfg.get(key, default))
        return value if isinstance(value, bool) else str(value).strip().lower() not in ("0", "false", "no", "off", "")

    return {"host": pick("APE_ARCHIVE_HOST", "archive_host"),
            "data_root": pick("APE_ARCHIVE_DATA_ROOT", "archive_data_root", "~/rwr_task2/data"),
            "remote_rsync": pick("APE_REMOTE_RSYNC", "archive_remote_rsync"),
            "ssh": pick("APE_ARCHIVE_SSH", "archive_ssh"),
            "after_transfer": pick("APE_ARCHIVE_AFTER_TRANSFER", "archive_after_transfer"),
            "transfer_processed": flag("APE_ARCHIVE_TRANSFER_PROCESSED", "archive_transfer_processed"),
            "require_hf_backup": flag("APE_ARCHIVE_REQUIRE_HF_BACKUP", "archive_require_hf_backup")}


class DataError(Exception):
    pass


def safe(name):
    if not SAFE_NAME.match(name or "") or ".." in name:
        raise DataError("Invalid name.")
    return name


def data_tool():
    """rwr_il/scripts/rwr_data.py: keeps manifest.csv, builds dataset versions, uploads to Hugging Face."""
    if os.environ.get("RWR_DATA_TOOL"):
        return Path(os.environ["RWR_DATA_TOOL"])
    try:
        spec = importlib.util.find_spec("srl_il")  # rwr_il is installed editable in the workspace environment
        folder = Path(spec.origin).parent if spec and spec.origin else Path(list(spec.submodule_search_locations)[0])
        tool = folder.resolve().parent / "scripts" / "rwr_data.py"
        if tool.is_file():
            return tool
    except Exception:
        pass
    return Path.home() / "Projects" / "rwr_il" / "scripts" / "rwr_data.py"


class Station:
    """Recorded data lives under `data`: raw/<session>/, processed/<session>_10hz/, manifest.csv."""

    def __init__(self, workspace: Path, data: Path):
        self.workspace = workspace
        self.data = data
        self.raw = data / "raw"
        self.processed = data / "processed"
        self.tool = data_tool()
        self.previews = workspace / "work" / "ape_previews"
        self.archive_status = workspace / "work" / "archive_status"   # one JSON per demo, written by archive_job.py
        self.hand_cfg_dir = workspace / "install/hand_control/share/hand_control/orca/configs/orcahand_v2/right"
        self.calib_backups = workspace / "hand_calibration_backups"
        self.logger_cfg = workspace / "src/rwr_system/experiments/cfgs/logger/datacollection_right.yaml"
        self.inspect_script = workspace / "src/rwr_system/experiments/scripts/inspect_demonstration.py"
        self._h5 = None  # (path, handle) of the demo open in the player
        self._h5_lock = threading.Lock()

    # -- hand ------------------------------------------------------------------
    def hand_config(self):
        """Joint names, ranges of motion (degrees) and which joints make up each finger."""
        import yaml

        try:
            cfg = yaml.safe_load((self.hand_cfg_dir / "config.yaml").read_text())
            joints = list(cfg["joint_ids"])
            roms = {j: [float(v) for v in cfg["joint_roms"][j]] for j in joints}
        except Exception:
            return {"joints": [], "roms": {}, "fingers": {}}
        fingers = {}
        for f in FINGERS:
            # flexion joints only: abduction (side to side) and the thumb base say little about open/closed
            fingers[f] = [i for i, j in enumerate(joints) if j.startswith(f + "_") and not j.endswith(("_abd", "_cmc"))]
        return {"joints": joints, "roms": roms, "fingers": fingers}

    def calibration(self):
        f = self.hand_cfg_dir / "calibration.yaml"
        if not f.exists():
            return {"exists": False, "written": None}
        return {"exists": True, "written": f.stat().st_mtime}

    def recalibrate_at_next_start(self):
        """Move the calibration file away, so the hand calibrates itself at the next launch."""
        f = self.hand_cfg_dir / "calibration.yaml"
        if not f.exists():
            raise DataError("There is no calibration file: the hand will calibrate at the next start anyway.")
        self.calib_backups.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(f.stat().st_mtime))
        target = self.calib_backups / f"calibration_{stamp}.yaml"
        shutil.move(str(f), str(target))
        return str(target)

    # -- datasets ------------------------------------------------------------------
    def _summary(self, dataset, demo):
        f = self.previews / dataset / demo / "summary.json"
        if not f.exists():
            return None
        try:
            d = json.loads(f.read_text())
            streams = d.get("streams", {})
            # frames of the workspace camera (raw check) or of the synchronized file; else the first stream
            first = streams.get("/oakd_front_view/color") or next(iter(streams.values()), {})
            return {
                "ok": bool(d.get("ok")),
                "errors": [str(e) for e in d.get("errors", [])][:8],
                "duration_s": d.get("overlap_s", first.get("duration_s")),
                "frames": first.get("samples"),
                "max_gap_s": max((s.get("max_gap_s", 0) for s in streams.values()), default=None),
            }
        except Exception:
            return {"ok": False, "errors": ["The check result could not be read."], "duration_s": None, "frames": None, "max_gap_s": None}

    def _manifest(self):
        """{(session, file stem): manifest row}."""
        try:
            with open(self.data / "manifest.csv", newline="") as f:
                return {(r.get("session", ""), Path(r.get("file", "")).stem): r for r in csv.DictReader(f)}
        except Exception:
            return {}

    def register(self, session, labels):
        """Add the session's new recordings to manifest.csv with the labels chosen in the page.
        Returns an error text, or "" when it worked."""
        cmd = [sys.executable, str(self.tool), "--data", str(self.data), "scan", "--session", safe(session)]
        for flag, key in (("--colour", "colour"), ("--arena", "arena"), ("--cell", "cell"), ("--orientation", "orientation"), ("--lighting", "lighting")):
            value = str(labels.get(key) or "").strip()[:40]
            if value:
                cmd += [flag, value]
        description = " ".join(str(labels.get("description") or "").split())[:200]
        if description:
            cmd.append("--description=" + description)   # "=" form: the text may start with a dash
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        except Exception as exc:
            return f"manifest.csv was not updated: {exc}"
        return "" if res.returncode == 0 else "manifest.csv was not updated: " + (res.stderr.strip() or res.stdout.strip())[-300:]

    def write_meta(self, session, filename, labels, extra):
        """Write raw/<session>/<demo>.meta.json: the settings in use when this demo was saved.
        Returns an error text, or "" when it worked."""
        def text(path):
            try:
                return Path(path).read_text()
            except Exception:
                return None

        def commit():
            try:
                repo = self.workspace / "src/rwr_system"
                head = subprocess.run(["git", "-C", str(repo), "rev-parse", "--short", "HEAD"], capture_output=True, text=True, timeout=10).stdout.strip()
                dirty = subprocess.run(["git", "-C", str(repo), "status", "--porcelain", "--untracked-files=no"], capture_output=True, text=True, timeout=10).stdout.strip()
                return head + ("+local-changes" if head and dirty else "")
            except Exception:
                return ""

        try:
            stem = safe(Path(filename).stem)
            retarget = self.workspace / "install/experiments/share/experiments/cfgs/retargeter"
            meta = {
                "session": safe(session), "file": stem + ".h5", "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "labels": {name: str(labels.get(k) or "").strip()[:40] for name, k in (("colour", "colour"), ("arena_pose", "arena"), ("cube_cell", "cell"), ("cube_orientation", "orientation"), ("lighting", "lighting"))}
                          | {"description": " ".join(str(labels.get("description") or "").split())[:200]},
                **(extra or {}),
                "rwr_system_commit": commit(),
                "hand_config_yaml": text(self.hand_cfg_dir / "config.yaml"),
                "hand_calibration_yaml": text(self.hand_cfg_dir / "calibration.yaml"),
                "retargeter_cfgs_yaml": text(retarget / "retargeter_cfgs_orcahand_v2.yaml"),
                "mano_adjustment_yaml": text(retarget / "mano_adjustment_orcahand_v2.yaml"),
            }
            target = self.raw / meta["session"] / (stem + ".meta.json")
            tmp = target.with_suffix(".tmp")
            tmp.write_text(json.dumps(meta, indent=1))
            tmp.replace(target)
            return ""
        except Exception as exc:
            return f"The settings file of this demo was not written: {exc}"

    def _archive_job_args(self, action, dataset):
        a = archive_settings()
        cmd = [sys.executable, "-u", str(ARCHIVE_JOB), action, "--data", str(self.data), "--session", safe(dataset),
               "--status-dir", str(self.archive_status), "--dest-root", a["data_root"]]
        if a["host"]:
            cmd += ["--host", a["host"]]
        if a["remote_rsync"]:
            cmd += ["--rsync-path", a["remote_rsync"]]
        if a["ssh"]:
            cmd += ["--ssh", a["ssh"]]
        cmd.append("--no-processed")   # archive only: the Training machine rebuilds the processed files
        if action == "transfer" and a["after_transfer"]:
            cmd += ["--after-transfer", a["after_transfer"]]
        if action == "clear" and not a["require_hf_backup"]:
            cmd.append("--no-require-hf-backup")
        return cmd

    def transfer_command(self, dataset):
        """Encode what is still missing, copy the archives and manifest.csv, verify, start the after-transfer step."""
        safe(dataset)
        return self._archive_job_args("transfer", dataset)

    def clear_command(self, dataset):
        """Delete the local files of demos that are verified on the Training machine (re-verified first)."""
        if not archive_settings()["host"]:
            raise DataError("No Training machine is set: add archive_host to static/local/station.json.")
        return self._archive_job_args("clear", dataset)

    def archive_state(self, dataset, demo):
        """Short archive / transfer state of one demo for the demo list."""
        try:
            s = json.loads((self.archive_status / dataset / f"{demo}.json").read_text())
        except Exception:
            return None
        a, v, c, hf = s.get("archive") or {}, s.get("verified") or {}, s.get("cleared"), s.get("hf") or {}
        if c:
            state, text = "cleared", f"on the Training machine, cleared here {c.get('at', '')}"
        elif v.get("ok"):
            state, text = "verified", f"copied and verified {v.get('at', '')}"
        elif a and not a.get("ok", True):   # the cause, rather than "not verified" (its consequence)
            state, text = "archive_failed", "; ".join(a.get("errors") or []) or "archiving failed"
        elif v:
            state, text = "not_verified", "; ".join(v.get("problems") or []) or "not verified"
        elif a:
            state, text = "archived", "archived, not copied yet"
        else:
            return None
        if v.get("ok") and not c:
            text += " · " + (hf.get("text") or "Hugging Face not checked yet")
        if a.get("ok") and a.get("psnr_mean"):
            text += " · PSNR " + ", ".join(f"{k.replace('oakd_', '').replace('_view', '')} {v:.1f} dB" for k, v in a["psnr_mean"].items())
        return {"state": state, "text": text, "hf": bool(hf.get("ok")),
                "archive_mb": round((a.get("bytes") or 0) / 1e6, 1) if a.get("bytes") else None}

    def hf_repo(self):
        try:
            return (self.data / "hf_repo.txt").read_text().strip()
        except Exception:
            return os.environ.get("RWR_HF_REPO", "")

    def list_datasets(self, busy=None):
        """Sessions with their demos for card 3. busy: the background worker's info() (what it is doing now)."""
        from background import progress_of

        busy = busy or {}
        out = []
        names = set()
        for folder in (self.raw, self.data / "archive", self.archive_status):
            if folder.is_dir():
                names |= {d.name for d in folder.iterdir() if d.is_dir()}
        if self.processed.is_dir():  # processed folders of older sessions
            names |= {d.name[: -len(PROCESSED_SUFFIX)] for d in self.processed.iterdir() if d.is_dir() and d.name.endswith(PROCESSED_SUFFIX)}
        manifest = self._manifest()
        prefix_of = lambda session: (re.match(r"^(s\d+)_\d{4}-\d{2}-\d{2}_.+$", session) or [None, session])[1]
        for name in sorted(names):
            raw_dir, proc_dir, arc_dir = self.raw / name, self.processed / (name + PROCESSED_SUFFIX), self.data / "archive" / name
            raw = {f.stem: f for f in raw_dir.glob("*.h5")} if raw_dir.is_dir() else {}
            proc = {f.stem: f for f in proc_dir.glob("*.h5")} if proc_dir.is_dir() else {}
            arc = {f.name[: -len(".archive.json")] for f in arc_dir.glob("*.archive.json")} if arc_dir.is_dir() else set()
            status_dir = self.archive_status / name
            known = {f.stem for f in status_dir.glob("*.json")} if status_dir.is_dir() else set()
            stems = set(raw) | set(proc) | arc | known
            if not stems:
                continue
            # Name in dataset versions, as rwr_data.py builds it: <session prefix>_<colour>_<position in the session>
            listed = sorted({stem for (ses, stem) in manifest if ses == name} | stems)
            number = {stem: i for i, stem in enumerate(listed, start=1)}
            demos = []
            for stem in sorted(stems):
                row = manifest.get((name, stem), {})
                state = self.archive_state(name, stem)
                prog = progress_of(arc_dir, stem)
                if prog:
                    work = {"task": "encode", "pct": round(100 * prog.get("done", 0) / max(1, prog.get("total", 1))), "stage": prog.get("stage", "")}
                elif busy.get("session") == name and busy.get("demo") == stem:
                    work = {"task": busy.get("task"), "pct": None, "stage": ""}
                elif stem in raw and stem not in arc and not (state and state["state"] == "archive_failed"):
                    work = {"task": "wait_encode", "pct": 0, "stage": ""}
                else:
                    work = None
                demos.append({
                    "dataset_name": f"{prefix_of(name)}_{row.get('colour') or 'nocolour'}_{number[stem]:04d}.h5",
                    "description": row.get("description", ""),
                    "name": stem,
                    "raw_gb": round(raw[stem].stat().st_size / 1e9, 2) if stem in raw else None,
                    "processed_gb": round(proc[stem].stat().st_size / 1e9, 2) if stem in proc else None,
                    "check": self._summary(name, stem),
                    "colour": row.get("colour", ""),
                    "keep": row.get("keep", ""),
                    "archive": state,
                    "work": work,
                    "viewable": stem in raw or self._archive_complete(arc_dir, stem),
                    "stem_archived": stem in arc,
                })
            local_gb = lambda d: (d["raw_gb"] or 0) + (d["processed_gb"] or 0) + (((d["archive"] or {}).get("archive_mb") or 0) / 1000 if d["stem_archived"] else 0)
            freed_gb = lambda d: (d["raw_gb"] or 0) + (d["processed_gb"] or 0)   # Clear keeps the archive
            verified = [d for d in demos if d["archive"] and d["archive"]["state"] == "verified"]
            out.append({
                "name": name,
                "demos": demos,
                "total_gb": round(sum(local_gb(d) for d in demos), 1),
                "to_encode": sum(1 for d in demos if d["work"] and d["work"]["task"] in ("encode", "wait_encode")),
                "to_check": sum(1 for d in demos if d["raw_gb"] is not None and d["check"] is None),
                # verified on the Training machine; the clear job checks the Hugging Face backup again itself
                "clearable": len(verified),
                "clearable_hf": sum(1 for d in verified if d["archive"]["hf"]),
                "clearable_gb": round(sum(freed_gb(d) for d in verified), 1),
                "next_number": len(listed) + 1,
            })
        return out

    @staticmethod
    def _archive_complete(arc_dir, stem):
        try:
            s = json.loads((arc_dir / f"{stem}.archive.json").read_text())
            return all((arc_dir / n).is_file() for n in s.get("files", {}))
        except Exception:
            return False

    # -- player ------------------------------------------------------------------
    # A demo is shown from its raw file while it is on this PC (exact images, available the moment it is saved), and
    # from its video archive once the raw file is cleared (Clear keeps the archive). The timeline is the workspace
    # camera's frames.
    def _open(self, dataset, demo):
        raw = self.raw / safe(dataset) / (safe(demo) + ".h5")
        arc_dir = self.data / "archive" / dataset
        if raw.is_file():
            key = ("raw", raw)
        elif self._archive_complete(arc_dir, demo):
            key = ("archive", arc_dir / f"{demo}.archive.json")
        else:
            raise DataError("Neither the raw recording nor its archive is on this PC.")
        if self._h5 is None or self._h5[0] != key:
            if self._h5 is not None:
                self._h5[1].close()
            self._h5 = (key, RawSource(raw) if key[0] == "raw" else ArchiveSource(arc_dir, demo))
        return self._h5[1]

    def demo_meta(self, dataset, demo):
        with self._h5_lock:
            src = self._open(dataset, demo)
            return {"frames": src.frames, "freq": src.freq, "cameras": list(src.cameras), "source": src.kind}

    def demo_frame(self, dataset, demo, index, width=480):
        """One JPEG with both camera views of frame `index` side by side."""
        import cv2

        with self._h5_lock:
            src = self._open(dataset, demo)
            tiles = []
            for cam, img in src.frame(int(index)):
                img = rotate_for_display(cam, img)
                height = 270  # same height for every view, so turned (portrait) views fit next to landscape ones
                tiles.append(cv2.resize(img, (int(round(img.shape[1] * height / img.shape[0])), height), interpolation=cv2.INTER_AREA))
        if not tiles:
            raise DataError("This demo contains no camera images.")
        frame = cv2.cvtColor(np.hstack(tiles), cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            raise DataError("Could not encode the frame.")
        return buf.tobytes()


def _timeline(stamps_by_cam):
    """Frame count, rate and, per camera, the index of the image nearest to each frame of the first camera."""
    cams = [c for c in CAMERAS if c in stamps_by_cam and len(stamps_by_cam[c])]
    if not cams:
        return 0, 10.0, cams, {}
    base = np.asarray(stamps_by_cam[cams[0]], dtype=np.int64)
    freq = float(1e9 / np.median(np.diff(base))) if len(base) > 1 else 10.0
    nearest = {}
    for c in cams:
        t = np.asarray(stamps_by_cam[c], dtype=np.int64)
        j = np.clip(np.searchsorted(t, base), 1, max(1, len(t) - 1))
        left = t[j - 1] if len(t) > 1 else t[0]
        right = t[np.minimum(j, len(t) - 1)]
        nearest[c] = np.where(np.abs(base - left) <= np.abs(right - base), j - 1, np.minimum(j, len(t) - 1)) if len(t) > 1 else np.zeros(len(base), int)
    return len(base), round(freq, 2), cams, nearest


class RawSource:
    """Frames straight from a raw logger file: /<cam>/color/<receive time ns> datasets."""
    kind = "raw"

    def __init__(self, path):
        import h5py

        self.h5 = h5py.File(path, "r")
        self.names = {}
        for cam in CAMERAS:
            g = self.h5.get(f"/{cam}/color")
            if g is not None:
                self.names[cam] = sorted(g.keys(), key=int)
        self.frames, self.freq, self.cameras, self.nearest = _timeline({c: [int(n) for n in v] for c, v in self.names.items()})

    def frame(self, i):
        i = max(0, min(i, self.frames - 1))
        return [(c, self.h5[f"/{c}/color/{self.names[c][self.nearest[c][i]]}"][()]) for c in self.cameras]

    def close(self):
        self.h5.close()


class ArchiveSource:
    """Frames decoded from a demo's video archive (<demo>.<cam>.mp4, frame times in <demo>.h5)."""
    kind = "archive"

    def __init__(self, arc_dir, stem):
        import cv2
        import h5py

        stamps, self.caps, self.pos = {}, {}, {}
        with h5py.File(arc_dir / f"{stem}.h5", "r") as h:
            index = h.get("_video_archive")
            for cam in CAMERAS:
                if index is not None and cam in index:
                    stamps[cam] = index[cam]["timestamps"][:]
                    video = arc_dir / str(index[cam].attrs.get("video", f"{stem}.{cam}.mp4"))
                    cap = cv2.VideoCapture(str(video))
                    if not cap.isOpened():
                        raise DataError(f"The archive video {video.name} could not be opened.")
                    self.caps[cam], self.pos[cam] = cap, 0
        self.frames, self.freq, self.cameras, self.nearest = _timeline(stamps)

    def frame(self, i):
        import cv2

        i = max(0, min(i, self.frames - 1))
        out = []
        for c in self.cameras:
            j, cap = int(self.nearest[c][i]), self.caps[c]
            if j != self.pos[c]:            # sequential playback reads on; a jump seeks
                cap.set(cv2.CAP_PROP_POS_FRAMES, j)
            ok, img = cap.read()
            self.pos[c] = j + 1 if ok else -1
            if not ok:
                raise DataError(f"Frame {j} of the {c} video could not be decoded.")
            out.append((c, cv2.cvtColor(img, cv2.COLOR_BGR2RGB)))
        return out

    def close(self):
        for cap in self.caps.values():
            cap.release()
