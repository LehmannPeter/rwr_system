"""Recorded data, hand configuration and hand calibration helpers for APE UI."""
import csv
import importlib.util
import json
import os
import re
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
            first = next(iter(streams.values()), {})
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

    def upload_command(self):
        return [sys.executable, "-u", str(self.tool), "--data", str(self.data), "push"]

    def hf_repo(self):
        try:
            return (self.data / "hf_repo.txt").read_text().strip()
        except Exception:
            return os.environ.get("RWR_HF_REPO", "")

    def list_datasets(self):
        out = []
        raw_names = {d.name for d in self.raw.iterdir() if d.is_dir()} if self.raw.is_dir() else set()
        if self.processed.is_dir():  # processed folders whose raw folder is gone
            raw_names |= {d.name[: -len(PROCESSED_SUFFIX)] for d in self.processed.iterdir() if d.is_dir() and d.name.endswith(PROCESSED_SUFFIX)}
        manifest = self._manifest()
        prefix_of = lambda session: (re.match(r"^(s\d+)_\d{4}-\d{2}-\d{2}_.+$", session) or [None, session])[1]
        for name in sorted(raw_names):
            raw_dir, proc_dir = self.raw / name, self.processed / (name + PROCESSED_SUFFIX)
            raw = {f.stem: f for f in raw_dir.glob("*.h5")} if raw_dir.is_dir() else {}
            proc = {f.stem: f for f in proc_dir.glob("*.h5")} if proc_dir.is_dir() else {}
            # Name in dataset versions, as rwr_data.py builds it: <session prefix>_<colour>_<position in the session>
            listed = sorted({stem for (ses, stem) in manifest if ses == name} | set(raw) | set(proc))
            number = {stem: i for i, stem in enumerate(listed, start=1)}
            demos = []
            for stem in sorted(set(raw) | set(proc)):
                row = manifest.get((name, stem), {})
                demos.append({
                    "dataset_name": f"{prefix_of(name)}_{row.get('colour') or 'nocolour'}_{number[stem]:04d}.h5",
                    "description": row.get("description", ""),
                    "name": stem,
                    "raw_gb": round(raw[stem].stat().st_size / 1e9, 2) if stem in raw else None,
                    "processed_gb": round(proc[stem].stat().st_size / 1e9, 2) if stem in proc else None,
                    "check": self._summary(name, stem) if stem in proc else None,
                    "colour": manifest.get((name, stem), {}).get("colour", ""),
                    "keep": manifest.get((name, stem), {}).get("keep", ""),
                })
            out.append({
                "name": name,
                "demos": demos,
                "total_gb": round(sum((d["raw_gb"] or 0) + (d["processed_gb"] or 0) for d in demos), 1),
                "todo": sum(1 for d in demos if d["raw_gb"] is not None and (d["processed_gb"] is None or d["check"] is None)),
                "processed_dir": str(proc_dir),
                "next_number": len(listed) + 1,
            })
        return out

    def process_command(self, dataset):
        """Shell script: synchronize the demos that are not processed yet, then check the unchecked ones."""
        safe(dataset)
        raw_dir, proc_dir = self.raw / dataset, self.processed / (dataset + PROCESSED_SUFFIX)
        if not raw_dir.is_dir():
            raise DataError("This dataset has no raw recordings.")
        prev = self.previews / dataset
        return (
            f'set -u; cd "{self.workspace}"\n'
            f'echo "== Synchronizing new recordings of {dataset} =="\n'
            f'ros2 run logger syncronize_data.py "{raw_dir}" --compress --skip_existing --max_parallel 2 --output_folder "{proc_dir}" --cfg_file "{self.logger_cfg}"\n'
            f'sync_status=$?\n'
            f'echo "== Checking recordings =="\n'
            f'for f in "{proc_dir}"/*.h5; do [ -e "$f" ] || continue; n=$(basename "$f" .h5)\n'
            f'  if [ ! -f "{prev}/$n/summary.json" ]; then echo "-- $n"; '
            f'python "{self.inspect_script}" "$f" --output "{prev}/$n" > "{prev}/$n.log" 2>&1 '
            f'&& echo "   ok" || echo "   PROBLEMS FOUND (see the demo list)"; fi\n'
            f'done\n'
            f'echo "== Done =="; exit $sync_status\n'
        ), prev

    # -- player ------------------------------------------------------------------
    def _open(self, dataset, demo):
        import h5py

        path = self.processed / (safe(dataset) + PROCESSED_SUFFIX) / (safe(demo) + ".h5")
        if not path.is_file():
            raise DataError("This demo has not been processed yet.")
        if self._h5 is None or self._h5[0] != path:
            if self._h5 is not None:
                try:
                    self._h5[1].close()
                except Exception:
                    pass
            self._h5 = (path, h5py.File(path, "r"))
        return self._h5[1]

    def demo_meta(self, dataset, demo):
        with self._h5_lock:
            h = self._open(dataset, demo)
            n = len(h["timestamps"]) if "timestamps" in h else 0
            freq = float(h.attrs.get("sampling_frequency", 10))

            def series(key, cols=None):
                if key not in h:
                    return None
                v = np.asarray(h[key][:], dtype=float)
                if cols is not None:
                    v = v[:, cols]
                return np.round(v, 4).tolist()

            return {
                "frames": n,
                "freq": freq,
                "cameras": [c for c in CAMERAS if f"observations/images/{c}/color" in h],
                "arm_pos": series("observations/right/qpos_arm", slice(0, 3)),
                "arm_cmd": series("right/actions_arm", slice(0, 3)),
                "hand": series("observations/right/qpos_hand"),
                "hand_cmd": series("right/actions_hand"),
            }

    def demo_frame(self, dataset, demo, index, width=480):
        """One JPEG with both camera views of frame `index` side by side."""
        import cv2

        with self._h5_lock:
            h = self._open(dataset, demo)
            tiles = []
            for cam in CAMERAS:
                key = f"observations/images/{cam}/color"
                if key not in h:
                    continue
                ds = h[key]
                img = rotate_for_display(cam, ds[max(0, min(int(index), len(ds) - 1))])
                height = 270  # same height for every view, so turned (portrait) views fit next to landscape ones
                tiles.append(cv2.resize(img, (int(round(img.shape[1] * height / img.shape[0])), height), interpolation=cv2.INTER_AREA))
        if not tiles:
            raise DataError("This demo contains no camera images.")
        frame = cv2.cvtColor(np.hstack(tiles), cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            raise DataError("Could not encode the frame.")
        return buf.tobytes()
