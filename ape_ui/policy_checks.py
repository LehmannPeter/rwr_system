"""Trained policies on this PC, and the checks that compare what a policy was trained on
with the station as it is now, before the policy is run on the robot.

A run is a folder <policies>/<run>/ holding config.yaml and checkpoints/*.pth, as
rwr_il writes it under runs/. The run's config.yaml names the dataset version
(`dataset_name`); data/datasets/<version>/version.csv lists its demos, and each demo's
raw/<session>/<demo>.meta.json holds the station settings at recording time.
"""
import csv
import json
import math
import re
import subprocess
import threading
import time
from collections import Counter
from pathlib import Path

RUN_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.+-]{0,120}$")
CKPT_SUFFIXES = (".pth", ".pt", ".ckpt")
# Fields of the Franka robot state that are set in Franka Desk (end effector and load).
EE_FIELDS = (
    ("m_ee", "Hand mass (kg)"), ("f_x_cee", "Hand centre of mass (m)"), ("i_ee", "Hand inertia"),
    ("f_t_ee", "Flange to end-effector transform"), ("ee_t_k", "End-effector to stiffness-frame transform"),
    ("m_load", "Load mass (kg)"), ("f_x_cload", "Load centre of mass (m)"), ("i_load", "Load inertia"),
)
EE_TOL = 1e-4
START_OK_M, START_OK_DEG = 0.05, 15.0  # the arm counts as "at the demos' start" within these


class PolicyError(Exception):
    pass


def _get(d, path, default=None):
    for key in path.split("."):
        if not isinstance(d, dict) or key not in d:
            return default
        d = d[key]
    return d


def _same(a, b, tol=1e-9):
    """Equal YAML values, with a tolerance on numbers."""
    if isinstance(a, bool) or isinstance(b, bool):
        return a is b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isclose(a, b, rel_tol=0.0, abs_tol=tol)
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_same(a[k], b[k], tol) for k in a)
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_same(x, y, tol) for x, y in zip(a, b))
    return a == b


def _flat(value):
    return [float(v) for v in value] if isinstance(value, (list, tuple)) else [float(value)]


def _ee_differences(recorded, now):
    """Texts for the end-effector fields that differ between two parameter sets."""
    out = []
    for field, label in EE_FIELDS:
        if field not in recorded or field not in now:
            continue
        a, b = _flat(recorded[field]), _flat(now[field])
        if len(a) != len(b) or any(abs(x - y) > EE_TOL for x, y in zip(a, b)):
            show = lambda v: f"{v[0]:.3f}" if len(v) == 1 else "[" + ", ".join(f"{x:.3g}" for x in v) + "]"
            out.append(f"{label}: recorded {show(a)}, now {show(b)}")
    return out


def _ee_key(p):
    try:
        return tuple(round(v, 4) for field, _ in EE_FIELDS for v in _flat(p[field]))
    except Exception:
        return None


def _plural(n, word):
    return f"{n} {word}{'' if n == 1 else 's'}"


class Policies:
    def __init__(self, workspace: Path, data: Path, folder: Path, hand_cfg_dir: Path):
        self.workspace, self.data, self.folder, self.hand_cfg_dir = Path(workspace), Path(data), Path(folder), Path(hand_cfg_dir)
        self._cache = {}
        self._lock = threading.Lock()

    # -- files -----------------------------------------------------------------------
    def _first(self, *relative):
        paths = [self.workspace / r for r in relative]
        return next((p for p in paths if p.is_file()), paths[0])

    @property
    def inference_cfg(self):
        return self._first("install/experiments/share/experiments/cfgs/inference_config_cube_sorting.yaml",
                           "src/rwr_system/experiments/cfgs/inference_config_cube_sorting.yaml")

    @property
    def node_source(self):
        return self._first("install/experiments/lib/experiments/model_inference_node.py",
                           "src/rwr_system/experiments/model_inference_node.py")

    @property
    def logger_cfg(self):
        return self._first("src/rwr_system/experiments/cfgs/logger/datacollection_right.yaml",
                           "install/experiments/share/experiments/cfgs/logger/datacollection_right.yaml")

    def _cached(self, kind, path, load):
        """load(path), remembered until the file changes. None if the file is missing or unreadable."""
        path = Path(path)
        try:
            st = path.stat()
        except OSError:
            return None
        key, stamp = (kind, str(path)), (st.st_mtime_ns, st.st_size)
        hit = self._cache.get(key)
        if hit and hit[0] == stamp:
            return hit[1]
        try:
            value = load(path)
        except Exception:
            value = None
        self._cache[key] = (stamp, value)
        return value

    def _yaml(self, path):
        import yaml

        return self._cached("yaml", path, lambda p: yaml.safe_load(p.read_text()) or {})

    def _json(self, path):
        return self._cached("json", path, lambda p: json.loads(p.read_text()))

    def _h5(self, path):
        """Sampling frequency and first arm pose of a processed demo."""
        def load(p):
            import h5py

            with h5py.File(p, "r") as h:
                freq = h.attrs.get("sampling_frequency")
                key = "observations/right/qpos_arm"
                start = [float(v) for v in h[key][0]] if key in h and len(h[key]) else None
                return {"freq": float(freq) if freq is not None else None, "start": start}

        return self._cached("h5", path, load)

    # -- runs ------------------------------------------------------------------------
    def resolve(self, run, ckpt):
        """(run folder, checkpoint file, config.yaml or None). The config is looked up as
        srl_il's get_policy_from_ckpt does: beside the checkpoint, then one folder up."""
        parts = Path(str(ckpt or "")).parts
        if not RUN_NAME.match(str(run or "")) or not 1 <= len(parts) <= 2 or not all(RUN_NAME.match(p) for p in parts):
            raise PolicyError("Choose a run and a checkpoint first.")
        run_dir = self.folder / run
        path = run_dir.joinpath(*parts)
        if path.suffix not in CKPT_SUFFIXES or not path.is_file():
            raise PolicyError(f"The checkpoint {run}/{ckpt} was not found in {self.folder}.")
        config = next((c for c in (path.parent / "config.yaml", path.parent.parent / "config.yaml") if c.is_file()), None)
        return run_dir, path, config

    def list_runs(self):
        runs = []
        folders = sorted((d for d in self.folder.iterdir() if d.is_dir() and RUN_NAME.match(d.name)), key=lambda d: d.name, reverse=True) if self.folder.is_dir() else []
        for d in folders:
            files = [p for sub in (d, d / "checkpoints") if sub.is_dir() for p in sub.iterdir() if p.is_file() and p.suffix in CKPT_SUFFIXES and RUN_NAME.match(p.name)]
            if not files:
                continue
            files.sort(key=lambda p: ("best" not in p.name, -p.stat().st_mtime))  # best_model first, then newest
            cfg = self._yaml(d / "config.yaml") or self._yaml(d / "checkpoints" / "config.yaml") or {}
            previewed = self._json(d / ".ape_previewed.json") or {}
            runs.append({
                "name": d.name,
                "dataset": str(cfg.get("dataset_name") or ""),
                "algo": str(_get(cfg, "algo_cfg._target_", "")).rsplit(".", 1)[-1],
                "checkpoints": [{
                    "name": str(p.relative_to(d)), "mb": round(p.stat().st_size / 1e6),
                    "modified": p.stat().st_mtime, "previewed": previewed.get(str(p.relative_to(d)), ""),
                } for p in files],
            })
        return runs

    def mark_previewed(self, run, ckpt):
        """Remember that this checkpoint was started in preview on this PC."""
        try:
            run_dir, path, _ = self.resolve(run, ckpt)
            f = run_dir / ".ape_previewed.json"
            done = dict(self._json(f) or {})
            done[str(path.relative_to(run_dir))] = time.strftime("%Y-%m-%d %H:%M")
            tmp = f.with_suffix(".tmp")
            tmp.write_text(json.dumps(done, indent=1))
            tmp.replace(f)
        except Exception:
            pass

    # -- the training data of a run ------------------------------------------------------
    def _demos(self, dataset):
        """Demos of a dataset version, each with its settings file and processed-file facts. None if the version is not here."""
        if not dataset or not RUN_NAME.match(str(dataset)):
            return None
        folder = self.data / "datasets" / str(dataset)
        rows = self._cached("csv", folder / "version.csv", lambda p: list(csv.DictReader(open(p, newline=""))))
        if rows is None:
            return None
        demos = []
        for r in rows:
            session, stem = r.get("session", ""), Path(r.get("file", "")).stem
            h5 = self._h5(folder / r.get("name", "")) or self._h5(self.data / "processed" / (session + "_10hz") / (stem + ".h5"))
            demos.append({"name": r.get("name", ""), "row": r, "meta": self._json(self.data / "raw" / session / (stem + ".meta.json")), "h5": h5})
        return demos

    def start_pose(self, demos):
        """The typical first arm pose of the demos: the real start pose closest to the median start position.
        Returns (pose of 7, largest distance of any demo start from it in m) or (None, None)."""
        starts = [d["h5"]["start"] for d in demos or [] if d["h5"] and d["h5"].get("start") and len(d["h5"]["start"]) == 7]
        if not starts:
            return None, None
        median = [sorted(s[k] for s in starts)[len(starts) // 2] for k in range(3)]
        pose = min(starts, key=lambda s: math.dist(s[:3], median))
        return pose, max(math.dist(s[:3], pose[:3]) for s in starts)

    def _gpu(self):
        hit = self._cache.get("gpu")
        if hit and time.time() - hit[0] < 10:
            return hit[1]
        try:
            out = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.used,memory.total", "--format=csv,noheader,nounits"],
                                 capture_output=True, text=True, timeout=4)
            name, used, total = [v.strip() for v in out.stdout.strip().splitlines()[0].split(",")]
            value = {"name": name, "free_gb": (float(total) - float(used)) / 1024, "total_gb": float(total) / 1024}
        except Exception:
            value = None
        self._cache["gpu"] = (time.time(), value)
        return value

    def _commit(self):
        hit = self._cache.get("commit")
        if hit and time.time() - hit[0] < 30:
            return hit[1]
        try:
            repo = str(self.workspace / "src/rwr_system")
            head = subprocess.run(["git", "-C", repo, "rev-parse", "--short", "HEAD"], capture_output=True, text=True, timeout=5).stdout.strip()
            dirty = subprocess.run(["git", "-C", repo, "status", "--porcelain", "--untracked-files=no"], capture_output=True, text=True, timeout=5).stdout.strip()
            value = head + ("+local-changes" if head and dirty else "")
        except Exception:
            value = ""
        self._cache["commit"] = (time.time(), value)
        return value

    # -- checks ----------------------------------------------------------------------
    def check(self, run, ckpt, live=None):
        """Compare the run with the station. `live`: {"ee_params", "arm" (pose of 7), "robot" (mode dict)}; each
        may be None when the Franka controller is not running.

        Returns {"items": [{"id", "title", "status", "text"}], "bad", "warn", "start_pose", "dataset"}.
        Status: ok, info, warn (could not be verified, or a difference that may matter), bad (a mismatch
        that makes a live run wrong or impossible)."""
        with self._lock:
            return self._check(run, ckpt, live or {})

    def _check(self, run, ckpt, live):
        run_dir, path, config = self.resolve(run, ckpt)
        items = []

        def add(ident, title, status, text):
            items.append({"id": ident, "title": title, "status": status, "text": text})

        def guarded(ident, title, func):
            try:
                func()
            except Exception as exc:
                add(ident, title, "warn", f"This check could not be carried out ({type(exc).__name__}: {exc}).")

        cfg = self._yaml(config) if config else None
        if not cfg:
            add("files", "Policy files", "bad", "No readable config.yaml next to the checkpoint or one folder up, so the policy cannot be loaded. "
                "Copy config.yaml from the training run into " + str(run_dir) + ".")
            return {"items": items, "bad": 1, "warn": 0, "start_pose": None, "dataset": ""}
        add("files", "Policy files", "ok", f"{path.name} ({path.stat().st_size / 1e6:.0f} MB) with {config.relative_to(run_dir)}.")

        dataset = str(cfg.get("dataset_name") or "")
        demos = self._demos(dataset)
        with_meta = [d for d in demos or [] if isinstance(d["meta"], dict)]
        inf = self._yaml(self.inference_cfg) or {}

        def data():
            if not dataset:
                return add("data", "Training data", "warn", "The run's config.yaml names no dataset version (dataset_name), so the checks against the recordings are skipped.")
            if demos is None:
                return add("data", "Training data", "warn", f"The run was trained on dataset version {dataset} (dataset_name in its config.yaml). "
                           f"That version is not in {self.data / 'datasets'} on this PC, so the checks against the recordings are skipped.")
            count = lambda key: ", ".join(f"{v or 'not set'} {n}" for v, n in sorted(Counter(d["row"].get(key, "") for d in demos).items()))
            text = (f"Trained on {dataset}: {_plural(len(demos), 'demo')}. Colour: {count('colour')}. Lighting: {count('lighting')}. "
                    f"Arena pose: {count('arena_pose')}. Operator: {count('operator')}.")
            missing = len(demos) - len(with_meta)
            if missing:
                return add("data", "Training data", "warn", text + f" {_plural(missing, 'demo')} of them {'has' if missing == 1 else 'have'} no settings file "
                           "(recorded before APE stored them), so the checks below cover only the others.")
            add("data", "Training data", "ok", text)

        def frequency():
            title = "Control frequency"
            try:
                m = re.search(r"control_freq_hz\s*=\s*([0-9.]+)", self.node_source.read_text())
            except OSError:
                m = None
            if not m:
                return add("freq", title, "warn", f"The control rate could not be read from {self.node_source}.")
            node = float(m.group(1))
            freqs = Counter(d["h5"]["freq"] for d in demos or [] if d["h5"] and d["h5"].get("freq") is not None)
            if not freqs:
                return add("freq", title, "warn", f"The inference node runs at {node:g} Hz. The processed demos of this dataset version are not on this PC, "
                           "so their sampling rate could not be read. They must be resampled at the same rate.")
            if set(freqs) == {node}:
                return add("freq", title, "ok", f"The inference node runs at {node:g} Hz and all {_plural(sum(freqs.values()), 'demo')} are resampled at {node:g} Hz.")
            rates = ", ".join(f"{_plural(n, 'demo')} at {f:g} Hz" for f, n in freqs.most_common())
            add("freq", title, "bad", f"The inference node runs at {node:g} Hz, but the training data is resampled differently: {rates}. "
                "The policy would move at the wrong speed. Change control_freq_hz in model_inference_node.py, or retrain on data at that rate.")

        def end_effector():
            title = "Franka end-effector parameters"
            groups = Counter(k for k in (_ee_key(d["meta"].get("end_effector_parameters") or {}) for d in with_meta) if k)
            now = live.get("ee_params")
            if not groups:
                return add("ee", title, "warn", "The demos have no recorded end-effector parameters to compare with. Compare Franka Desk with your screenshot by hand.")
            if not now or _ee_key(now) is None:
                return add("ee", title, "warn", "The Franka controller is not running, so the values set in Franka Desk cannot be read. "
                           "Start the controller (section 1); this check then repeats by itself.")
            current = _ee_key(now)
            same = lambda a, b: all(abs(x - y) <= EE_TOL for x, y in zip(a, b))
            matching = sum(n for k, n in groups.items() if same(k, current))
            total = sum(groups.values())
            if matching == total:
                return add("ee", title, "ok", f"The values set in Franka Desk now (hand mass {now['m_ee']:.2f} kg) match all {_plural(total, 'demo')}.")
            recorded = next(d["meta"]["end_effector_parameters"] for d in with_meta if _ee_key(d["meta"].get("end_effector_parameters") or {}) == groups.most_common(1)[0][0])
            diffs = _ee_differences(recorded, now) if not same(groups.most_common(1)[0][0], current) else []
            text = f"The values set in Franka Desk now differ from {total - matching} of {_plural(total, 'demo')}."
            if diffs:
                text += " " + "; ".join(diffs) + "."
            add("ee", title, "bad", text + " With other values the arm holds its pose differently and reports a different hand pose. Set them in Franka Desk as at recording time.")

        def hand_files():
            import yaml

            for ident, title, key, filename, effect in (
                ("calib", "Hand calibration", "hand_calibration_yaml", "calibration.yaml",
                 "The hand was recalibrated since, so the same joint angles can give slightly different finger positions. Watch the fingers in preview."),
                ("handcfg", "Hand config", "hand_config_yaml", "config.yaml", "Check that the difference does not change how the fingers move."),
            ):
                texts = [d["meta"].get(key) for d in with_meta if d["meta"].get(key)]
                if not texts:
                    add(ident, title, "warn", "The demos have no recorded copy to compare with.")
                    continue
                try:
                    now = yaml.safe_load((self.hand_cfg_dir / filename).read_text())
                except OSError:
                    add(ident, title, "warn", f"There is no {filename} for the hand on this PC right now"
                        + (": the hand calibrates itself at the next launch start." if ident == "calib" else "."))
                    continue
                parsed = {t: yaml.safe_load(t) for t in set(texts)}
                differing = sum(1 for t in texts if not _same(parsed[t], now))
                if not differing:
                    add(ident, title, "ok", f"Same as in all {_plural(len(texts), 'demo')}.")
                    continue
                recorded = parsed[Counter(texts).most_common(1)[0][0]]
                keys = sorted(str(k) for k in set(recorded or {}) | set(now or {}) if not _same((recorded or {}).get(k), (now or {}).get(k))) if isinstance(recorded, dict) and isinstance(now, dict) else []
                add(ident, title, "warn", f"Differs from {differing} of {_plural(len(texts), 'demo')}" + (f" (in: {', '.join(keys)})" if keys else "") + ". " + effect)

        def consistency():
            title = "Demos agree with each other"
            if not with_meta:
                return
            mixed = []
            for label, value in (
                ("end-effector parameters", lambda m: _ee_key(m.get("end_effector_parameters") or {})),
                ("hand calibrations", lambda m: m.get("hand_calibration_yaml")),
                ("hand configs", lambda m: m.get("hand_config_yaml")),
                ("retargeter configs", lambda m: (m.get("retargeter_cfgs_yaml"), m.get("mano_adjustment_yaml"))),
            ):
                groups = Counter(value(d["meta"]) for d in with_meta)
                if len(groups) > 1:
                    mixed.append(f"{len(groups)} different {label} ({' + '.join(str(n) for _, n in groups.most_common())} demos)")
            if mixed:
                return add("mixed", title, "warn", "The training demos were not all recorded with the same settings: " + "; ".join(mixed) + ". The policy learned from a mixture.")
            add("mixed", title, "ok", f"All {_plural(len(with_meta), 'demo')} were recorded with the same end-effector parameters, hand calibration, hand config and retargeter config.")

        def names():
            title = "Observation and action names"
            needed = [o[0] for o in _get(cfg, "algo_cfg.policy_cfg.policy_obs_list", []) or []]
            targets = _get(cfg, "algo_cfg.algo_cfg.target_dims", {}) or {}
            if not inf or not needed or not targets:
                return add("names", title, "warn", "The observation and action names could not be read from the run's config.yaml or from " + self.inference_cfg.name + ".")
            provided = set(inf.get("camera_names") or []) | set(inf.get("observations") or {})
            actions = list(inf.get("actions") or {})
            problems = []
            if [k for k in needed if k not in provided]:
                problems.append("the policy expects " + ", ".join(k for k in needed if k not in provided) + f", which {self.inference_cfg.name} does not provide")
            if [k for k in actions if k not in targets]:
                problems.append(f"{self.inference_cfg.name} sends " + ", ".join(k for k in actions if k not in targets) + ", which the policy does not output")
            hand_dim, want = targets.get("actions_hand"), inf.get("hand_qpos_dim")
            if isinstance(hand_dim, int) and isinstance(want, int) and hand_dim != want:
                problems.append(f"the policy outputs {hand_dim} hand joints, the inference config expects {want}")
            if problems:
                return add("names", title, "bad", "The policy and the inference config do not fit: " + "; ".join(problems) + ". The inference node would stop with an error.")
            unsent = [k for k in targets if k not in actions]
            add("names", title, "warn" if unsent else "ok", f"The inference config provides all {len(needed)} inputs ({', '.join(needed)}) and sends {', '.join(actions)}."
                + (f" The policy also outputs {', '.join(unsent)}, which is not sent to the robot." if unsent else ""))

        def sources():
            title = "Cameras and sensors feed the same inputs"
            needed = [o[0] for o in _get(cfg, "algo_cfg.policy_cfg.policy_obs_list", []) or []]
            trained = {k[0]: k[1] for k in _get(cfg, "dataset_cfg.data.keys_traj", []) or [] if isinstance(k, (list, tuple)) and len(k) >= 2}
            topics = dict(zip(inf.get("camera_names") or [], inf.get("camera_topics") or [])) | dict(inf.get("observations") or {})
            recorded_as = (self._yaml(self.logger_cfg) or {}).get("keys_and_topics") or {}
            if not needed or not trained or not topics or not recorded_as:
                return add("sources", title, "warn", "The mapping from recorded topics to policy inputs could not be read, so it was not compared.")
            wrong, unknown, good = [], [], []
            for name in needed:
                source, topic = trained.get(name), topics.get(name)
                if not source or not topic:
                    continue
                key = next((template.format(*m.groups()) for template, spec in recorded_as.items()
                            for m in [re.fullmatch(str(spec.get("topic", "")), topic)] if m), None)
                if key is None:
                    unknown.append(f"{name} ({topic})")
                elif key == source or key.endswith("/" + source):
                    good.append(f"{name} ← {topic}")
                else:
                    wrong.append(f"{name} was trained on {source}, but is now read from {topic} (recorded as {key})")
            if wrong:
                return add("sources", title, "bad", "; ".join(wrong) + ". The policy would see the wrong camera or sensor. Correct " + self.inference_cfg.name + ".")
            if unknown:
                return add("sources", title, "warn", "Not recorded by the logger config, so not comparable: " + ", ".join(unknown) + ".")
            add("sources", title, "ok", "Each input is read from the topic it was recorded from: " + "; ".join(good) + ".")

        def images():
            title = "Image size and cropping"
            size, want = _get(cfg, "dataset_cfg.data.rgb_resize_to"), inf.get("rgb_resize_size")
            crop, process = _get(cfg, "dataset_cfg.data.rgb_crop_dict") or {}, inf.get("rgb_process_dict") or {}
            if not size or not want:
                return add("images", title, "warn", "The image size could not be read from both configs.")
            if sorted(size) != sorted(want):
                return add("images", title, "bad", f"Training resized the images to {list(size)}, the inference config resizes to {list(want)}. Set rgb_resize_size in {self.inference_cfg.name}.")
            if bool(crop) != bool(process):
                return add("images", title, "bad", ("Training cropped the images (rgb_crop_dict), but the inference config applies no crop." if crop else
                           "The inference config crops or turns the images (rgb_process_dict), but training did not.") + f" Make rgb_process_dict in {self.inference_cfg.name} mirror the training crop.")
            if crop:
                return add("images", title, "warn", f"Both sides resize to {list(size)} and both crop. The two crops are written in different forms and were not compared: check by hand that they are the same.")
            add("images", title, "ok", f"Both sides resize to {list(size)} without cropping.")

        pose, spread = self.start_pose(demos)

        def start():
            title = "Arm start pose"
            if pose is None:
                return add("start", title, "info", "The demos' start poses are not on this PC (no processed files), so the arm's pose was not compared.")
            where = f"The demos start at x {pose[0] * 100:.0f}, y {pose[1] * 100:.0f}, z {pose[2] * 100:.0f} cm (all within {spread * 100:.0f} cm of that)."
            arm = live.get("arm")
            if not arm or len(arm) < 7:
                return add("start", title, "info", where + " The arm's pose is unknown: the Franka controller is not running.")
            dist = math.dist(arm[:3], pose[:3])
            angle = math.degrees(2 * math.acos(min(1.0, abs(sum(a * b for a, b in zip(arm[3:7], pose[3:7]))))))
            if dist <= START_OK_M and angle <= START_OK_DEG:
                return add("start", title, "ok", where + f" The arm is {dist * 100:.0f} cm and {angle:.0f}° from it.")
            add("start", title, "warn", where + f" The arm is {dist * 100:.0f} cm and {angle:.0f}° away from it. Bring it there before starting the policy: it has only seen motions that begin there.")

        def station():
            robot = live.get("robot")
            if not robot:
                add("robot", "Franka controller", "warn", "Not running. The policy then gets no arm pose and produces no actions, in preview too. Start it in section 1.")
            elif robot.get("mode") != "ready":
                add("robot", "Franka controller", "warn", f"Running, but the robot is {str(robot.get('mode', '')).replace('_', ' ')}. A preview works; a live run needs Ready.")
            else:
                add("robot", "Franka controller", "ok", "Running and the robot is Ready.")
            gpu = self._gpu()
            if not gpu:
                add("gpu", "Graphics card", "bad", "No NVIDIA graphics card answers (nvidia-smi). The policy is loaded onto the graphics card and cannot start without it.")
            elif gpu["free_gb"] < 2.0:
                add("gpu", "Graphics card", "warn", f"Only {gpu['free_gb']:.1f} of {gpu['total_gb']:.0f} GB graphics memory is free. Loading the policy may fail: close other programs that use it.")
            else:
                add("gpu", "Graphics card", "ok", f"{gpu['name']}: {gpu['free_gb']:.1f} of {gpu['total_gb']:.0f} GB memory free.")
            when = (self._json(run_dir / ".ape_previewed.json") or {}).get(str(path.relative_to(run_dir)))
            if when:
                add("preview", "Tried in preview", "ok", f"This checkpoint was last started in preview on this PC on {when}.")
            else:
                add("preview", "Tried in preview", "warn", "This checkpoint has not been started in preview on this PC yet. Do that before a live run.")

        def notes():
            chunk, agg = _get(cfg, "algo_cfg.algo_cfg.T_target"), _get(cfg, "algo_cfg.policy_cfg.policy_aggregator_cfg") or {}
            if chunk:
                how = f"aggregation {agg.get('type')}, a new prediction every {agg.get('update_every', 1)} steps" + (f", k = {agg.get('k')}" if agg.get("type") == "temporal_aggr" else "")
                add("chunk", "Action chunks", "info", f"The policy predicts {chunk} steps per chunk ({how}). This is read from policy_aggregator_cfg in the run's config.yaml and can be edited there.")
            commits = Counter(d["meta"].get("rwr_system_commit") or "unknown" for d in with_meta)
            if commits:
                add("code", "Code version", "info", "Demos recorded with rwr_system " + ", ".join(f"{c} ({n})" for c, n in commits.most_common()) + f"; now {self._commit() or 'unknown'}.")

        for ident, title, func in (("data", "Training data", data), ("freq", "Control frequency", frequency), ("ee", "Franka end-effector parameters", end_effector),
                                   ("calib", "Hand calibration and config", hand_files), ("mixed", "Demos agree with each other", consistency),
                                   ("names", "Observation and action names", names), ("sources", "Cameras and sensors feed the same inputs", sources),
                                   ("images", "Image size and cropping", images), ("start", "Arm start pose", start),
                                   ("robot", "Station", station), ("chunk", "Notes", notes)):
            guarded(ident, title, func)
        return {"items": items, "bad": sum(1 for i in items if i["status"] == "bad"), "warn": sum(1 for i in items if i["status"] == "warn"),
                "start_pose": pose, "dataset": dataset}
