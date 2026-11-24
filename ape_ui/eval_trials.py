#!/usr/bin/env python3
"""Evaluation of policy runs for APE.

Every live run of a policy (from Start policy to Stop policy in card 4) is recorded: both camera
views as video and every arm and hand signal APE receives. The cubes placed on the surface of their
colour are counted live from the workspace camera (LiveTracker, eval_detect.py --live) and counted
again from the saved video when the run has ended, as a check.
One folder per run under <data>/eval/<run id>/:

    trial.json            policy, placements found, result, automatic measures (the source of truth)
    signals.h5            one group per stream with t (seconds since the policy started) and data
    oakd_front_view.mp4   10 frames per second; video time = trial time
    oakd_wrist_view.mp4
    <data>/eval/trials.csv   one row per run, rewritten after every change

This file has no ROS in it: server.py pushes samples and camera messages in through feed() and
feed_frame(), so it can be tested without the robot (tests/test_eval.py).
"""
import csv
import json
import os
import queue
import re
import shutil
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

GRADED_SECONDS = 180.0      # the course counts relocations in 3 minutes: the count is also given for this time
VIDEO_FPS = 10              # the policy's control rate and the demos' sampling rate
MAX_RATE_HZ = 100.0         # no stream is stored faster than this
STILL_SECONDS = 5.0         # arm and hand motionless for this long = one automatic "still" interval
EVENT_KINDS = {"placed": "Cube placed correctly"}   # the only event for now; more kinds can be added here
END_REASONS = {
    "operator": "stopped in the page",
    "robot": "stopped because the robot left Ready",
    "launch_stopped": "the policy launch ended",
    "stopped": "the policy stopped",
}
TRIAL_ID = re.compile(r"^\d{8}-\d{6}(_\d+)?$")
CSV_FIELDS = ["id", "started", "run", "ckpt", "algo", "dataset", "duration_s", "end_reason", "placed", "placed_180", "counted_live", "recounted",
              "seconds_per_cube", "auto_still_s", "auto_hand_closings", "predict_ms", "dropped_steps", "note"]


class EvalError(Exception):
    pass


def find_ffmpeg():
    """$FFMPEG, then ffmpeg on PATH, then the binary shipped with imageio-ffmpeg (the order the video archive uses)."""
    for candidate in (os.environ.get("FFMPEG"), shutil.which("ffmpeg")):
        if candidate and os.path.exists(candidate):
            return candidate
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return None


class VideoFile:
    """One camera's video. Frames are placed by their arrival time, and a gap is filled by repeating
    the last frame, so a moment of the trial is found at the same time in the video."""

    FORMATS = {"rgb8": ("rgb24", 3), "bgr8": ("bgr24", 3), "mono8": ("gray", 1)}

    def __init__(self, path, fps, ffmpeg):
        self.path, self.fps, self.ffmpeg = Path(path), fps, ffmpeg
        self.proc = self.cv = self.size = self.first = None
        self.written = 0
        self.times, self.index = [], []   # arrival time and position in the video of every distinct frame
        self.codec, self.error = "", ""

    def _open(self, w, h, fmt):
        self.size = (w, h, fmt)
        if self.ffmpeg:
            cmd = [self.ffmpeg, "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", fmt, "-s", f"{w}x{h}", "-r", str(self.fps),
                   "-i", "-", "-an", "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20",
                   "-pix_fmt", "yuv420p", "-threads", "2", "-movflags", "+faststart", str(self.path)]
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            self.codec = "h264"
        else:   # plays in VLC, not in the browser
            import cv2
            self.cv = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (w, h), fmt != "gray")
            self.codec = "mp4v"

    def write(self, stamp, msg):
        if msg.encoding not in self.FORMATS:
            self.error = f"image encoding {msg.encoding} is not supported"
            return
        fmt, ch = self.FORMATS[msg.encoding]
        w, h = int(msg.width), int(msg.height)
        if self.size is None:
            self._open(w, h, fmt)
        if (w, h, fmt) != self.size:
            self.error = "the image size changed during the trial"
            return
        if self.first is None:
            self.first = stamp
        idx = int(round((stamp - self.first) * self.fps))
        if idx < self.written:
            return
        copies = idx - self.written + 1
        if copies > 10 * self.fps:   # a gap of more than 10 s is not filled: later frames move forward by the rest
            self.first += (copies - 10 * self.fps) / self.fps
            copies = 10 * self.fps
        buf = np.frombuffer(msg.data, np.uint8)
        if int(msg.step) != w * ch:
            buf = np.ascontiguousarray(buf.reshape(h, int(msg.step))[:, : w * ch])
        for _ in range(copies):
            if self.proc is not None:
                self.proc.stdin.write(buf)
            else:
                import cv2
                img = buf.reshape(h, w, ch) if ch == 3 else buf.reshape(h, w)
                self.cv.write(cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if fmt == "rgb24" else img)
        self.written += copies
        self.times.append(stamp)
        self.index.append(self.written - 1)

    def close(self):
        try:
            if self.proc is not None:
                self.proc.stdin.close()
                err = self.proc.stderr.read().decode(errors="replace").strip()
                if self.proc.wait(timeout=120) != 0:
                    self.error = self.error or ("ffmpeg failed: " + err[-300:])
            elif self.cv is not None:
                self.cv.release()
        except Exception as exc:
            self.error = self.error or f"{type(exc).__name__}: {exc}"

    def info(self):
        return {"file": self.path.name if self.written else None, "frames": len(self.times), "video_frames": self.written, "first_unix": self.first,
                "fps": self.fps, "codec": self.codec, "size": list(self.size[:2]) if self.size else None, "error": self.error}


class Recorder:
    """Collects the samples and frames of one trial. add() and add_frame() are called from the ROS
    thread and must return at once: the video is written by a thread of its own."""

    def __init__(self, folder, cams, ffmpeg):
        self.folder = Path(folder)
        self.signals, self._last = {}, {}
        self.videos = {cam: VideoFile(self.folder / f"{cam}.mp4", VIDEO_FPS, ffmpeg) for cam in cams}
        self._frame_last = {}
        self.dropped_frames = 0
        self.open = True
        self._q = queue.Queue(maxsize=40)
        self._thread = threading.Thread(target=self._work, daemon=True)
        self._thread.start()

    def add(self, stream, stamp, values):
        if stamp - self._last.get(stream, 0.0) < 1.0 / MAX_RATE_HZ:
            return
        self._last[stream] = stamp
        self.signals.setdefault(stream, []).append((stamp, values))

    def add_frame(self, cam, stamp, msg):
        if cam not in self.videos or stamp - self._frame_last.get(cam, 0.0) < 0.9 / VIDEO_FPS:
            return
        self._frame_last[cam] = stamp
        try:
            self._q.put_nowait((cam, stamp, msg))
        except queue.Full:   # the encoder cannot keep up: lose a frame, never block the ROS thread
            self.dropped_frames += 1

    def _work(self):
        while True:
            item = self._q.get()
            if item is None:
                return
            cam, stamp, msg = item
            try:
                self.videos[cam].write(stamp, msg)
            except Exception as exc:
                self.videos[cam].error = f"{type(exc).__name__}: {exc}"

    def close(self):
        self.open = False
        self._q.put(None)
        self._thread.join(120)
        for v in self.videos.values():
            v.close()


def write_signals(path, signals, videos, t0):
    import h5py

    with h5py.File(path, "w") as f:
        f.attrs["t0_unix"] = float(t0)
        f.attrs["about"] = "Each group: t = seconds since the policy started, data = one row per sample. video/<camera>: t per stored frame and its position in the mp4."
        for name, rows in signals.items():
            if not rows:
                continue
            width = max(len(r[1]) for r in rows)
            data = np.full((len(rows), width), np.nan, np.float32)
            for i, (_, values) in enumerate(rows):
                data[i, : len(values)] = values
            g = f.create_group(name)
            g.create_dataset("t", data=np.array([r[0] for r in rows]) - t0)
            g.create_dataset("data", data=data, compression="gzip", compression_opts=4)
        for cam, v in videos.items():
            g = f.create_group(f"video/{cam}")
            g.create_dataset("t", data=np.array(v.times, dtype=np.float64) - t0)
            g.create_dataset("frame", data=np.array(v.index, dtype=np.int32))


def _series(signals, name, t0, t_end, width):
    rows = [r for r in signals.get(name, []) if t0 <= r[0] <= t_end and len(r[1]) >= width]
    if len(rows) < 20:
        return None, None
    return np.array([r[0] for r in rows]) - t0, np.array([r[1][:width] for r in rows], dtype=np.float64)


def _on_grid(t, x, grid):
    return np.stack([np.interp(grid, t, x[:, k]) for k in range(x.shape[1])], axis=1)


def auto_measures(signals, t0, t_end):
    """What can be read from the arm and hand signals alone, without the cameras. These are measurements,
    not judgements: none of them has been compared with what an observer saw (see README)."""
    out = {}
    t, pos = _series(signals, "arm", t0, t_end, 3)
    if t is None:
        return out
    out["arm_path_m"] = round(float(np.linalg.norm(np.diff(pos, axis=0), axis=1).sum()), 3)
    grid = np.arange(t[0], t[-1], 0.1)
    if len(grid) < 30:
        return out
    p = _on_grid(t, pos, grid)
    still = np.linalg.norm(p[10:] - p[:-10], axis=1) < 0.005          # the hand moved less than 5 mm in 1 s
    width = len(signals["hand"][0][1]) if signals.get("hand") else 0
    th, hand = _series(signals, "hand", t0, t_end, width) if width else (None, None)
    closings = None
    if th is not None:
        h = _on_grid(th, hand, grid)
        still &= np.abs(h[10:] - h[:-10]).max(axis=1) < 0.03           # and no finger joint moved more than 0.03 rad
        lo, hi = np.percentile(h, 2, axis=0), np.percentile(h, 98, axis=0)
        used = (hi - lo) > 0.2                                         # joints that moved at all in this trial
        if used.any():
            closed = ((h[:, used] - lo[used]) / (hi - lo)[used]).mean(axis=1)   # 0 = most open, 1 = most closed in this trial
            closings, low = 0, True
            for c in closed:
                if low and c > 0.6:
                    closings, low = closings + 1, False
                elif c < 0.4:
                    low = True
    runs, start = [], None
    for i, s in enumerate(list(still) + [False]):
        if s and start is None:
            start = i
        elif not s and start is not None:
            seconds = (i - start) * 0.1 + 1.0
            if seconds >= STILL_SECONDS:
                runs.append({"t": round(float(grid[start]), 1), "seconds": round(seconds, 1)})
            start = None
    out.update(still=runs, still_s=round(sum(r["seconds"] for r in runs), 1), hand_closings=closings)
    return out


def count(trial):
    """The result of a trial from its events (recomputed after every correction)."""
    events = trial.get("events", [])
    res = {k: sum(1 for e in events if e["kind"] == k) for k in EVENT_KINDS}
    # a mark added after the trial has no time: it counts for the trial, and for the first 3 minutes only if the trial was that short
    short = (trial.get("duration_s") or 0) <= GRADED_SECONDS
    res["placed_180"] = sum(1 for e in events if e["kind"] == "placed" and (e["t"] <= GRADED_SECONDS if e.get("t") is not None else short))
    seconds = trial.get("duration_s") or 0
    res["seconds_per_cube"] = round(seconds / res["placed"], 1) if res["placed"] else None
    return res


class LiveTracker:
    """The live count: a helper process (eval_detect.py --live) gets the workspace camera's frames, 10 per second
    at 640 pixels width, and answers with where the cube is and which placements it has counted. A process of its
    own, at low priority, so that the page and the recording are not slowed down."""

    WIDTH = 640

    def __init__(self, cam):
        self.cam, self.proc, self._q = cam, None, None
        self._started = self._last = self._stamp = 0.0
        self.t0, self.state, self.hull, self.size, self.trail = None, {}, None, None, []
        self._lock = threading.Lock()

    @property
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self):
        if self.running or time.time() - self._started < 10.0:   # not again and again if it cannot start
            return
        self._started = self.t0 = time.time()
        cmd = [sys.executable, str(Path(__file__).with_name("eval_detect.py")), "--live"]
        try:
            self.proc = subprocess.Popen((["nice", "-n", "10"] if shutil.which("nice") else []) + cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        except OSError:
            self.proc = None
            return
        self._q = queue.Queue(maxsize=10)
        with self._lock:
            self.state, self.hull, self.size, self.trail = {}, None, None, []
        threading.Thread(target=self._send, args=(self.proc, self._q), daemon=True).start()
        threading.Thread(target=self._read, args=(self.proc,), daemon=True).start()

    def stop(self):
        proc, self.proc = self.proc, None
        if proc is not None:
            try:
                self._q.put_nowait(None)   # ends the sending thread
            except queue.Full:
                pass
            try:
                proc.terminate()
            except OSError:
                pass

    def feed(self, msg):
        """A camera message (ROS thread: returns at once; a frame is skipped if the helper is behind)."""
        if self.proc is None:
            return
        now = time.time()
        if now - self._last < 0.09:
            return
        self._last = now
        try:
            self._q.put_nowait((now, msg))
        except queue.Full:
            pass

    def reset(self, t0):
        """A policy run begins at t0: the count starts at zero."""
        self.t0 = float(t0)
        if self.proc is not None:
            try:
                self._q.put((self.t0, None), timeout=2.0)
            except queue.Full:
                pass

    def _send(self, proc, q):
        import cv2

        while True:
            item = q.get()
            if item is None:
                return
            stamp, msg = item
            try:
                if msg is None:
                    proc.stdin.write(struct.pack("<dHHI", stamp, 0, 0, 1))
                elif msg.encoding in ("rgb8", "bgr8"):
                    w, h = int(msg.width), int(msg.height)
                    img = np.frombuffer(msg.data, np.uint8)
                    img = (img.reshape(h, int(msg.step))[:, : w * 3] if int(msg.step) != w * 3 else img).reshape(h, w, 3)
                    if msg.encoding == "bgr8":
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    if w != self.WIDTH:
                        img = cv2.resize(img, (self.WIDTH, int(round(h * self.WIDTH / w))), interpolation=cv2.INTER_AREA)
                    proc.stdin.write(struct.pack("<dHHI", stamp, img.shape[1], img.shape[0], 0))
                    proc.stdin.write(np.ascontiguousarray(img).tobytes())
                proc.stdin.flush()
            except (OSError, ValueError):   # the helper is gone
                return

    def _read(self, proc):
        for line in proc.stdout:
            try:
                st = json.loads(line)
            except ValueError:
                continue
            with self._lock:
                if "hull" in st:
                    self.hull, self.size = st.pop("hull"), st.pop("size", None)
                cube = st.get("cube")
                self.trail = (self.trail + [cube[:2] if cube else None])[-25:]
                self.state, self._stamp = st, time.time()

    def info(self):
        with self._lock:
            st, t0 = self.state, self.t0 or 0.0
            return {
                "running": self.running, "fresh": self.running and time.time() - self._stamp < 1.5,
                "colour": st.get("colour"), "cube": st.get("cube"), "at_surface": bool(st.get("at_surface")),
                "placed": [round(t - t0, 1) for t in st.get("placed", []) if t >= t0], "problem": st.get("problem", ""),
                "hull": self.hull, "size": self.size, "trail": list(self.trail),
            }

    def result(self):
        """What is stored with a run when it ends, or None if the live count was not running."""
        st = self.info()
        return {"placed": st["placed"], "colour": st["colour"], "problem": st["problem"]} if st["fresh"] else None


class EvalTrials:
    """The run that is being recorded (at most one) and the saved ones."""

    def __init__(self, folder, cams):
        self.folder, self.cams = Path(folder), tuple(cams)
        self.state = "idle"       # idle | starting | running | saving
        self.current = None
        self.rec = None
        self.last_id = None
        self.message = ""
        self._lock = threading.RLock()
        self._cache = {}

    # -- data in (ROS thread) ---------------------------------------------------
    def feed(self, stream, values):
        rec = self.rec
        if rec is not None and rec.open:
            rec.add(stream, time.time(), values)

    def feed_frame(self, cam, msg):
        rec = self.rec
        if rec is not None and rec.open:
            rec.add_frame(cam, time.time(), msg)

    # -- lifecycle --------------------------------------------------------------
    @property
    def busy(self):
        return self.state != "idle"

    def begin(self, meta):
        """Called right before the policy is started: recording begins now, the clock at started()."""
        with self._lock:
            if self.busy:
                raise EvalError("The previous run is still being saved.")
            base = time.strftime("%Y%m%d-%H%M%S")
            trial_id, n = base, 1
            while (self.folder / trial_id).exists():
                n += 1
                trial_id = f"{base}_{n}"
            folder = self.folder / trial_id
            folder.mkdir(parents=True)
            self.current = {"id": trial_id, "started": time.strftime("%Y-%m-%d %H:%M:%S"), **meta,
                            "events": [], "asked": time.time(), "t0": None, "end_reason": None}
            self.rec = Recorder(folder, self.cams, find_ffmpeg())
            self.state, self.message = "starting", ""
            return trial_id

    def cancel(self, message=""):
        """The policy did not start: nothing happened, so nothing is kept."""
        with self._lock:
            if self.state != "starting":
                return
            rec, trial = self.rec, self.current
            self.rec, self.current, self.state, self.message = None, None, "idle", message
        rec.close()
        shutil.rmtree(self.folder / trial["id"], ignore_errors=True)   # created a moment ago by begin()

    def started(self, t0, **extra):
        with self._lock:
            if self.state == "starting":
                self.current.update(extra, t0=float(t0))
                self.state = "running"

    def finish(self, reason, extra=None):
        with self._lock:
            if self.state != "running":
                return
            trial, rec = self.current, self.rec
            trial["ended"] = time.time()
            trial["end_reason"] = trial["end_reason"] or reason
            trial.update(extra or {})
            self.state = "saving"
        threading.Thread(target=self._save, args=(trial, rec), daemon=True).start()

    def _save(self, live, rec):
        problems = []
        t0, t_end = live["t0"], live["ended"]
        # the page keeps reading the live entry until the file is written, so the bookkeeping fields stay in it
        trial = {k: v for k, v in live.items() if k not in ("t0", "ended", "asked")}
        folder = self.folder / trial["id"]
        try:
            rec.close()
            trial["duration_s"] = round(t_end - t0, 1)
            trial["t0_unix"] = round(t0, 3)
            trial["video"] = {cam: v.info() for cam, v in rec.videos.items()}
            for v in trial["video"].values():   # when the video begins, in run time (the recording starts just before the policy does)
                first = v.pop("first_unix")
                v["starts_at_s"] = round(first - t0, 2) if first else None
            trial["video_frames_lost"] = rec.dropped_frames
            problems += [f"{cam}: {v.error}" for cam, v in rec.videos.items() if v.error]
            problems += [f"{cam}: no image arrived" for cam, v in rec.videos.items() if not v.written]
            try:
                write_signals(folder / "signals.h5", rec.signals, rec.videos, t0)
                trial["signals"] = {name: len(rows) for name, rows in rec.signals.items()}
            except Exception as exc:
                problems.append(f"signals.h5 was not written: {type(exc).__name__}: {exc}")
            for stream in ("arm", "hand"):
                if not rec.signals.get(stream):
                    problems.append(f"no {stream} data arrived")
            try:
                trial["auto"] = auto_measures(rec.signals, t0, t_end)
            except Exception as exc:
                trial["auto"] = {}
                problems.append(f"automatic measures failed: {type(exc).__name__}: {exc}")
            trial["recording_problems"] = problems
            if trial.get("live"):   # the count as it stood when the run ended
                trial["events"] = [{"t": t, "kind": "placed", "source": "live"} for t in trial["live"]["placed"] if t <= trial["duration_s"]]
            trial["detection"] = {"state": "running"}
            trial["result"] = count(trial)
            self._write(trial)
            self.write_csv()
            self.message = "The run was saved, but the recording is incomplete: " + "; ".join(problems) if problems else ""
        except Exception as exc:
            self.message = f"Saving the run failed: {type(exc).__name__}: {exc}"
        finally:
            with self._lock:
                self.last_id, self.rec, self.current, self.state = trial["id"], None, None, "idle"
        self._detect(trial["id"])   # after the state is idle again: the next run can start meanwhile

    def _detect(self, trial_id):
        """Count the placed cubes again from the saved workspace video, in a process of its own at low priority.
        The result is stored next to the live count; it becomes the count only if there was no live count."""
        try:
            video = self.folder / trial_id / f"{self.cams[0]}.mp4"
            cmd = [sys.executable, str(Path(__file__).with_name("eval_detect.py")), "--json", str(video)]
            proc = subprocess.run((["nice", "-n", "10"] if shutil.which("nice") else []) + cmd, capture_output=True, text=True, timeout=3600)
            found = json.loads(proc.stdout.strip().splitlines()[-1]) if proc.returncode == 0 and proc.stdout.strip() else {"placed": [], "problem": "the detection failed: " + proc.stderr.strip()[-300:]}
        except Exception as exc:
            found = {"placed": [], "problem": f"the detection did not run: {type(exc).__name__}: {exc}"}
        try:
            with self._lock:
                trial = self.get(trial_id)
                offset = (trial.get("video", {}).get(self.cams[0], {}) or {}).get("starts_at_s") or 0.0
                times = [round(t + offset, 1) for t in found.get("placed", [])]
                trial["detection"] = {"state": "done", "placed": times, "colour": found.get("colour"), "problem": found.get("problem", ""), "seen_share": found.get("seen_share")}
                if not trial.get("live"):
                    kept = [e for e in trial["events"] if e.get("source") != "detected"]
                    trial["events"] = sorted(kept + [{"t": t, "kind": "placed", "source": "detected"} for t in times], key=lambda e: (e["t"] is None, e["t"] or 0))
                trial["result"] = count(trial)
                self._write(trial)
                self.write_csv()
        except Exception as exc:
            self.message = f"The placements of run {trial_id} could not be stored: {type(exc).__name__}: {exc}"

    def _write(self, trial):
        f = self.folder / trial["id"] / "trial.json"
        tmp = f.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(trial, indent=1))
        tmp.replace(f)

    # -- saved trials -------------------------------------------------------------
    def trials(self):
        out = []
        if not self.folder.is_dir():
            return out
        for d in sorted(self.folder.iterdir(), reverse=True):
            f = d / "trial.json"
            if not TRIAL_ID.match(d.name) or not f.is_file():
                continue
            try:
                mtime = f.stat().st_mtime
                if self._cache.get(d.name, (None,))[0] != mtime:
                    self._cache[d.name] = (mtime, json.loads(f.read_text()))
                out.append(self._cache[d.name][1])
            except (OSError, ValueError):
                continue
        return out

    def get(self, trial_id):
        if not TRIAL_ID.match(trial_id or ""):
            raise EvalError("No such run.")
        f = self.folder / trial_id / "trial.json"
        if not f.is_file():
            raise EvalError("No such run.")
        return json.loads(f.read_text())

    def update(self, trial_id, body):
        """Corrections after the run: note, and placements added or removed while watching the video."""
        with self._lock:
            trial = self.get(trial_id)
            if "note" in body:
                trial["note"] = str(body["note"])[:500]
            if body.get("add") is not None:
                if body["add"] not in EVENT_KINDS:
                    raise EvalError("Unknown event.")
                t = body.get("t")
                trial["events"].append({"t": round(float(t), 2) if isinstance(t, (int, float)) else None, "kind": body["add"], "source": "operator"})
            if body.get("remove") is not None:   # the last mark of that kind
                idx = [i for i, e in enumerate(trial["events"]) if e["kind"] == body["remove"]]
                if not idx:
                    raise EvalError("There is nothing to remove.")
                trial["events"].pop(idx[-1])
            trial["events"].sort(key=lambda e: (e["t"] is None, e["t"] or 0))
            trial["result"] = count(trial)
            self._write(trial)
            self.write_csv()
            return trial

    def write_csv(self):
        rows = []
        for t in sorted(self.trials(), key=lambda t: t["id"]):
            res, auto, timing = t.get("result") or {}, t.get("auto") or {}, t.get("timing") or {}
            row = {**{k: t.get(k, "") for k in CSV_FIELDS}, **{k: res.get(k, "") for k in ("placed", "placed_180", "seconds_per_cube")},
                   "counted_live": len(t["live"]["placed"]) if t.get("live") else "",
                   "recounted": len((t.get("detection") or {}).get("placed") or []) if (t.get("detection") or {}).get("state") == "done" else "",
                   "auto_still_s": auto.get("still_s", ""), "auto_hand_closings": auto.get("hand_closings", ""),
                   "predict_ms": timing.get("predict_ms", ""), "dropped_steps": timing.get("dropped_total", "")}
            rows.append({k: ("" if row.get(k) is None else row.get(k)) for k in CSV_FIELDS})
        tmp = self.folder / "trials.csv.tmp"
        with open(tmp, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            w.writeheader()
            w.writerows(rows)
        tmp.replace(self.folder / "trials.csv")

    def video(self, trial_id, cam):
        if not TRIAL_ID.match(trial_id or "") or cam not in self.cams:
            raise EvalError("No such video.")
        f = self.folder / trial_id / f"{cam}.mp4"
        if not f.is_file():
            raise EvalError("No such video.")
        return f

    def info(self):
        """For the page, once per second."""
        with self._lock:
            cur = None
            if self.current is not None:
                c, now = self.current, time.time()
                elapsed = ((c.get("ended") or now) - c["t0"]) if c.get("t0") else 0.0
                cur = {"id": c["id"], "run": c.get("run"), "ckpt": c.get("ckpt"),
                       "elapsed": round(elapsed, 1)}
            return {"state": self.state, "graded_seconds": GRADED_SECONDS, "kinds": EVENT_KINDS,
                    "end_reasons": END_REASONS, "current": cur, "last_id": self.last_id, "message": self.message, "folder": str(self.folder)}
