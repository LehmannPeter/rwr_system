"""Background work on recorded demos (APE): encode each new demo into its video archive as soon as it is saved, then
check its raw file, one demo at a time, at the lowest CPU priority. Starts the transfer to the Training machine when
it is asked for.

- Encoding comes first (it is what the transfer needs); checks run only when nothing is left to encode.
- Nothing runs while a policy runs or an evaluation records (a running encode is stopped and redone later), nor while
  a card-3 job (transfer, clear, evaluation upload) runs.
- While a demo is being recorded, reviewed or saved, the running task is suspended (SIGSTOP) and continued (SIGCONT)
  right after the save or discard, so its CPU and disk use cannot add stream gaps to the recording. Tests of that
  effect: APE_ENCODE_WHILE_RECORDING=1 turns this off. The transfer job encodes whatever is still missing itself.
- A demo whose encoding failed is not retried here (the next transfer retries it); one whose check crashed neither.
- Encoding progress is in data/archive/<session>/.<demo>.progress.json (written by encode_archive.py, whoever runs it).
"""
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
LOGGER_DIR = HERE.parent / "src" / "logger"
sys.path.insert(0, str(LOGGER_DIR))
sys.path.insert(0, str(HERE))
from archive_job import record_archive  # noqa: E402
from encode_archive import is_archived  # noqa: E402

NICE = ["nice", "-n", "19"]
ENCODE_THREADS = 4


def progress_of(archive_dir, stem):
    """{"stage", "done", "total"} of an encode in progress, or None (also when its process is gone)."""
    try:
        p = json.loads((Path(archive_dir) / f".{stem}.progress.json").read_text())
        os.kill(int(p.get("pid", 0)), 0)
        return p
    except Exception:
        return None


class Background:
    def __init__(self, station, paused, job_running, start_transfer, log_file, hold=lambda: ""):
        self.station = station
        self.paused = paused                    # () -> reason text or "" (policy running, ...): stop and redo later
        self.hold = hold                        # () -> reason text or "" (a demo is being recorded): suspend
        self.job_running = job_running          # () -> bool
        self.start_transfer = start_transfer    # (session) -> None, starts the transfer job
        self.log_file = Path(log_file)
        self.current = None                     # {"task", "session", "stem", "proc", "started"}
        self.wanted = []                        # sessions whose transfer was asked for, oldest first
        self.check_failed = set()               # (session, stem) whose check crashed without a result
        self.note = ""                          # why it is waiting, for the page
        self._wake = threading.Event()
        self._lock = threading.Lock()

    # -- called by the server ----------------------------------------------------------------
    def start(self):
        threading.Thread(target=self._loop, daemon=True, name="background").start()

    def stop(self):
        """APE is shutting down: stop the running task (it is redone at the next start) and do nothing more."""
        self.paused = lambda: "APE is stopping"
        c = self.current
        if c:
            try:
                os.killpg(c["proc"].pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    def suspend_now(self, reason="a demo is being recorded"):
        """Suspend the running task at once (called right before a recording starts, so not even the next poll
        overlaps with it). The loop continues it when the hold is over."""
        with self._lock:
            c = self.current
            if c and not c.get("held"):
                self._signal(c["proc"], signal.SIGSTOP)
                c["held"] = True
                self.note = f"paused: {reason}"
                self._log(f"{c['task']} {c['session']}/{c['stem']} suspended: {reason}")

    def wake(self):
        self._wake.set()

    def request_transfer(self, session):
        with self._lock:
            if session not in self.wanted:
                self.wanted.append(session)
        self.wake()

    def info(self):
        c = self.current
        return {"task": c["task"] if c else "", "session": c["session"] if c else "", "demo": c["stem"] if c else "",
                "transfer_wanted": list(self.wanted), "note": self.note}

    # -- work selection ----------------------------------------------------------------------
    def _raw_files(self):
        """Complete raw recordings (*.h5, never *.h5.part), the newest session first."""
        raw = self.station.raw
        if not raw.is_dir():
            return []
        sessions = sorted((d for d in raw.iterdir() if d.is_dir()), key=lambda d: d.stat().st_mtime, reverse=True)
        return [(d.name, f) for d in sessions for f in sorted(d.glob("*.h5"))]

    def _archive_failed(self, session, stem):
        try:
            s = json.loads((self.station.archive_status / session / f"{stem}.json").read_text())
            return s.get("archive", {}).get("ok") is False
        except Exception:
            return False

    def next_task(self):
        files = self._raw_files()
        for session, f in files:
            if not is_archived(f, self.station.data / "archive" / session) and not self._archive_failed(session, f.stem):
                return "encode", session, f
        for session, f in files:
            if (session, f.stem) in self.check_failed:
                continue
            if not (self.station.previews / session / f.stem / "summary.json").is_file():
                return "check", session, f
        return None

    # -- the loop -------------------------------------------------------------------------------
    def _loop(self):
        while True:
            try:
                self._tick()
            except Exception as exc:   # never let the thread die
                self._log(f"background error: {exc!r}")
            self._wake.wait(3.0)
            self._wake.clear()

    def _tick(self):
        reason = self.paused() or self.hold()
        if reason:
            self.note = f"paused: {reason}"
            return
        if self.job_running():
            self.note = "waiting for the job in card 3"
            return
        with self._lock:
            session = self.wanted.pop(0) if self.wanted else None
        if session:
            self.note = ""
            self._log(f"transfer of {session} started")
            self.start_transfer(session)
            return
        task = self.next_task()
        if not task:
            self.note = ""
            return
        self.note = ""
        kind, session, raw = task
        if kind == "encode":
            self._encode(session, raw)
        else:
            self._check(session, raw)

    def _run(self, kind, session, raw, argv):
        """Run one task in its own process group, stopped when a policy starts (a transfer that is asked for waits
        for it). Returns the exit code, or None if it was stopped."""
        log = open(self.log_file, "a")
        log.write(f"\n--- {time.strftime('%F %T')} {kind} {session}/{raw.stem}: {' '.join(map(str, argv))}\n")
        log.flush()
        # its own process group (stopped as a whole), but in APE's session, so whatever supervises APE sees it
        proc = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, process_group=0,
                                cwd=str(self.station.workspace))
        self.current = {"task": kind, "session": session, "stem": raw.stem, "proc": proc, "started": time.time()}
        stopped = False
        c = self.current
        signal_group = lambda sig: self._signal(proc, sig)
        try:
            while proc.poll() is None:
                reason = self.paused()
                if reason and not stopped:
                    self._log(f"{kind} {session}/{raw.stem} stopped: {reason} (redone later)")
                    with self._lock:
                        if c.get("held"):
                            signal_group(signal.SIGCONT)   # a stopped process only acts on SIGTERM once continued
                            c["held"] = False
                        signal_group(signal.SIGTERM)
                    stopped = True
                hold = "" if stopped else self.hold()
                if hold:
                    self.suspend_now(hold)
                elif c.get("held") and not stopped:
                    with self._lock:
                        signal_group(signal.SIGCONT); c["held"] = False
                    self.note = ""
                    self._log(f"{kind} {session}/{raw.stem} continued")
                time.sleep(0.1 if c.get("held") else 0.5)
        finally:
            with self._lock:
                if c.get("held"):
                    signal_group(signal.SIGCONT)
                self.current = None
            log.write(f"--- {time.strftime('%F %T')} exit {proc.returncode}\n")
            log.close()
        return None if stopped else proc.returncode

    @staticmethod
    def _signal(proc, sig):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            pass

    def _encode(self, session, raw):
        archive = self.station.data / "archive" / session
        argv = [*NICE, sys.executable, "-u", str(LOGGER_DIR / "encode_archive.py"), str(raw), "-o", str(archive),
                "--skip-existing", "--jobs", "1", "--threads", str(ENCODE_THREADS)]
        code = self._run("encode", session, raw, argv)
        if code is None:   # stopped: remove what the stopped encode left (redone from scratch later)
            for left in archive.glob(f".{raw.stem}.*"):
                shutil.rmtree(left, ignore_errors=True) if left.is_dir() else left.unlink(missing_ok=True)
            return
        record_archive(self.station.archive_status / session, archive, raw.parent, raw.stem,
                       failure=None if code == 0 else f"encoding failed (exit {code}); see work/ape_background.log")

    def _check(self, session, raw):
        out = self.station.previews / session / raw.stem
        out.parent.mkdir(parents=True, exist_ok=True)
        argv = [*NICE, sys.executable, str(self.station.inspect_script), str(raw), "--output", str(out)]
        code = self._run("check", session, raw, argv)
        if code is not None and not (out / "summary.json").is_file():
            self.check_failed.add((session, raw.stem))
            self._log(f"check of {session}/{raw.stem} gave no result (exit {code})")

    def _log(self, text):
        try:
            with open(self.log_file, "a") as f:
                f.write(f"{time.strftime('%F %T')} {text}\n")
        except OSError:
            pass
