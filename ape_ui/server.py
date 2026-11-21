#!/usr/bin/env python3
"""APE UI: RWR26 Task 2 station dashboard, start/stop processes, logs, robot status,
teleop and recording controls.

Run from the ROS workspace so rclpy and the workspace messages are available:

    cd ~/rwr_humble_ws && pixi run python src/rwr_system/ape_ui/server.py

The page is NOT a safety device. The physical stop button stays the only stop.
"""
import argparse
import asyncio
import base64
import fcntl
import os
import pty
import re
import shutil
import signal
import struct
import subprocess
import termios
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path

import json
import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

HERE = Path(__file__).resolve().parent
sys_path_added = str(HERE)
import sys
if sys_path_added not in sys.path:
    sys.path.insert(0, sys_path_added)
from station_data import CAMERAS, DataError, Station, rotate_for_display, safe
from policy_checks import Policies, PolicyError
WORKSPACE = Path(os.environ.get("RWR_WS", Path.home() / "rwr_humble_ws"))
DATA = WORKSPACE / "data"  # raw/<session>/, processed/<session>_10hz/, datasets/<version>/, manifest.csv
RAW = DATA / "raw"
station = Station(WORKSPACE, DATA)
# Trained policies: <POLICIES>/<run>/config.yaml and <run>/checkpoints/*.pth, as rwr_il writes them under runs/
POLICIES = Path(os.environ.get("APE_POLICY_DIR", WORKSPACE / "policies"))
policies = Policies(WORKSPACE, DATA, POLICIES, station.hand_cfg_dir)
INFERENCE_LOGS = WORKSPACE / "inference_logs"  # written by the inference node, relative to the launch directory
POLICY_COMMAND_TOPIC = "/model_inference_command"

ROS_ENV = {"ROS_DOMAIN_ID": os.environ.get("APE_ROS_DOMAIN_ID", "7"), "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}
SIDE = "right"
START_POSE_FILE = Path(os.environ.get("APE_START_POSE_FILE", Path.home() / ".ros" / f"teleop_start_pose_{SIDE}.json"))
MAX_MOVE_M = 0.80  # sanity limit for the smooth moves (they travel at about 10 cm/s)
CENTER_ADJUST_FILE = Path(os.environ.get("APE_CENTER_ADJUST_FILE", Path.home() / ".ros" / f"teleop_center_adjust_{SIDE}.json"))
# Franka joint limits in radians (lower, upper), for the "close to a joint limit" warnings
JOINT_LIMITS = [(-2.8973, 2.8973), (-1.7628, 1.7628), (-2.8973, 2.8973), (-3.0718, -0.0698), (-2.8973, 2.8973), (-0.0175, 3.7525), (-2.8973, 2.8973)]
# Station correction read by the teleop node at every resume: {"yaw_deg": 180} reverses sideways and forward/back
COIL_ADJUST_FILE = Path(os.environ.get("APE_COIL_ADJUST_FILE", Path.home() / ".ros" / f"teleop_coil_adjust_{SIDE}.json"))
SETTINGS_FILE = Path(os.environ.get("APE_SETTINGS_FILE", Path.home() / ".ros" / "ape_ui_settings.json"))
HAND_SETTLE_S = 1.0  # "Start teleop + recording": time for the fingers to reach the glove pose before the arm and the recording start
CENTER_POS = (0.45, 0.0, 0.40)  # must match the teleop node's center_position parameter

NUC_HOST = "paco"
NUC_ENTER_CONTAINER = "cd ~/multipanda_ros2 && ./run"
# The robot's address is station-specific and must stay out of git (rwr_system/CLAUDE.md). It is kept in
# static/local/station.json as {"robot_ip": "..."}: "local/" is git-ignored, and the page reads the same file
# for its Franka Desk button.
STATION_FILE = HERE / "static" / "local" / "station.json"


def robot_ip():
    """The Franka's address, or "" when it is not set on this machine. Read at every use, so setting it needs no restart."""
    try:
        value = str(json.loads(STATION_FILE.read_text()).get("robot_ip") or "").strip()
    except (OSError, ValueError, AttributeError):
        return ""
    return value if re.fullmatch(r"[0-9A-Za-z.:-]+", value) else ""  # it is typed into a shell on the NUC


def nuc_launch():
    """The command typed into the NUC container to start the controller, or "" when the robot's address is not set."""
    ip = robot_ip()
    if not ip:
        return ""
    # The marker is split with quotes so the typed command does not itself match the marker.
    return (
        "source ~/multipanda_ws/install/setup.bash; "
        'if pgrep -af ros2_control_node; then echo UI_OLD"_"CONTROLLER; '
        f"else ros2 launch franka_bringup multimode_franka.launch.py robot_ip_1:={ip}; fi"
    )

ANSI = re.compile(r"\x1b\][^\x07\x1b]*(?:\x07|\x1b\\)|\x1b\[[0-9;?]*[ -/]*[@-~]|\x1b[=>()][0-9A-B]?")
PROMPT_CONTAINER = re.compile(r"developer@[\w.-]+:[^\n]*[$#] $")
PROMPT_ANY = re.compile(r"[\w.-]+@[\w.-]+:[^\n]*[$#] $")
DATASET_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")


class PtyProcess:
    """A command running in its own pseudo-terminal, with a replayable output buffer."""

    MAX_BYTES = 2_000_000

    def __init__(self, name):
        self.name = name
        self.pid = None
        self.fd = None
        self.exit_code = None
        self.started_at = None
        self.stopping = False
        self.command = ""
        self._chunks = []  # (seq, bytes)
        self._seq = 0
        self._size = 0
        self._lock = threading.Lock()

    # -- lifecycle ---------------------------------------------------------
    @property
    def running(self):
        return self.pid is not None

    def start(self, argv, cwd=None, env=None):
        if self.running:
            raise RuntimeError(f"{self.name} is already running")
        self.exit_code = None
        self.stopping = False
        self.command = " ".join(argv)
        self._append(f"\r\n\x1b[36m--- {time.strftime('%H:%M:%S')} starting: {self.command} ---\x1b[0m\r\n".encode())
        full_env = dict(os.environ)
        full_env.update(env or {})
        full_env.setdefault("TERM", "xterm-256color")
        pid, fd = pty.fork()
        if pid == 0:  # child: becomes session leader with the pty as controlling terminal
            try:
                if cwd:
                    os.chdir(cwd)
                os.execvpe(argv[0], argv, full_env)
            finally:
                os._exit(127)
        self.pid, self.fd = pid, fd
        self.started_at = time.time()
        self.resize(40, 160)
        threading.Thread(target=self._reader, args=(pid, fd), daemon=True).start()

    def _reader(self, pid, fd):
        while True:
            try:
                data = os.read(fd, 65536)
            except OSError:
                data = b""
            if not data:
                break
            self._append(data)
            self.on_output(data)
        try:
            _, status = os.waitpid(pid, 0)
            code = os.waitstatus_to_exitcode(status)
        except ChildProcessError:
            code = None
        try:
            os.close(fd)
        except OSError:
            pass
        self.exit_code = code
        self.pid, self.fd = None, None
        self._append(f"\r\n\x1b[36m--- {time.strftime('%H:%M:%S')} ended (exit code {code}) ---\x1b[0m\r\n".encode())
        self.on_exit()

    def on_output(self, data):
        pass

    def on_exit(self):
        pass

    # -- I/O ---------------------------------------------------------------
    def _append(self, data):
        with self._lock:
            self._seq += 1
            self._chunks.append((self._seq, data))
            self._size += len(data)
            while self._size > self.MAX_BYTES and len(self._chunks) > 1:
                self._size -= len(self._chunks.pop(0)[1])

    def read_since(self, seq):
        with self._lock:
            return [(s, d) for s, d in self._chunks if s > seq]

    def write(self, data: bytes):
        if self.fd is not None:
            os.write(self.fd, data)

    def resize(self, rows, cols):
        if self.fd is not None:
            fcntl.ioctl(self.fd, termios.TIOCSWINSZ, struct.pack("HHHH", rows, cols, 0, 0))

    def interrupt(self):
        """Same as pressing Ctrl-C in the terminal."""
        self.write(b"\x03")

    def stop(self, grace=20.0):
        """Ctrl-C, then escalate if the process does not exit. Returns immediately."""
        if not self.running or self.stopping:
            return
        self.stopping = True
        pid = self.pid
        self.interrupt()

        def escalate():
            for sig, wait in ((signal.SIGTERM, grace), (signal.SIGKILL, 8.0)):
                end = time.time() + wait
                while time.time() < end:
                    if self.pid != pid:
                        return
                    time.sleep(0.2)
                try:
                    self._append(f"\r\n\x1b[33m--- no exit yet, sending {sig.name} ---\x1b[0m\r\n".encode())
                    os.killpg(pid, sig)
                except ProcessLookupError:
                    return

        threading.Thread(target=escalate, daemon=True).start()

    def info(self):
        return {
            "running": self.running,
            "stopping": self.stopping and self.running,
            "exit_code": self.exit_code,
            "started_at": self.started_at if self.running else None,
            "command": self.command,
            "age": (time.time() - self.started_at) if self.running and self.started_at else None,
            "hand": getattr(self, "hand", None),
            "mode": getattr(self, "mode", None),
        }


class NucSession(PtyProcess):
    """ssh session to the NUC. The user gives the password; the rest is automated.

    States: disconnected -> connecting -> password -> auth -> host -> container -> launching -> running
    The ssh session stays open after the controller stops, so a restart needs no password.
    """

    WARN_PATTERNS = ("libfranka", "aborted by reflex", "rejected", "[ERROR]", "[FATAL]", "Controller already loaded", "User stopped")

    def __init__(self):
        super().__init__("nuc")
        self.state = "disconnected"
        self.auto = False  # continue automatically to the controller launch
        self.warnings = []
        self.note = ""
        self.auth_failures = 0  # passwords ssh refused in this login attempt
        self._tail = ""
        self._sent_for = None

    def connect(self, auto=True):
        self.auto = auto
        self.warnings, self.note, self._tail, self._sent_for = [], "", "", None
        self.auth_failures = 0
        self.state = "connecting"
        self.start(["ssh", "-tt", "-o", "ServerAliveInterval=15", NUC_HOST])

    def start_controller(self):
        if not self.running:
            self.connect(auto=True)
            return
        if self.state in ("launching", "running"):
            raise RuntimeError("The controller is already starting or running")
        self.auto = True
        self.warnings, self.note, self._sent_for = [], "", None
        if self.state in ("host", "container"):
            self.write(b"\n")  # prints a fresh prompt, which triggers the next step

    def submit_password(self, password):
        """Answer ssh's password prompt. Only while ssh asks: anywhere else the text would
        land in a shell, where it is shown and kept in the history."""
        if self.state != "password":
            raise RuntimeError("The NUC is not asking for a password right now.")
        self._tail = ""   # the old prompt must not count as a new one
        self.state = "auth"
        self.write(password.encode() + b"\n")

    def stop_controller(self):
        self.auto = False
        self.restart_pending = False
        self.interrupt()

    def restart_controller(self):
        """One click for Stop controller + Start controller: needed after a reflex or the stop button."""
        if not self.running or self.state not in ("launching", "running"):
            self.start_controller()
            return
        self.warnings, self.note, self._sent_for = [], "", None
        self.auto = False
        self.restart_pending = True   # launch again as soon as the container prompt is back
        self.interrupt()

    def _send(self, key, command):
        if self._sent_for == key:
            return
        self._sent_for = key
        self._tail = ""
        self.write(command.encode() + b"\n")

    def on_output(self, data):
        text = ANSI.sub("", data.decode(errors="replace")).replace("\r", "")
        self._tail = (self._tail + text)[-4000:]
        tail = self._tail

        for line in text.split("\n"):
            if any(p in line for p in self.WARN_PATTERNS) and line.strip() not in self.warnings:
                self.warnings = (self.warnings + [line.strip()[:300]])[:6]  # keep the first ones: they are the cause

        if "UI_OLD_CONTROLLER" in text:
            self.auto = False
            self.note = "An old controller is still running on the NUC. Wait a few seconds and press Start again."
        if "Connection to FCI refused" in text:
            self.note = "FCI is not active. Activate FCI in Franka Desk, then press Start controller again."
        elif "aborted by reflex" in text:
            self.note = "A reflex stopped the arm. Follow the recovery steps in the robot card, then restart the controller."
        elif '("User stopped")' in text or "(User stopped)" in text:
            self.note = "The black stop button is pressed. Release it fully, then restart the controller."
        if "Configured and activated multi_mode_controller" in text:
            self.state = "running"
        if self.state in ("password", "auth") and "Permission denied" in text:
            self.auth_failures += 1
        if re.search(r"password:\s*$", tail, re.I) or "passphrase" in tail[-80:].lower():
            self.state = "password"
            return
        if PROMPT_CONTAINER.search(tail):
            self.state = "container"
            if getattr(self, "restart_pending", False):
                self.restart_pending = False
                self.auto = True
                self._sent_for = None
                time.sleep(1.0)   # give the old controller a moment to release the robot
            if self.auto:
                launch = nuc_launch()
                if launch:
                    self.state = "launching"
                    self._send("launch", launch)
                else:
                    self.note = f"The robot's address is not set on this laptop. Write it into {STATION_FILE} (see the README), then press Start controller again."
                    self._sent_for = None
                self.auto = False  # never relaunch by itself after a stop or a crash
            else:
                self._sent_for = None
        elif PROMPT_ANY.search(tail):
            self.state = "host"
            if self.auto:
                self._send("enter", NUC_ENTER_CONTAINER)

    def on_exit(self):
        if not self.stopping:  # ssh gave up by itself: say why, the terminal may be folded away
            if self.state in ("password", "auth") and self.auth_failures:
                self.note = "The NUC refused the password too often and closed the connection. Press Start controller to try again."
            elif self.state == "connecting":
                last = next((l.strip() for l in reversed(self._tail.split("\n")) if l.strip()), "")
                self.note = "Could not connect to the NUC" + (f": {last[:200]}" if last else ".")
        self.state, self.auto = "disconnected", False
        self.restart_pending = False

    def info(self):
        d = super().info()
        if self.state != getattr(self, "_last_state", None):
            self._last_state, self._state_since = self.state, time.time()
        d.update(state=self.state, warnings=self.warnings, note=self.note, state_age=time.time() - self._state_since,
                 auth_failures=self.auth_failures)
        return d


class ControlError(Exception):
    pass


class InferenceProc(PtyProcess):
    """The policy inference launch. The inference node does not report its state on a topic,
    so what it is doing is read from its log."""

    ERROR_MARKS = ("Traceback", "Error:", "[ERROR]", "[FATAL]", "process has died")

    def __init__(self):
        super().__init__("inference")
        self.preview, self.run, self.ckpt, self.arm_init = True, None, None, False
        self.init_pose = None      # where this launch's init_arm goes: the center pose as [x, y, z, qx, qy, qz, qw]
        self._clear()

    def _clear(self):
        self.policy_on = False     # between the node's "Policy started." and "Policy stopped."
        self.policy_since = None
        self.init = ""             # arm move to the center pose: "", "moving", "done" or "failed: <why>"
        self.problems = []         # the first error lines of the log: they are the cause
        self.pauses, self.last_pause = 0, None   # "Policy output paused" warnings since the policy was started
        self.slow = 0
        self.note = ""
        self.marked = False        # recorded as "tried in preview"
        self._rest = ""
        self._seen_none = False    # no listener of a previous launch is left on the command topic
        self._init_asked = None    # when APE asked for the move to the center pose, until the node confirms it

    def begin(self, argv, run, ckpt, preview, arm_init, init_pose=None):
        if self.running:
            raise RuntimeError("The policy launch is already running")
        self._clear()
        self.run, self.ckpt, self.preview, self.arm_init = run, ckpt, bool(preview), bool(arm_init)
        self.init_pose = list(init_pose) if arm_init and init_pose else None
        self.start(argv, cwd=WORKSPACE, env=ROS_ENV)

    def on_output(self, data):
        lines = (self._rest + ANSI.sub("", data.decode(errors="replace")).replace("\r", "")).split("\n")
        self._rest = lines.pop()[-2000:]
        for line in lines:
            if "Policy started." in line:
                self.policy_on, self.policy_since, self.pauses, self.last_pause, self.slow, self.note = True, time.time(), 0, None, 0, ""
            elif "Policy stopped." in line:
                self.policy_on = False
            elif "Arm initialization started" in line:
                self.init, self._init_asked = "moving", None
            elif "Arm reached init pose" in line:
                self.init = "done"
            elif "Arm initialization stopped:" in line:
                self.init = "failed: " + line.split("Arm initialization stopped:", 1)[1].strip().rstrip(".")
            elif "init_arm requires" in line:
                self.init = "failed: the inference node refused the move"
            if "Policy output paused:" in line:   # the node prints this at most every 2 s
                self.pauses += 1
                self.last_pause = (time.time(), line.split("Policy output paused:", 1)[1].strip()[:200])
            elif "Policy execution is too slow" in line:
                self.slow += 1
            elif any(m in line for m in self.ERROR_MARKS) and "Arm initialization stopped" not in line:
                text = line.strip()[-300:]
                if text not in self.problems:
                    self.problems = (self.problems + [text])[:6]

    def on_exit(self):
        self.policy_on = False
        if self.init == "moving":
            self.init = ""

    def ask_init(self):
        self.init, self._init_asked = "moving", time.time()

    def node_ready(self):
        """True once the inference node of THIS launch listens for commands: it does so after its policy is loaded.
        A listener left over from a launch stopped a moment ago does not count."""
        count = monitor.policy_listeners() if monitor and self.running else 0
        if not count:
            self._seen_none = self.running
        if self.init == "moving" and self._init_asked and time.time() - self._init_asked > 3.0:
            self.init, self._init_asked = "failed: the inference node did not answer", None
        return bool(count and (self._seen_none or (self.started_at and time.time() - self.started_at > 15.0)))

    def info(self):
        d = super().info()
        d.update(
            preview=self.preview, run=self.run, ckpt=self.ckpt, arm_init=self.arm_init,
            init_target="center" if self.arm_init and self.init_pose else None, policy_on=self.policy_on and self.running,
            policy_for=(time.time() - self.policy_since) if self.policy_on and self.policy_since else None,
            init=self.init, problems=self.problems, pauses=self.pauses, slow=self.slow, note=self.note,
            last_pause={"age": time.time() - self.last_pause[0], "text": self.last_pause[1]} if self.last_pause else None,
            node_ready=self.node_ready(),
            producing=bool(monitor and monitor.policy_running),
        )
        return d


class RosMonitor:
    """ROS node in a background thread: reads status, and calls the same services the
    keyboard runner calls. It never publishes arm or hand commands itself."""

    SERVICES = {
        "resume": (f"/franka_teleop/{SIDE}/resume", "Trigger"),
        "pause": (f"/franka_teleop/{SIDE}/pause", "Trigger"),
        "move2init": (f"/franka_teleop/{SIDE}/move2init", "Trigger"),
        "move2center": (f"/franka_teleop/{SIDE}/move2center", "Trigger"),
        "accurate": (f"/franka_teleop/{SIDE}/set_accurate_mode", "SetBool"),
        "set_start": (f"/franka_teleop/{SIDE}/set_start_pose_here", "Trigger"),
        "hand_follow": (f"/{SIDE}/retargeter/set_enabled", "SetBool"),
        "hand_torque": (f"/{SIDE}/hand/set_torque_enabled", "SetBool"),
        "rec_start": ("/demo_logger/start_recording", "Trigger"),
        "rec_stop": ("/demo_logger/stop_recording", "Trigger"),
        "rec_save": ("/demo_logger/save_recording", "Trigger"),
    }

    MODES = {0: "other", 1: "idle", 2: "ready", 3: "guiding", 4: "reflex", 5: "user_stopped", 6: "recovering"}

    def __init__(self):
        self.ok = False
        self.error = ""
        self.robot = {}
        self.robot_stamp = 0.0
        self.checks = {}
        self._glove_last = None
        self._glove_msgs = []
        self._glove_updates = []
        self._cmd_times = []
        self._hand_times = []
        self.arm_pos = None
        self.arm_stamp = 0.0
        self.tele = {}       # latest values per stream, for the live panels
        self.stamps = {}     # stream name -> recent arrival times, for the rates
        self.frames = {}     # camera name -> (arrival time, latest Image message)
        self.move_until = 0.0  # a smooth move started from this page is in progress until then
        self.move_what = ""
        self._clients = {}
        self._srv_types = {}
        self._policy_pub = None
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            os.environ.update(ROS_ENV)
            import rclpy
            from franka_msgs.msg import FrankaState
            from geometry_msgs.msg import PoseStamped
            from std_msgs.msg import Float32MultiArray, String
            from std_srvs.srv import SetBool, Trigger
            from rclpy.executors import ExternalShutdownException
            from rclpy.signals import SignalHandlerOptions

            rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
            node = rclpy.create_node("ape_ui_monitor")

            def mark(name):
                self.stamps.setdefault(name, []).append(time.time())

            def on_state(msg):
                mark("arm_state")
                self.tele["franka"] = {
                    "q": [round(float(v), 4) for v in msg.q],
                    "dq": [round(float(v), 4) for v in msg.dq],
                    "tau": [round(float(v), 3) for v in msg.tau_j],
                    "tau_ext": [round(float(v), 3) for v in msg.tau_ext_hat_filtered],
                    "wrench": [round(float(v), 2) for v in msg.o_f_ext_hat_k],
                    "joint_contact": [int(v > 0) for v in msg.joint_contact],
                    "joint_collision": [int(v > 0) for v in msg.joint_collision],
                    "cart_contact": [int(v > 0) for v in msg.cartesian_contact],
                    "cart_collision": [int(v > 0) for v in msg.cartesian_collision],
                    "m_ee": round(float(msg.m_ee), 3),
                    "com_ee": [round(float(v), 3) for v in msg.f_x_cee],
                    "m_load": round(float(msg.m_load), 3),
                }
                # End-effector and load parameters as set in Franka Desk; stored with every saved demo.
                self.ee_params = {
                    "m_ee": float(msg.m_ee), "f_x_cee": [float(v) for v in msg.f_x_cee], "i_ee": [float(v) for v in msg.i_ee],
                    "f_t_ee": [float(v) for v in msg.f_t_ee], "ee_t_k": [float(v) for v in msg.ee_t_k],
                    "m_load": float(msg.m_load), "f_x_cload": [float(v) for v in msg.f_x_cload], "i_load": [float(v) for v in msg.i_load],
                    "read_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                }

                def active(errors):
                    return [f for f in errors.get_fields_and_field_types() if getattr(errors, f) is True]

                self.robot = {
                    "mode": self.MODES.get(msg.robot_mode, "other"),
                    "mode_id": int(msg.robot_mode),
                    "success_rate": float(msg.control_command_success_rate),
                    "current_errors": active(msg.current_errors),
                    "last_motion_errors": active(msg.last_motion_errors),
                }
                self.robot_stamp = time.time()

            def pose7(msg):
                p, o = msg.pose.position, msg.pose.orientation
                return [round(v, 4) for v in (p.x, p.y, p.z, o.x, o.y, o.z, o.w)]

            def floats(name, digits):
                def cb(msg):
                    mark(name)
                    self.tele[name] = [round(float(v), digits) for v in msg.data]
                return cb

            def on_cmd(msg):
                self._cmd_times.append(time.time())
                mark("arm_cmd")
                self.tele["arm_cmd"] = pose7(msg)

            def on_hand_cmd(msg):
                self._hand_times.append(time.time())
                mark("hand_cmd")
                self.tele["hand_cmd"] = [round(float(v), 4) for v in msg.data]

            def on_image(cam):
                def cb(msg):
                    mark(cam)
                    self.frames[cam] = (time.time(), msg)
                return cb

            def on_glove(msg):
                mark("glove")
                now = time.time()
                p = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
                self._glove_msgs.append(now)
                if p != self._glove_last:
                    self._glove_updates.append(now)
                    self._glove_last = p

            def on_timer():
                now = time.time()
                self._glove_msgs = [t for t in self._glove_msgs if now - t < 3]
                self._glove_updates = [t for t in self._glove_updates if now - t < 3]
                for name, times in list(self.stamps.items()):
                    self.stamps[name] = [t for t in times if now - t < 3.0][-400:]
                self._cmd_times = [t for t in self._cmd_times if now - t < 1.0]
                self._hand_times = [t for t in self._hand_times if now - t < 1.0]
                listeners = node.get_subscriptions_info_by_topic(f"/{SIDE}/arm/end_effector_pose_cmd")
                self.checks = {
                    "controller_listening": sum(1 for i in listeners if i.node_namespace.startswith("/paco")),
                    "teleop_publishing": node.count_publishers(f"/{SIDE}/arm/end_effector_pose_cmd"),
                    "arm_cmd_hz": len(self._cmd_times),
                    "hand_cmd_hz": len(self._hand_times),
                    "services": {k: c.service_is_ready() for k, c in self._clients.items()},
                    "cameras": {c: node.count_publishers(f"/oakd_{c}/color") for c in ("front_view", "wrist_view")},
                    "glove_publish_hz": round(len(self._glove_msgs) / 3, 1),
                    "glove_update_hz": round(len(self._glove_updates) / 3, 1),
                    "nodes": sorted(n for n in node.get_node_names() if not n.startswith("_") and n != "ape_ui_monitor"),
                }

            node.create_subscription(FrankaState, "/paco/franka_robot_state_broadcaster/robot_state", on_state, 5)
            node.create_subscription(PoseStamped, f"/ingress/{SIDE}/wrist", on_glove, 50)
            node.create_subscription(PoseStamped, f"/{SIDE}/arm/end_effector_pose_cmd", on_cmd, 50)

            def on_arm(msg):
                self.arm_pos = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
                self.arm_stamp = time.time()
                mark("arm_pose")
                self.tele["arm"] = pose7(msg)

            node.create_subscription(PoseStamped, f"/{SIDE}/arm/end_effector_pose", on_arm, 5)
            node.create_subscription(Float32MultiArray, f"/{SIDE}/hand/policy_output", on_hand_cmd, 50)
            node.create_subscription(Float32MultiArray, f"/{SIDE}/hand/joint_position_from_motor", floats("hand", 4), 5)
            node.create_subscription(Float32MultiArray, f"/{SIDE}/hand/motor_current", floats("hand_current", 0), 5)
            node.create_subscription(Float32MultiArray, f"/{SIDE}/hand/motor_temperature", floats("hand_temp", 0), 5)

            def on_policy_arm(msg):   # what the policy outputs; in preview it goes nowhere else
                mark("policy_arm")
                self.tele["policy_arm"] = pose7(msg)

            node.create_subscription(PoseStamped, "/policy_inference/actions_arm", on_policy_arm, 10)
            node.create_subscription(Float32MultiArray, "/policy_inference/actions_hand", floats("policy_hand", 4), 10)
            self._String = String
            self._policy_pub = node.create_publisher(String, POLICY_COMMAND_TOPIC, 10)
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import Image
            image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            for cam in CAMERAS:
                node.create_subscription(Image, f"/{cam}/color", on_image(cam), image_qos)
            self._srv_types = {"Trigger": Trigger, "SetBool": SetBool}
            for key, (name, kind) in self.SERVICES.items():
                self._clients[key] = node.create_client(self._srv_types[kind], name)
            node.create_timer(1.0, on_timer)
            self.ok = True
            try:
                rclpy.spin(node)
            except ExternalShutdownException:
                pass
        except Exception as exc:  # the page still works for start/stop without ROS
            self.error = f"{type(exc).__name__}: {exc}"
            self.ok = False

    # -- live data -------------------------------------------------------------
    def rates(self):
        """Per stream: messages per second over the last 2 s, the largest gap in that window (ms) and its age (s)."""
        now, out = time.time(), {}
        for name, times in list(self.stamps.items()):
            recent = [t for t in times if now - t < 2.0]
            if not times:
                continue
            gaps = [b - a for a, b in zip(recent, recent[1:])]
            out[name] = {"hz": round(len(recent) / 2.0, 1), "max_gap_ms": round(max(gaps) * 1000) if gaps else None, "age": round(now - times[-1], 1)}
        return out

    def live(self):
        now = time.time()

        def get(key, stream, limit=1.5):
            """Latest value stored under `key`, if its stream delivered a message recently."""
            times = self.stamps.get(stream)
            return self.tele.get(key) if times and now - times[-1] < limit else None

        policy = self.policy_running   # then show what the policy outputs as the command, also in preview
        return {
            "t": round(now, 2),
            "arm": get("arm", "arm_pose"),
            "arm_cmd": get("policy_arm", "policy_arm") if policy else get("arm_cmd", "arm_cmd", 1e9),   # the controller keeps holding the last target, so keep showing it
            "hand": get("hand", "hand"),
            "hand_cmd": get("policy_hand", "policy_hand") if policy else get("hand_cmd", "hand_cmd"),
            "policy": policy,
            "hand_current": get("hand_current", "hand_current"),
            "hand_temp": get("hand_temp", "hand_temp", 8.0),
            "franka": get("franka", "arm_state"),
        }

    def jpeg(self, cam, width=960):
        """Latest frame of a camera as JPEG bytes, or None if there is no recent frame."""
        import cv2
        import numpy as np

        item = self.frames.get(cam)
        if not item or time.time() - item[0] > 2.0:
            return None
        msg = item[1]
        channels = 3 if msg.encoding in ("rgb8", "bgr8") else 1
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, : msg.width * channels]
        img = img.reshape(msg.height, msg.width, channels) if channels == 3 else img
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if img.shape[1] > width:
            img = cv2.resize(img, (width, int(round(img.shape[0] * width / img.shape[1]))), interpolation=cv2.INTER_AREA)
        img = rotate_for_display(cam, img)  # display only: the recorded data is not turned
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
        return buf.tobytes() if ok else None

    # -- control ---------------------------------------------------------------
    def ready(self, key):
        c = self._clients.get(key)
        return bool(c and c.service_is_ready())

    def call(self, key, data=None, timeout=5.0):
        """Call one of SERVICES and return (success, message). Raises ControlError if it cannot."""
        name, kind = self.SERVICES[key]
        cli = self._clients.get(key)
        if cli is None or not cli.service_is_ready():
            raise ControlError(f"{name} is not available. Is the launch running?")
        req = self._srv_types[kind].Request()
        if kind == "SetBool":
            req.data = bool(data)
        done = threading.Event()
        fut = cli.call_async(req)
        fut.add_done_callback(lambda f: done.set())
        if not done.wait(timeout):
            fut.cancel()
            raise ControlError(f"{name} did not answer within {timeout:.0f} s")
        res = fut.result()
        return bool(res.success), str(getattr(res, "message", ""))

    def policy_command(self, text):
        """Send start, stop, reset or init_arm to the inference node (what `ros2 topic pub /model_inference_command` does)."""
        if self._policy_pub is None:
            raise ControlError("ROS is not available, so the policy cannot be commanded.")
        msg = self._String()
        msg.data = text
        self._policy_pub.publish(msg)

    def policy_listeners(self):
        """Nodes listening for policy commands: the inference node does once its policy is loaded."""
        return self._policy_pub.get_subscription_count() if self._policy_pub is not None else 0

    @property
    def policy_running(self):
        """True while the inference node produces actions (in preview they stay on /policy_inference)."""
        now = time.time()
        return len([t for t in self.stamps.get("policy_arm") or [] if now - t < 1.0]) >= 3

    @property
    def commands_flowing(self):
        """True while arm commands are really being sent (teleop running, or a smooth move)."""
        return len([t for t in self._cmd_times if time.time() - t < 0.5]) >= 5

    @property
    def moving(self):
        return time.time() < self.move_until

    @property
    def teleop_running(self):
        """True while the glove drives the arm: commands flow and it is not one of our smooth moves."""
        return self.commands_flowing and not self.moving

    @property
    def hand_following(self):
        return len([t for t in self._hand_times if time.time() - t < 0.5]) >= 5

    def info(self):
        age = time.time() - self.robot_stamp if self.robot_stamp else None
        fresh = age is not None and age < 2.0
        return {
            "ros_ok": self.ok,
            "ros_error": self.error,
            "robot": self.robot if fresh else None,
            "robot_age": age,
            "checks": self.checks,
        }


def ros2_cmd(*args):
    """Use ros2 directly when the server runs inside the pixi env, else go through pixi."""
    if shutil.which("ros2"):
        return ["ros2", *args]
    return ["pixi", "run", "ros2", *args]


def external_launches(own_pids):
    """Launches of the same kind that were started outside this page (e.g. in a terminal)."""
    try:
        out = subprocess.run(["pgrep", "-af", r"ros2 launch experiments"], capture_output=True, text=True, timeout=3).stdout
    except Exception:
        return []
    found = []
    for line in out.splitlines():
        pid, _, cmd = line.partition(" ")
        if not pid.isdigit() or "pgrep" in cmd:
            continue
        pid = int(pid)
        try:
            sid = os.getsid(pid)
        except ProcessLookupError:
            continue
        if sid in own_pids or pid in own_pids:
            continue
        kind = "recording" if "record_demonstration" in cmd else "teleop" if "teleop_rokoko" in cmd else "inference" if "run_policy_inference" in cmd else "other"
        if kind not in found:
            found.append(kind)
    return found


class Operate:
    """What the keyboard runner did, as button actions. Recording state is tracked here
    because the logger node does not report it."""

    def __init__(self):
        self.rec = "idle"  # idle | recording | review | saving
        self.rec_started = None
        self.rec_duration = None
        self.last_saved = None
        self.message = ""
        self.accurate = False
        self.torque = True
        self.dataset = None
        self.relax_after_start = False
        self.move_check = None
        self._lock = threading.Lock()
        self.after_stop = "off"   # where the arm goes after "Stop recording and pause": off | center | start
        try:
            value = json.loads(SETTINGS_FILE.read_text()).get("after_stop", "off")
            self.after_stop = value if value in ("off", "center", "start") else "off"
        except Exception:
            pass

    def set_after_stop(self, value):
        if value not in ("off", "center", "start"):
            raise ControlError("Choose off, center or start.")
        if value == "start" and not self.start_pose()["saved"]:
            raise ControlError("No custom pose is saved yet. Set one first.")
        self.after_stop = value
        try:
            SETTINGS_FILE.parent.mkdir(parents=True, exist_ok=True)
            SETTINGS_FILE.write_text(json.dumps({"after_stop": value}, indent=2))
        except OSError:
            pass
        return ""

    def reset(self, dataset=None):
        self.rec, self.rec_started, self.rec_duration = "idle", None, None
        self.message, self.accurate, self.torque, self.dataset = "", False, True, dataset
        self.relax_after_start = True   # switch the hand torque off once the hand node is up
        self.move_check = None          # (time when a smooth move ends, what it was)

    def _torque(self, on):
        """Hand torque on/off if the hand node is there. Returns a note on failure, never raises."""
        if not monitor.ready("hand_torque"):
            return ""
        try:
            ok, msg = monitor.call("hand_torque", on)
            if ok:
                self.torque = bool(on)
                return ""
            return f"Hand torque could not be switched {'on' if on else 'off'}: {msg}"
        except ControlError as exc:
            return str(exc)

    def supervise(self):
        """Background loop: keeps the hand relaxed whenever it is neither teleoperated nor moving to a pose."""
        import math

        while True:
            time.sleep(0.3)
            try:
                if monitor is None:
                    continue
                if getattr(self, "relax_after_start", False) and monitor.ready("hand_torque"):
                    time.sleep(1.5)  # the hand node has just finished moving to its neutral pose
                    self.relax_after_start = False
                    if not monitor.commands_flowing and not monitor.hand_following:
                        self._torque(False)
                check = getattr(self, "move_check", None)
                if check and time.time() > check[0] + 3.0:   # the move is over and the arm has settled
                    self.move_check = None
                    if not monitor.teleop_running and not monitor.hand_following:
                        self._torque(False)
                    arm, goal = monitor.tele.get("arm"), check[2]   # the true goal, not the (corrected) command
                    if arm and goal:
                        dot = abs(sum(a * b for a, b in zip(arm[3:], goal)))
                        off = math.degrees(2 * math.acos(min(1.0, dot)))
                        if off > 6.0:
                            near = self.joints_near_limit(25.0)
                            if near:
                                why = ("A joint close to its limit can block the turn: " + ", ".join(f"joint {j} ({max(m, 0):.0f}° left)" for j, m in near)
                                       + ". Untwist the arm (see Arm details) and move there again.")
                            else:
                                why = ("No joint is near its limit, so the arm is not blocked. The controller holds the hand's orientation only softly, "
                                       "and joint friction can leave it a few degrees short; which way depends on where the arm came from. "
                                       "Arm details shows commanded against measured.")
                            self.message = f"The arm reached the {check[1]} position, but its hand orientation is {off:.0f}° away from the target. {why}"
            except Exception:
                pass

    @staticmethod
    def joints_near_limit(margin_deg):
        import math

        fr = monitor.tele.get("franka") if monitor else None
        if not fr or not monitor.stamps.get("arm_state") or time.time() - monitor.stamps["arm_state"][-1] > 2.0:
            return []
        out = []
        for i, (q, (lo, hi)) in enumerate(zip(fr["q"], JOINT_LIMITS)):
            m = math.degrees(min(q - lo, hi - q))
            if m < margin_deg:
                out.append((i + 1, m))
        return out

    # teleop ---------------------------------------------------------------
    def teleop_start(self, record=False):
        if record and self.rec != "idle":
            raise ControlError("Save or discard the previous demo first." if self.rec == "review" else f"The logger is {self.rec}.")
        if record and not monitor.ready("rec_start"):
            raise ControlError("The logger is not running, so nothing can be recorded.")
        if monitor.moving:
            raise ControlError("The arm is still moving to a pose. Wait, or stop the move first.")
        robot = monitor.info()["robot"]
        notes = []
        if monitor.ready("resume") and not robot:
            raise ControlError("The Franka controller is not running. Start it first. (To test only the hand, use the Hand follows glove switch.)")
        if robot and robot["mode"] != "ready":
            raise ControlError(f"The robot is not ready ({robot['mode'].replace('_', ' ')}). Restart the controller first.")
        if not monitor.ready("resume") and not monitor.ready("hand_follow"):
            raise ControlError("Neither arm teleop nor hand tracking is available. Start the launch first.")
        self.relax_after_start = False
        note = self._torque(True)   # the hand is only powered while teleop runs or the arm moves to a pose
        if note:
            notes.append(note)
        if record:
            # Recording must begin AT the start pose, without the fingers' power-on snap in it:
            # let the fingers reach the glove pose first, then start the arm and the recording together.
            if monitor.ready("hand_follow"):
                monitor.call("hand_follow", True)
                time.sleep(HAND_SETTLE_S)
            else:
                notes.append("Hand tracking is not available.")
            if monitor.ready("resume"):
                ok, _ = monitor.call("resume")
                if not ok:
                    self.teleop_pause()
                    raise ControlError("Arm teleop did not start: the teleop node has no robot pose or no glove data yet.")
            else:
                notes.append("Arm teleop is not running in this launch.")
            try:
                self.rec_start()
            except ControlError as exc:
                notes.append(f"Teleop is running, but the recording did not start: {exc}")
            return " ".join(notes)
        if monitor.ready("resume"):
            ok, _ = monitor.call("resume")
            if not ok:
                self._torque(False)
                raise ControlError("Arm teleop did not start: the teleop node has no robot pose or no glove data yet.")
        else:
            if not hand_only_launch():   # there it is the point, not a problem
                notes.append("Arm teleop is not running in this launch.")
        if monitor.ready("hand_follow"):
            monitor.call("hand_follow", True)
        else:
            notes.append("Hand tracking is not available.")
        return " ".join(notes)

    def teleop_pause(self, relax=True):
        monitor.move_until = 0.0
        errors = []
        for key, arg in (("pause", None), ("hand_follow", False)):
            if monitor.ready(key):
                try:
                    monitor.call(key, arg)
                except ControlError as exc:
                    errors.append(str(exc))
        note = self._torque(False) if relax else ""   # relax the hand while paused, so its motors can cool down
        if note:
            errors.append(note)
        if errors:
            raise ControlError(" ".join(errors))
        return ""

    def _move(self, key, what, target, goal_quat=None):
        if monitor.commands_flowing:
            raise ControlError("A move is already running." if monitor.moving else "Pause teleop first.")
        robot = monitor.info()["robot"]
        if not robot or robot["mode"] != "ready":
            raise ControlError("The robot is not ready.")
        dist = self._distance(target)
        if dist is None:
            raise ControlError("The arm position is unknown.")
        if dist > MAX_MOVE_M:
            raise ControlError(f"The {what} is {dist * 100:.0f} cm away, more than the {MAX_MOVE_M * 100:.0f} cm limit. Bring the arm closer first.")
        self.relax_after_start = False
        self._torque(True)   # keep the fingers from flopping while the arm moves
        ok, msg = monitor.call(key)
        if not ok:
            self._torque(False)
            raise ControlError(msg or "The teleop node refused: it has no robot pose yet, or teleop is running.")
        try:
            duration = float(msg)
        except ValueError:
            duration = dist / 0.10 + 1.0
        monitor.move_until, monitor.move_what = time.time() + duration + 0.4, what
        self.move_check = (monitor.move_until, what, goal_quat)
        self.message = ""
        return f"The arm is moving to the {what} ({duration:.0f} s)."

    def move2init(self):
        sp = self.start_pose()
        if not sp["saved"]:
            raise ControlError("No custom pose is saved for this station yet. Bring the arm to the pose you want and press Set.")
        return self._move("move2init", "custom pose", sp["position"], sp.get("quat"))

    def center_pose(self):
        """The center pose as [x, y, z, qx, qy, qz, qw] in the world frame, as the teleop node builds it for move2center."""
        from scipy.spatial.transform import Rotation

        # the node's center orientation: palm straight down (180 deg about x), then the turn about the vertical axis
        goal = Rotation.from_euler("z", self.center_yaw(), degrees=True) * Rotation.from_euler("x", 180, degrees=True)
        return [float(v) for v in CENTER_POS] + [float(v) for v in goal.as_quat()]

    def move2center(self):
        return self._move("move2center", "center pose", CENTER_POS, self.center_pose()[3:])

    @staticmethod
    def center_yaw():
        import json

        try:
            yaw = float(json.loads(CENTER_ADJUST_FILE.read_text()).get("yaw_deg", 0.0))
            return yaw if abs(yaw) <= 45.0 else 0.0   # the node ignores values outside +-45 (nan fails this test too)
        except Exception:
            return 0.0

    def set_center_yaw(self, value):
        import json

        try:
            yaw = float(value)
        except (TypeError, ValueError):
            raise ControlError("Enter the correction as a number of degrees.")
        if not -45.0 <= yaw <= 45.0:
            raise ControlError("The correction must be between -45 and 45 degrees.")
        CENTER_ADJUST_FILE.parent.mkdir(parents=True, exist_ok=True)
        CENTER_ADJUST_FILE.write_text(json.dumps({"yaw_deg": yaw}, indent=2))
        return f"Center pose rotation correction set to {yaw:g} degrees. It applies at the next Move to center."

    @staticmethod
    def coil_yaw():
        """Yaw correction of the glove directions in degrees, as the teleop node will read it at the next resume."""
        try:
            yaw = float(json.loads(COIL_ADJUST_FILE.read_text()).get("yaw_deg", 0.0))
            return yaw if abs(yaw) <= 180.0 else 0.0   # the node ignores values outside +-180 (nan fails this test too)
        except Exception:
            return 0.0

    def set_coil_reverse(self, on):
        if monitor and monitor.teleop_running:
            raise ControlError("Pause teleop before changing the arm directions.")
        if self.rec != "idle":
            raise ControlError("Save or discard the current demo before changing the arm directions.")
        COIL_ADJUST_FILE.parent.mkdir(parents=True, exist_ok=True)
        COIL_ADJUST_FILE.write_text(json.dumps({"yaw_deg": 180.0 if on else 0.0}, indent=2))
        if on:
            return "Arm directions reversed: sideways and forward/back swap sides from the next Start teleop. Make the first moves small."
        return "Arm directions back to normal from the next Start teleop."

    def move_stop(self):
        """Stop a smooth move: pausing makes the node hold the last target."""
        monitor.call("pause")
        monitor.move_until = 0.0
        self.move_check = None
        self._torque(False)
        return "Move stopped."

    @staticmethod
    def _distance(target):
        import math

        if monitor and monitor.arm_pos and time.time() - monitor.arm_stamp < 2.0:
            return round(math.dist(target, monitor.arm_pos), 3)
        return None

    def set_start(self):
        if monitor.teleop_running:
            raise ControlError("Pause teleop first.")
        ok, msg = monitor.call("set_start")
        if not ok:
            raise ControlError(msg or "The teleop node refused.")
        return "Custom pose saved: the arm's current pose."

    def start_pose(self):
        """Saved start pose and its distance from the arm's current position."""
        import json
        import math

        out = {"saved": False, "position": None, "distance": None}
        try:
            data = json.loads(START_POSE_FILE.read_text())
            pos = [float(v) for v in data["position"]]
            out.update(saved=True, position=[round(v, 3) for v in pos], quat=[float(v) for v in data.get("quat_xyzw", [])] or None)
        except Exception:
            return out
        out["distance"] = self._distance(pos)
        return out

    def set_accurate(self, on):
        if monitor.teleop_running:
            raise ControlError("Pause teleop before changing the motion scale.")
        ok, msg = monitor.call("accurate", on)
        if not ok:
            raise ControlError(msg or "The teleop node refused.")
        self.accurate = bool(on)
        return f"Arm motion scale: {msg}"

    def set_torque(self, on):
        ok, msg = monitor.call("hand_torque", on)
        if not ok:
            raise ControlError(msg or "The hand node refused.")
        self.torque = bool(on)
        return ""

    # recording --------------------------------------------------------------
    def rec_start(self):
        with self._lock:
            if self.rec != "idle":
                raise ControlError("Save or discard the previous demo first." if self.rec == "review" else f"The logger is {self.rec}.")
            ok, _ = monitor.call("rec_start")
            if not ok:
                raise ControlError("The logger refused to start (it may still be saving).")
            self.rec, self.rec_started, self.rec_duration, self.message = "recording", time.time(), None, ""
        return ""

    def rec_stop(self):
        """Stop the recording, then pause teleop: the keyboard runner's order."""
        with self._lock:
            if self.rec != "recording":
                raise ControlError("No recording is running.")
            ok, _ = monitor.call("rec_stop")
            self.rec_duration = time.time() - self.rec_started
            self.rec = "review" if ok else "idle"
        note = "" if ok else "The logger reported no active recording."
        go = self.after_stop if self.after_stop in ("center", "start") else None
        try:
            # when the arm returns to a pose right away, keep the fingers powered through the move
            self.teleop_pause(relax=go is None)
        except ControlError as exc:
            note = (note + " " + str(exc)).strip()
        if go:
            threading.Thread(target=self._return_after_stop, args=(go,), daemon=True).start()
        return note

    def rec_stop_no_return(self):
        """Stop the recording and pause teleop without the automatic return move (used before a controller restart)."""
        keep, self.after_stop = self.after_stop, "off"
        try:
            return self.rec_stop()
        finally:
            self.after_stop = keep

    def _return_after_stop(self, where):
        """After a recording: move the arm to the chosen pose once teleop commands have stopped."""
        end = time.time() + 3.0
        while time.time() < end and monitor.commands_flowing:
            time.sleep(0.1)
        try:
            self.move2center() if where == "center" else self.move2init()
        except ControlError as exc:
            self._torque(False)
            self.message = f"The arm did not return to the {'center' if where == 'center' else 'custom'} pose: {exc}"

    def rec_discard(self):
        with self._lock:
            if self.rec != "review":
                raise ControlError("There is no stopped demo to discard.")
            self.rec, self.message = "idle", "Demo discarded."
        return ""

    def rec_save(self, labels=None):
        with self._lock:
            if self.rec != "review":
                raise ControlError("There is no stopped demo to save.")
            self.rec, self.message = "saving", ""

        def work():
            try:
                ok, _ = monitor.call("rec_save", timeout=1800)
                if ok:
                    files = self.files()
                    self.last_saved = files[-1]["name"] if files else None
                    self.message = f"Saved {self.last_saved}." if self.last_saved else "Saved."
                    problem = station.register(self.dataset, labels or {})
                    if self.last_saved:
                        extra = {
                            "arm_motion_scale": "accurate" if self.accurate else "normal",
                            "glove_direction_yaw_deg": self.coil_yaw(),
                            "end_effector_parameters": getattr(monitor, "ee_params", None),
                        }
                        problem = " ".join(p for p in (problem, station.write_meta(self.dataset, self.last_saved, labels or {}, extra)) if p)
                    if problem:
                        self.message += " " + problem
                else:
                    self.message = "The logger could not save: no recording found."
            except ControlError as exc:
                self.message = f"Saving failed: {exc}"
            self.rec = "idle"

        threading.Thread(target=work, daemon=True).start()
        return ""

    def files(self):
        if not self.dataset:
            return []
        folder = RAW / self.dataset
        if not folder.is_dir():
            return []
        out = [{"name": f.name, "gb": round(f.stat().st_size / 1e9, 2)} for f in sorted(folder.glob("*.h5"))]
        return out

    def info(self):
        files = self.files()
        logger = monitor.ready("rec_start") if monitor else False
        if not logger and self.rec in ("recording", "review"):
            self.rec, self.message = "idle", "The recording launch stopped; the unsaved demo is lost."
        policy = bool(monitor and monitor.policy_running)
        return {
            # a live policy also sends arm commands: that is not teleop
            "teleop_running": bool(monitor and monitor.teleop_running and not policy and not inference.running),
            "policy_running": policy,
            "hand_following": monitor.hand_following if monitor else False,
            "rec": self.rec,
            "rec_elapsed": (time.time() - self.rec_started) if self.rec == "recording" else self.rec_duration,
            "dataset": self.dataset,
            "demo_count": len(files),
            "last_files": files[-3:],
            "last_saved": self.last_saved,
            "message": self.message,
            "accurate": self.accurate,
            "coil_yaw_deg": self.coil_yaw(),
            "torque": self.torque,
            "after_stop": self.after_stop,
            "start_pose": self.start_pose(),
            "center": {"position": list(CENTER_POS), "distance": self._distance(CENTER_POS), "yaw_deg": self.center_yaw()},
            "max_move": MAX_MOVE_M,
            "near_limit": [{"joint": j, "margin": round(m, 1)} for j, m in self.joints_near_limit(12.0)],
            "moving": monitor.move_what if monitor and monitor.moving else "",
            "move_left": max(0.0, monitor.move_until - time.time()) if monitor else 0.0,
        }


operate = Operate()
nuc = NucSession()
teleop = PtyProcess("teleop")
recording = PtyProcess("recording")
job = PtyProcess("job")  # processing and checking recorded demos
job.dataset = None
inference = InferenceProc()
PROCS = {"nuc": nuc, "teleop": teleop, "recording": recording, "job": job, "inference": inference}
monitor = None


def cleanup():
    """Ctrl-C everything this page started and give it time to exit cleanly."""
    for p in (teleop, recording, inference, job):
        p.stop(grace=10)
    if nuc.state in ("launching", "running"):
        print("Stopping the NUC controller...", flush=True)
        nuc.stop_controller()
    end = time.time() + 15
    while time.time() < end and (teleop.running or recording.running or inference.running or nuc.state in ("launching", "running")):
        time.sleep(0.2)
    if nuc.running:
        nuc.stop(grace=3)
        end = time.time() + 5
        while time.time() < end and nuc.running:
            time.sleep(0.2)


@asynccontextmanager
async def lifespan(app):
    global monitor
    monitor = RosMonitor()
    threading.Thread(target=operate.supervise, daemon=True).start()
    threading.Thread(target=policy_watch, daemon=True).start()
    yield


app = FastAPI(lifespan=lifespan)
app.mount("/static", StaticFiles(directory=HERE / "static"), name="static")


@app.get("/")
def index():
    return FileResponse(HERE / "static" / "index.html", headers={"Cache-Control": "no-store"})



@app.get("/api/status")
def status():
    own = {p.pid for p in PROCS.values() if p.pid}
    usage = shutil.disk_usage(next(p for p in (RAW, DATA, WORKSPACE, Path.home()) if p.exists()))
    return {
        "procs": {k: p.info() for k, p in PROCS.items()},
        "external": external_launches(own),
        "ros": monitor.info() if monitor else {},
        "operate": operate.info(),
        "rates": monitor.rates() if monitor else {},
        "calibration": station.calibration(),
        "job_dataset": job.dataset,
        "disk_free_gb": round(usage.free / 1e9, 1),
        "datasets_dir": str(RAW),
        "hf_repo": station.hf_repo(),
        "features": ["lighting", "meta", "description", "orientation", "inference"],
        "time": time.time(),
    }


def hand_only_launch():
    return recording.running and getattr(recording, "mode", None) == "hand_only"


def _launch_guard():
    if teleop.running or recording.running or inference.running:
        raise HTTPException(409, "A launch is already running (station launch or policy). Stop it first: they start the same hand and camera nodes.")
    own = {p.pid for p in PROCS.values() if p.pid}
    ext = [e for e in external_launches(own) if e in ("teleop", "recording", "inference")]
    if ext:
        raise HTTPException(409, f"A {ext[0]} launch is already running in a terminal outside this page. Stop it there first.")


# -- running a trained policy --------------------------------------------------------
def _live_state():
    """What the checks compare the training data with: read from the robot right now, None where unknown."""
    now = time.time()
    fresh = bool(monitor and monitor.robot_stamp and now - monitor.robot_stamp < 2.0)
    return {
        "ee_params": getattr(monitor, "ee_params", None) if fresh else None,
        "arm": monitor.tele.get("arm") if monitor and monitor.arm_stamp and now - monitor.arm_stamp < 2.0 else None,
        "robot": monitor.info()["robot"] if monitor else None,
    }


def _failed(result):
    return [i["title"] for i in result["items"] if i["status"] == "bad"]


def _folder_gb(folder):
    total = 0
    try:
        for root, _, files in os.walk(folder):
            for f in files:
                try:
                    total += os.path.getsize(os.path.join(root, f))
                except OSError:
                    pass
    except OSError:
        pass
    return round(total / 1e9, 1)


def _relax_after_policy():
    """After a live policy stops: switch the hand motors off once its commands have ended, as after teleop."""
    end = time.time() + 3.0
    while time.time() < end and monitor.hand_following:
        time.sleep(0.1)
    if not inference.policy_on:
        operate._torque(False)


def policy_command(action, override=False):
    """Start, stop or reset the loaded policy, or move the arm to the center pose."""
    if action not in ("start", "stop", "reset", "init_arm"):
        raise ControlError("Unknown policy command.")
    if not inference.running:
        raise ControlError("Start the policy launch first.")
    if action == "stop":   # always allowed; it also ends a move to the center pose
        monitor.policy_command("stop")
        if not inference.preview:
            threading.Thread(target=_relax_after_policy, daemon=True).start()
        return ""
    if not inference.node_ready():
        raise ControlError("The policy is still loading. Wait until the card says it is ready.")
    if inference.init == "moving":
        raise ControlError("The arm is still moving to the center pose. Wait, or press Stop policy.")
    if inference.policy_on:
        raise ControlError("The policy is already running." if action == "start" else "Stop the policy first.")
    if action == "reset":
        monitor.policy_command("reset")
        return "Policy state reset."
    if action == "init_arm" and inference.preview:
        raise ControlError("In preview nothing is sent to the arm. Start the launch live to move it.")
    if not inference.preview:   # from here on the robot moves
        robot = monitor.info()["robot"]
        if not robot:
            raise ControlError("The Franka controller is not running. Start it first (section 1).")
        if robot["mode"] != "ready":
            raise ControlError(f"The robot is not ready ({robot['mode'].replace('_', ' ')}). Restart the controller first.")
        if not monitor.checks.get("controller_listening"):
            raise ControlError("The Franka controller is not listening for arm commands.")
    if action == "init_arm":   # the same pose as Move to center in the Operate card; here the inference node drives the arm
        pose = inference.init_pose
        if not inference.arm_init or not pose:
            raise ControlError("This launch cannot move the arm to the center pose. Stop it and start the policy launch live again.")
        if any(abs(a - b) > 1e-4 for a, b in zip(pose, operate.center_pose())):
            raise ControlError("The center pose rotation correction was changed after this policy launch started. Stop the launch and start it again to use the new value.")
        dist = operate._distance(pose[:3])
        if dist is None:
            raise ControlError("The arm position is unknown.")
        if dist > MAX_MOVE_M:
            raise ControlError(f"The center pose is {dist * 100:.0f} cm away, more than the {MAX_MOVE_M * 100:.0f} cm limit. Bring the arm closer first.")
        inference.ask_init()
        monitor.policy_command("init_arm")
        return f"The arm is moving to the center pose: {dist * 100:.0f} cm, about {max(1, round(dist / 0.02))} s."
    try:
        result = policies.check(inference.run, inference.ckpt, _live_state())
    except PolicyError as exc:
        raise ControlError(str(exc))
    note = ""
    if not inference.preview:
        failed = _failed(result)
        if failed and not override:
            raise ControlError("Not started, because a check failed: " + "; ".join(failed) + ". Fix it, or confirm in the page that you want to run anyway.")
        operate.relax_after_start = False
        note = operate._torque(True)   # the fingers need power to follow the policy
    monitor.policy_command("start")
    return note


def policy_watch():
    """Background loop: stops a live policy when the robot can no longer follow it, and notes a successful preview."""
    not_ready_since = None
    while True:
        time.sleep(0.25)
        try:
            if monitor is not None and inference.running:
                inference.node_ready()   # keeps its bookkeeping current while no page is open
            if monitor is None or not inference.running or not inference.policy_on:
                not_ready_since = None
                continue
            if inference.preview:
                if monitor.policy_running and not inference.marked:
                    inference.marked = True
                    policies.mark_previewed(inference.run, inference.ckpt)
                continue
            robot = monitor.info()["robot"]
            if robot and robot["mode"] == "ready":
                not_ready_since = None
                continue
            not_ready_since = not_ready_since or time.time()
            if time.time() - not_ready_since > 0.5:
                # Otherwise the policy keeps sending targets, and the arm would jump to the newest one when the controller restarts.
                monitor.policy_command("stop")
                inference.note = ("The policy was stopped by APE: " + (f"the robot went into {robot['mode'].replace('_', ' ')}." if robot else "no robot data arrives any more.")
                                  + " Recover the robot before starting the policy again.")
                threading.Thread(target=_relax_after_policy, daemon=True).start()
                not_ready_since = None
                time.sleep(1.0)
        except Exception:
            pass


@app.get("/api/policies")
def policy_list():
    return {"folder": str(POLICIES), "runs": policies.list_runs(), "logs_folder": str(INFERENCE_LOGS), "logs_gb": _folder_gb(INFERENCE_LOGS)}


@app.get("/api/policies/check")
def policy_check(run: str, ckpt: str):
    try:
        return policies.check(run, ckpt, _live_state())
    except PolicyError as exc:
        raise HTTPException(404, str(exc))


@app.post("/api/inference/start")
async def inference_start(request: Request):
    """Start the inference launch: cameras, ORCA hand and the policy. Nothing moves the arm until the policy is started."""
    body = await request.json()
    run, ckpt, preview = str(body.get("run", "")), str(body.get("ckpt", "")), bool(body.get("preview", True))
    try:
        _, path, config = policies.resolve(run, ckpt)
        result = await asyncio.to_thread(policies.check, run, ckpt, _live_state())
    except PolicyError as exc:
        raise HTTPException(400, str(exc))
    if config is None:
        raise HTTPException(400, result["items"][0]["text"])
    _launch_guard()
    failed = _failed(result)
    if failed and not preview and not body.get("override"):
        raise HTTPException(409, "Not started live, because a check failed: " + "; ".join(failed) + ". Fix it, or confirm in the page that you want to run anyway.")
    args = ["launch", "experiments", "run_policy_inference.launch.py", f"policy_ckpt_path:={path}", f"preview:={'true' if preview else 'false'}"]
    arm_init, pose = not preview, None
    if arm_init:   # lets the node's init_arm command bring the arm to the center pose, the one Move to center uses in the Operate card
        pose = operate.center_pose()
        args += ["allow_arm_init:=true", "arm_init_pose:=[" + ",".join(f"{v:.6f}" for v in pose) + "]"]
    test_cmd = os.environ.get("APE_INFERENCE_TEST_CMD")   # tests only: a stand-in for the launch
    inference.begin(json.loads(test_cmd) + args[3:] if test_cmd else ros2_cmd(*args), run, ckpt, preview, arm_init, pose)
    operate.reset()
    return inference.info()


@app.post("/api/inference/stop")
def inference_stop():
    if inference.running and monitor:
        try:
            monitor.policy_command("stop")   # end the action stream before the nodes go down
        except ControlError:
            pass
    inference.stop()
    return inference.info()


@app.post("/api/inference/cmd/{action}")
async def inference_cmd(action: str, request: Request):
    try:
        body = await request.json()
    except Exception:
        body = {}
    try:
        note = await asyncio.to_thread(policy_command, action, bool(body.get("override")))
    except ControlError as exc:
        raise HTTPException(409, str(exc))
    return {"note": note or "", "inference": inference.info()}


@app.post("/api/teleop/start")
async def teleop_start(request: Request):
    body = await request.json()
    _launch_guard()
    args = ["launch", "experiments", "teleop_rokoko.launch.py", "enable_keyboard_runner:=false"]
    if not body.get("hand", True):
        args.append("enable_hand_controller:=false")
    teleop.start(ros2_cmd(*args), cwd=WORKSPACE, env=ROS_ENV)
    teleop.hand = bool(body.get("hand", True))
    operate.reset()
    return teleop.info()


@app.post("/api/recording/start")
async def recording_start(request: Request):
    body = await request.json()
    name = str(body.get("dataset_name", "")).strip()
    if not DATASET_NAME.match(name):
        raise HTTPException(400, "Session name: letters, digits, _ and - only, starting with a letter or digit, e.g. s03_2026-11-24_opA.")
    _launch_guard()
    args = [
        "launch", "experiments", "record_demonstration.launch.py",
        f"storage_base_path:={RAW}", f"dataset_name:={name}", "enable_keyboard_runner:=false",
    ]
    if not body.get("hand", True):
        args.append("enable_hand_controller:=false")
    recording.hand, recording.mode = bool(body.get("hand", True)), "full"
    recording.start(ros2_cmd(*args), cwd=WORKSPACE, env=ROS_ENV)
    operate.reset(dataset=name)
    return recording.info()


@app.post("/api/hand_only/start")
def hand_only_start():
    """Gloves, retargeter and ORCA hand without the arm: no arm teleop, cameras or logger.
    It runs in the station launch's slot, so the same Stop button, Log and guards apply."""
    _launch_guard()
    recording.hand, recording.mode = True, "hand_only"
    recording.start(ros2_cmd("launch", "experiments", "teleop_rokoko.launch.py", "teleop_arm:=false", "enable_keyboard_runner:=false"),
                    cwd=WORKSPACE, env=ROS_ENV)
    operate.reset()
    return recording.info()


@app.post("/api/ctl/{action}")
async def control(action: str, request: Request):
    """Teleop and recording controls: the keyboard runner's steps as buttons."""
    try:
        body = await request.json()
    except Exception:
        body = {}
    on = bool(body.get("on", False))
    actions = {
        "teleop_start": operate.teleop_start,
        "teleop_start_rec": lambda: operate.teleop_start(record=True),
        "teleop_pause": operate.teleop_pause,
        "move2init": operate.move2init,
        "move2center": operate.move2center,
        "move_stop": operate.move_stop,
        "center_yaw": lambda: operate.set_center_yaw(body.get("value")),
        "after_stop": lambda: operate.set_after_stop(body.get("value")),
        "set_start": operate.set_start,
        "accurate": lambda: operate.set_accurate(on),
        "coil_reverse": lambda: operate.set_coil_reverse(on),
        "hand_follow": lambda: (monitor.call("hand_follow", on), "")[1],
        "hand_torque": lambda: operate.set_torque(on),
        "rec_start": operate.rec_start,
        "rec_stop": operate.rec_stop,
        "rec_save": lambda: operate.rec_save(body.get("labels") if isinstance(body.get("labels"), dict) else None),
        "rec_discard": operate.rec_discard,
    }
    if action not in actions:
        raise HTTPException(404)
    try:
        note = await asyncio.to_thread(actions[action])
    except ControlError as exc:
        raise HTTPException(409, str(exc))
    return {"note": note or "", "operate": operate.info()}


@app.get("/api/live")
async def live_stream(request: Request):
    """Live arm and hand values for the data panels, 10 times per second."""
    async def gen():
        while not await request.is_disconnected():
            yield "data: " + json.dumps(monitor.live() if monitor else {}) + "\n\n"
            await asyncio.sleep(0.1)

    return StreamingResponse(gen(), media_type="text/event-stream", headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})


@app.get("/api/cam/{cam}.jpg")
def camera_frame(cam: str):
    if cam not in CAMERAS or not monitor:
        raise HTTPException(404)
    data = monitor.jpeg(cam)
    if data is None:
        raise HTTPException(503, "No recent frame")
    return Response(data, media_type="image/jpeg", headers={"Cache-Control": "no-store"})


@app.get("/api/hand_config")
def hand_config():
    return station.hand_config()


@app.post("/api/hand/recalibrate")
def hand_recalibrate():
    own = {p.pid for p in PROCS.values() if p.pid}
    if teleop.running or recording.running or external_launches(own) or (monitor and monitor.ready("hand_torque")):
        raise HTTPException(409, "Stop the teleop or recording launch first: the hand controller is still running.")
    try:
        target = station.recalibrate_at_next_start()
    except DataError as exc:
        raise HTTPException(409, str(exc))
    return {"note": f"Old calibration moved to {target}. The hand will calibrate itself at the next launch start: keep clear of it."}


@app.get("/api/datasets")
def datasets():
    return {"datasets": station.list_datasets(), "job_running": job.running, "job_dataset": job.dataset}


@app.post("/api/datasets/{dataset}/process")
def dataset_process(dataset: str):
    if job.running:
        raise HTTPException(409, "Another processing job is still running.")
    if inference.policy_on:
        raise HTTPException(409, "Stop the policy first: processing is heavy and would make the policy stall.")
    if operate.rec in ("recording", "saving"):
        raise HTTPException(409, "Wait until the current demo is recorded and saved: processing is heavy and could disturb the recording.")
    try:
        script, prev = station.process_command(dataset)
    except DataError as exc:
        raise HTTPException(400, str(exc))
    prev.mkdir(parents=True, exist_ok=True)
    job.dataset = dataset
    job.start(["bash", "-c", script], cwd=WORKSPACE, env=ROS_ENV)
    return job.info()


@app.post("/api/upload")
def upload_start():
    """Upload raw and processed recordings, the manifest and the dataset version lists to Hugging Face."""
    if job.running:
        raise HTTPException(409, "Another processing or upload job is still running.")
    if inference.policy_on:
        raise HTTPException(409, "Stop the policy first: uploading could make the policy stall.")
    if operate.rec in ("recording", "saving"):
        raise HTTPException(409, "Wait until the current demo is recorded and saved: uploading could disturb the recording.")
    if not station.hf_repo():
        raise HTTPException(400, "No Hugging Face repo is set yet. Run once in a terminal: python " + str(station.tool) + " push --repo <user>/<name>")
    job.dataset = "upload"
    job.start(station.upload_command(), cwd=WORKSPACE)
    return job.info()


@app.post("/api/job/stop")
def job_stop():
    job.stop(grace=8)
    return job.info()


@app.get("/api/demo/{dataset}/{demo}/meta")
def demo_meta(dataset: str, demo: str):
    try:
        return station.demo_meta(dataset, demo)
    except DataError as exc:
        raise HTTPException(404, str(exc))


@app.get("/api/demo/{dataset}/{demo}/frame/{index}.jpg")
def demo_frame(dataset: str, demo: str, index: int):
    try:
        data = station.demo_frame(dataset, demo, index)
    except DataError as exc:
        raise HTTPException(404, str(exc))
    return Response(data, media_type="image/jpeg", headers={"Cache-Control": "max-age=3600"})


@app.get("/api/demo/{dataset}/{demo}/trajectory.png")
def demo_plot(dataset: str, demo: str):
    try:
        f = station.previews / safe(dataset) / safe(demo) / "trajectory.png"
    except DataError as exc:
        raise HTTPException(404, str(exc))
    if not f.is_file():
        raise HTTPException(404)
    return FileResponse(f)


@app.get("/api/terms")
async def terms_stream(request: Request):
    """All terminal outputs in one stream (browsers allow only a few open connections per site)."""
    async def gen():
        seqs = {name: 0 for name in PROCS}
        idle = 0
        while not await request.is_disconnected():
            sent = False
            for name, proc in PROCS.items():
                chunks = proc.read_since(seqs[name])
                if chunks:
                    seqs[name] = chunks[-1][0]
                    payload = base64.b64encode(b"".join(d for _, d in chunks)).decode()
                    yield f"event: {name}\ndata: {payload}\n\n"
                    sent = True
            idle = 0 if sent else idle + 1
            if idle and idle % 200 == 0:
                yield ": keepalive\n\n"
            await asyncio.sleep(0.05)

    return StreamingResponse(gen(), media_type="text/event-stream", headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})


@app.post("/api/{name}/stop")
def proc_stop(name: str):
    if name == "nuc":
        nuc.stop_controller()
        return nuc.info()
    if name not in ("teleop", "recording"):
        raise HTTPException(404)
    PROCS[name].stop()
    return PROCS[name].info()


@app.post("/api/nuc/password")
async def nuc_password(request: Request):
    """The password box: passed on to ssh's prompt, never stored or logged."""
    body = await request.json()
    password = str(body.get("password", ""))
    if not password or "\n" in password or "\r" in password:
        raise HTTPException(400, "Type the NUC password first.")
    try:
        nuc.submit_password(password)
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))
    return nuc.info()


@app.post("/api/nuc/{action}")
def nuc_action(action: str):
    try:
        if action == "start":
            if monitor and monitor.commands_flowing:
                raise RuntimeError(("The policy" if inference.running else "Teleop") + " is still sending arm commands. "
                                   + ("Stop the policy" if inference.running else "Pause teleop") + " first, so the arm does not jump when the controller starts.")
            nuc.start_controller()
        elif action == "connect":
            if nuc.running:
                raise RuntimeError("Already connected")
            nuc.connect(auto=False)
        elif action == "stop":
            nuc.stop_controller()
        elif action == "restart":
            robot = monitor.info()["robot"] if monitor else None
            if robot and robot["mode"] == "user_stopped":
                raise RuntimeError("The black stop button is still pressed. Release it fully first, then restart.")
            note = ""
            if inference.running and monitor and (inference.policy_on or inference.init == "moving" or monitor.commands_flowing):
                monitor.policy_command("stop")
                end = time.time() + 3.0
                while time.time() < end and monitor.commands_flowing:
                    time.sleep(0.1)
                if monitor.commands_flowing:
                    raise RuntimeError("The policy is still sending arm commands. Stop the policy first.")
                threading.Thread(target=_relax_after_policy, daemon=True).start()
            elif monitor and (monitor.commands_flowing or monitor.hand_following):
                # the restart must never happen while teleop still sends arm targets: pause it first
                try:
                    if operate.rec == "recording":
                        note = operate.rec_stop_no_return()
                    else:
                        operate.teleop_pause()
                except ControlError as exc:
                    raise RuntimeError(f"Teleop could not be paused, so the controller was not restarted: {exc}")
                end = time.time() + 3.0
                while time.time() < end and monitor.commands_flowing:
                    time.sleep(0.1)
                if monitor.commands_flowing:
                    raise RuntimeError("Teleop is still sending arm commands. Pause teleop first.")
            nuc.restart_controller()
        elif action == "disconnect":
            if nuc.state in ("launching", "running"):
                raise RuntimeError("Stop the controller first, then disconnect.")
            nuc.stop(grace=3)
        else:
            raise HTTPException(404)
    except RuntimeError as exc:
        raise HTTPException(409, str(exc))
    return nuc.info()


@app.post("/api/term/{name}/input")
async def term_input(name: str, request: Request):
    if name != "nuc":
        raise HTTPException(403, "Only the NUC terminal accepts typing.")
    body = await request.json()
    nuc.write(str(body.get("data", "")).encode())
    return {}


@app.post("/api/term/{name}/resize")
async def term_resize(name: str, request: Request):
    body = await request.json()
    rows, cols = int(body.get("rows", 40)), int(body.get("cols", 160))
    if name in PROCS and 5 <= rows <= 300 and 20 <= cols <= 500:
        PROCS[name].resize(rows, cols)
    return {}


@app.get("/api/term/{name}/stream")
async def term_stream(name: str, request: Request):
    if name not in PROCS:
        raise HTTPException(404)
    proc = PROCS[name]

    async def gen():
        seq = 0
        idle = 0
        while not await request.is_disconnected():
            chunks = proc.read_since(seq)
            if chunks:
                seq = chunks[-1][0]
                payload = base64.b64encode(b"".join(d for _, d in chunks)).decode()
                yield f"data: {payload}\n\n"
                idle = 0
            else:
                idle += 1
                if idle % 100 == 0:
                    yield ": keepalive\n\n"
            await asyncio.sleep(0.05 if name == "nuc" else 0.15)

    return StreamingResponse(gen(), media_type="text/event-stream", headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1", help="0.0.0.0 lets other devices on the network open (and control!) the page")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    print(f"\n  APE UI: http://{'localhost' if args.host == '127.0.0.1' else args.host}:{args.port}\n", flush=True)
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="critical", timeout_graceful_shutdown=3)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # a second Ctrl-C must not cut the cleanup short
        cleanup()
        print("Dashboard stopped.", flush=True)
        os._exit(0)  # skip interpreter teardown: the ROS reader thread would abort there


if __name__ == "__main__":
    main()
