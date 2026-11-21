#!/usr/bin/env python3

"""
Measure how well the glove teleop reproduces a fixed list of hand poses.
This script only measures: it changes no config and sends no commands to the hand.

Step 1, record (teleop running: ingress, retargeter enabled, hand controller):

    python retargeter_teleop_test.py record --out teleop_test.h5

    The script walks through the poses below. For each one, make the pose with the
    gloved hand, press Enter with the other hand, and hold still until the second beep.
    Per pose it stores the glove keypoints, the commanded joint angles and the joint
    angles read back from the motors.

Step 2, report (offline, no ROS needed):

    python retargeter_teleop_test.py report teleop_test.h5

    Prints, per pose, fingertip distances and joint angles at four stages:
        glove     distances between the raw glove keypoints
        target    the same after the MANO adjustments (what the retargeter aims for)
        commanded robot fingertips (forward kinematics) at the commanded joint angles
        measured  robot fingertips (forward kinematics) at the joint angles from the motors
    and writes the full tables next to the recording as CSV files.
    glove -> target shows the effect of the MANO adjustments, target -> commanded the
    effect of the loss weights and joint limits, commanded -> measured the hardware.
"""

import argparse
import csv
import json
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HAND_NAME = "orcahand_v2"
CFG_DIR = ROOT / "experiments" / "cfgs" / "retargeter"
MODEL_DIR = ROOT / "src" / "viz" / "models" / HAND_NAME

FINGERS = ["thumb", "index", "middle", "ring", "pinky"]
# row of each fingertip / finger base in the (22, 3) glove keypoint array
TIP_IDX = {"thumb": 5, "index": 9, "middle": 13, "ring": 17, "pinky": 21}
BASE_IDX = {"thumb": 2, "index": 6, "middle": 10, "ring": 14, "pinky": 18}

THUMB_TO = [f"thumb-{f}" for f in FINGERS[1:]]
GAPS = ["index-middle", "middle-ring", "ring-pinky"]
TO_PALM = [f"{f}-palm" for f in FINGERS]
DISTANCES = THUMB_TO + GAPS + TO_PALM

FLEX = [f"{f}_{j}" for f in FINGERS[1:] for j in ("mcp", "pip")]
ABD = [f"{f}_abd" for f in FINGERS[1:]]
THUMB = ["thumb_cmc", "thumb_abd", "thumb_mcp", "thumb_dip"]

LIMIT_MARGIN_DEG = 1.0


def pose(name, instruction, distances=(), joints=(), kind="static"):
    return dict(name=name, instruction=instruction, distances=list(distances),
                joints=list(joints), kind=kind)


def build_protocol(spacer_mm):
    spacer = f"hold a {spacer_mm:g} mm spacer between the two fingertips"
    protocol = [
        pose("flat", "Hand flat on the table, fingers together, thumb alongside the index.",
             joints=FLEX + ABD + THUMB + ["wrist"]),
        pose("spread", "Fingers straight and spread as wide as possible.",
             distances=GAPS, joints=ABD),
        pose("fist", "Full fist, thumb resting outside the fingers.",
             distances=TO_PALM[1:], joints=FLEX),
    ]
    for f in FINGERS[1:]:
        protocol.append(pose(f"flex_{f}", f"Only the {f} finger curled into the palm, the others straight.",
                             distances=[f"{f}-palm"], joints=[f"{f}_mcp", f"{f}_pip"]))
    protocol += [
        pose("hook", "Hook: knuckles straight, middle and end joints fully curled.", joints=FLEX),
        pose("tabletop", "Tabletop: knuckles bent 90 degrees, fingers themselves straight.", joints=FLEX),
    ]
    for a, b in (("index", "middle"), ("middle", "ring"), ("ring", "pinky")):
        protocol.append(pose(f"spread_{a}_{b}",
                             f"Fingers straight; open only the gap between {a} and {b}, keep the others together.",
                             distances=[f"{a}-{b}"], joints=[f"{a}_abd", f"{b}_abd"]))
    for f in FINGERS[1:]:
        protocol.append(pose(f"pinch_{f}", f"Thumb tip touching the {f} fingertip, light contact.",
                             distances=[f"thumb-{f}"], joints=THUMB + [f"{f}_mcp", f"{f}_pip"]))
    for f in FINGERS[1:]:
        protocol.append(pose(f"gap_{f}", f"Thumb and {f}: {spacer}.",
                             distances=[f"thumb-{f}"], joints=THUMB + [f"{f}_mcp", f"{f}_pip"]))
    protocol += [
        pose("key_pinch", "Key pinch: thumb pad pressed on the side of the index finger.",
             distances=["thumb-index"], joints=THUMB),
        pose("thumb_pinky_base", "Thumb tip touching the base of the pinky, fingers straight.",
             distances=["thumb-palm", "thumb-pinky"], joints=THUMB),
        pose("thumb_out", "L shape: fingers together, thumb stretched sideways in the plane of the palm.",
             distances=["thumb-index"], joints=THUMB),
        pose("thumb_forward", "Fingers together, thumb pointing straight out of the palm (as if holding a can).",
             distances=["thumb-index", "thumb-palm"], joints=THUMB),
        pose("wrist_flexed", "Open hand, wrist bent fully towards the palm side.", joints=["wrist"]),
        pose("wrist_extended", "Open hand, wrist bent fully towards the back of the hand.", joints=["wrist"]),
        pose("sweep_open_close", "Moving test: open and close the whole hand slowly, about one cycle every 4 s.",
             joints=FLEX + THUMB, kind="sweep"),
    ]
    return protocol


# --------------------------------------------------------------------------------------
# record
# --------------------------------------------------------------------------------------

def record(args):
    # must match the environment the teleop was started in, and be set before ROS is loaded
    if args.domain is not None:
        os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    if args.rmw:
        os.environ["RMW_IMPLEMENTATION"] = args.rmw

    import h5py
    import rclpy
    import yaml
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray

    side = args.chirality
    topics = {
        "mano": f"/ingress/{side}/mano",
        "commanded": f"/{side}/hand/policy_output",
        "measured": f"/{side}/hand/joint_position_from_motor",
        "current": f"/{side}/hand/motor_current",
    }

    class Recorder(Node):
        def __init__(self):
            super().__init__("retargeter_teleop_test")
            self.lock = threading.Lock()
            self.active = False
            self.buffers = {k: [] for k in topics}
            self.last_seen = {k: None for k in topics}
            for key, topic in topics.items():
                self.create_subscription(Float32MultiArray, topic, lambda msg, key=key: self.cb(key, msg), 50)

        def cb(self, key, msg):
            now = time.monotonic()
            self.last_seen[key] = now
            if self.active:
                with self.lock:
                    self.buffers[key].append((now, np.asarray(msg.data, dtype=np.float32)))

        def capture(self, duration):
            with self.lock:
                self.buffers = {k: [] for k in topics}
            self.active = True
            time.sleep(duration)
            self.active = False
            with self.lock:
                return {k: list(v) for k, v in self.buffers.items()}

    protocol = build_protocol(args.spacer_mm)
    if args.poses:
        unknown = set(args.poses) - {p["name"] for p in protocol}
        if unknown:
            sys.exit(f"Unknown pose names: {sorted(unknown)}")
        protocol = [p for p in protocol if p["name"] in args.poses]
    if args.list:
        for p in protocol:
            print(f"{p['name']:18s} {p['instruction']}")
        return

    out = Path(args.out or f"teleop_test_{datetime.now():%Y%m%d_%H%M%S}.h5").expanduser()
    if out.exists():
        sys.exit(f"{out} already exists; choose another --out.")

    scheme = yaml.safe_load((MODEL_DIR / f"scheme_{HAND_NAME}.yaml").read_text())

    rclpy.init()
    node = Recorder()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # refuse to start while a needed topic is silent, instead of recording empty poses
    required = {
        "mano": "the glove stream (Rokoko Studio, rokoko_node)",
        "commanded": "the retargeter output (is the retargeter enabled?)",
        "measured": "the hand controller",
    }
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline and any(node.last_seen[k] is None for k in required):
        time.sleep(0.1)
    dead = [k for k in required if node.last_seen[k] is None]
    if dead:
        for k in dead:
            print(f"No messages on {topics[k]}: {required[k]}")
        if len(dead) == len(required):
            print("Nothing at all is arriving. Either the teleop is not running, or it runs with other ROS settings.\n"
                  f"This session uses ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0')}, "
                  f"RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION', '(default)')}. "
                  "Use -d and -m to match the teleop.")
        if not args.force:
            rclpy.shutdown()
            spin_thread.join(timeout=1.0)
            sys.exit("Not recording. Fix the above, or pass --force to record anyway.")

    steps = [(rep, p) for rep in range(args.reps) for p in protocol]
    print(f"\n{len(protocol)} poses x {args.reps} repetitions, {args.duration:g} s each. Writing {out}\n"
          "  Enter : record this pose (starts after a short delay, hold until the second beep)\n"
          "  r     : redo the previous pose\n"
          "  s     : skip this pose\n"
          "  q     : quit (everything recorded so far is already saved)\n")

    with h5py.File(out, "w") as h5:
        h5.attrs["created"] = datetime.now().isoformat(timespec="seconds")
        h5.attrs["chirality"] = side
        h5.attrs["spacer_mm"] = args.spacer_mm
        h5.attrs["joint_names"] = json.dumps(list(scheme["gc_tendons"]))
        h5.attrs["topics"] = json.dumps(topics)
        h5.attrs["protocol"] = json.dumps(build_protocol(args.spacer_mm))
        # configs as they were during this run, so the report can reproduce the targets
        h5.attrs["mano_adjustments_yaml"] = (CFG_DIR / f"mano_adjustment_{HAND_NAME}.yaml").read_text()
        h5.attrs["retargeter_cfg_yaml"] = (CFG_DIR / f"retargeter_cfgs_{HAND_NAME}.yaml").read_text()
        h5.attrs["scheme_yaml"] = (MODEL_DIR / f"scheme_{HAND_NAME}.yaml").read_text()

        n_groups = 0
        i = 0
        last_group = None  # (step index, group name) of the previous recording
        while i < len(steps):
            rep, p = steps[i]
            duration = args.sweep_duration if p["kind"] == "sweep" else args.duration
            print(f"[{i + 1}/{len(steps)}] rep {rep + 1}  {p['name']}\n    {p['instruction']}")
            try:
                cmd = input("    press Enter to record > ").strip().lower()
            except EOFError:
                cmd = "q"
            if cmd == "q":
                break
            if cmd == "s":
                i += 1
                continue
            if cmd == "r":
                if last_group is None:
                    print("    Nothing to redo.")
                    continue
                i, group_name = last_group
                h5[group_name].attrs["discarded"] = True
                last_group = None
                continue
            if cmd != "":
                print("    Unknown command.")
                continue

            time.sleep(args.delay)
            print("\a    recording...", end="", flush=True)
            data = node.capture(duration)
            print("\a done.")

            group_name = f"rec_{n_groups:04d}"
            n_groups += 1
            g = h5.create_group(group_name)
            g.attrs["pose"] = p["name"]
            g.attrs["rep"] = rep
            g.attrs["kind"] = p["kind"]
            g.attrs["discarded"] = False
            counts = []
            for key, samples in data.items():
                counts.append(f"{key} {len(samples)}")
                sizes = {len(v) for _, v in samples}
                if not samples or len(sizes) != 1:
                    continue
                g.create_dataset(f"{key}_t", data=np.array([t for t, _ in samples]))
                g.create_dataset(key, data=np.stack([v for _, v in samples]))
            h5.flush()
            print(f"    samples: {', '.join(counts)}")
            missing = [k for k in ("mano", "commanded", "measured") if k not in g]
            if missing:
                print(f"    WARNING: nothing recorded for {missing}; press r to redo once the topic is alive.")
            last_group = (i, group_name)
            i += 1

    if n_groups == 0:
        out.unlink()
        print("\nNothing was recorded (no pose was started with Enter), so no file was kept.")
    else:
        print(f"\nSaved {n_groups} recordings to {out}\nNext: python {Path(__file__).name} report {out}")
    rclpy.shutdown()
    spin_thread.join(timeout=1.0)


# --------------------------------------------------------------------------------------
# report
# --------------------------------------------------------------------------------------

def glove_distances(points):
    """points: (N, 22, 3) keypoints. Returns {distance name: (N,) in metres}."""
    tips = {f: points[:, TIP_IDX[f]] for f in FINGERS}
    palm = (points[:, BASE_IDX["thumb"]] + points[:, BASE_IDX["pinky"]]) / 2
    return named_distances(tips, palm)


def named_distances(tips, palm):
    out = {}
    for name in THUMB_TO + GAPS:
        a, b = name.split("-")
        out[name] = np.linalg.norm(tips[a] - tips[b], axis=-1)
    for f in FINGERS:
        out[f"{f}-palm"] = np.linalg.norm(tips[f] - palm, axis=-1)
    return out


class RobotModel:
    """Forward kinematics and MANO adjustments, taken from the Retargeter itself."""

    def __init__(self, scheme, mano_adjustments, retargeter_cfg, chirality):
        import torch
        import xacro
        from faive_system.src.retargeter.retargeter import Retargeter, transform_from_anchor
        from faive_system.src.retargeter.utils import retarget_utils

        self.torch = torch
        self.transform_from_anchor = transform_from_anchor
        self.retarget_utils = retarget_utils
        urdf = xacro.process_file(str(MODEL_DIR / f"{HAND_NAME}.urdf.xacro"),
                                  mappings={"hand_chirality": chirality}).toxml()
        self.r = Retargeter(urdf_string=urdf, hand_scheme=scheme, mano_adjustments=mano_adjustments,
                            retargeter_cfg=retargeter_cfg, device="cpu")

    def robot_distances(self, joints_deg):
        """joints_deg: (N, n_joints) in the scheme order. Returns {distance name: (N,) in metres}."""
        torch, r = self.torch, self.r
        q = torch.as_tensor(np.deg2rad(joints_deg), dtype=torch.float32)
        transforms = r.chain.forward_kinematics(q @ r.joint_map.T)

        def position(frame):
            t = self.transform_from_anchor(transforms, frame, r.anchor_name)
            return t.transform_points(torch.zeros(1, 3)).reshape(-1, 3).numpy()

        tips = {f: position(r.finger_to_tip[f]) for f in FINGERS}
        palm = (position(r.finger_to_base["thumb"]) + position(r.finger_to_base["pinky"])) / 2
        return named_distances(tips, palm)

    def target_distances(self, mano):
        """mano: (N, 22, 3) raw glove keypoints. Distances after the MANO adjustments."""
        r = self.r
        adjusted = []
        for points in mano.astype(np.float64):
            n, _ = self.retarget_utils.normalize_points_to_hands_local(points)
            n = r.adjust_mano_fingers(n)
            adjusted.append(n @ r.model_rotation.T + r.model_center)
        return glove_distances(np.stack(adjusted))


def report(args):
    import h5py
    import yaml

    path = Path(args.file).expanduser()
    rows_d, rows_j = [], []
    with h5py.File(path, "r") as h5:
        joint_names = json.loads(h5.attrs["joint_names"])
        protocol = json.loads(h5.attrs["protocol"])
        scheme = yaml.safe_load(h5.attrs["scheme_yaml"])
        model = RobotModel(scheme, yaml.safe_load(h5.attrs["mano_adjustments_yaml"]),
                           yaml.safe_load(h5.attrs["retargeter_cfg_yaml"]), h5.attrs["chirality"])
        lower = np.array(scheme["gc_limits_lower"], dtype=float)
        upper = np.array(scheme["gc_limits_upper"], dtype=float)

        for name in sorted(h5):
            g = h5[name]
            if g.attrs["discarded"]:
                continue
            key = dict(pose=g.attrs["pose"], rep=int(g.attrs["rep"]), kind=g.attrs["kind"])

            stages = {}
            if "mano" in g:
                mano = g["mano"][:].reshape(len(g["mano"]), -1, 3)
                stages["glove"] = glove_distances(mano)
                stages["target"] = model.target_distances(mano)
            joints = {}
            for stage in ("commanded", "measured"):
                if stage in g:
                    joints[stage] = np.rad2deg(g[stage][:].astype(np.float64))
                    stages[stage] = model.robot_distances(joints[stage])

            for d in DISTANCES:
                row = dict(key, distance=d)
                for stage in ("glove", "target", "commanded", "measured"):
                    v = stages[stage][d] * 1000 if stage in stages else np.array([np.nan])
                    row[f"{stage}_mm"] = v.mean()
                    row[f"{stage}_min_mm"] = v.min()
                    row[f"{stage}_max_mm"] = v.max()
                rows_d.append(row)

            for j, joint in enumerate(joint_names):
                row = dict(key, joint=joint, lower_deg=lower[j], upper_deg=upper[j])
                for stage in ("commanded", "measured"):
                    v = joints[stage][:, j] if stage in joints else np.array([np.nan])
                    row[f"{stage}_deg"] = v.mean()
                    row[f"{stage}_std_deg"] = v.std()
                    row[f"{stage}_min_deg"] = v.min()
                    row[f"{stage}_max_deg"] = v.max()
                    at_limit = (v < lower[j] + LIMIT_MARGIN_DEG) | (v > upper[j] - LIMIT_MARGIN_DEG)
                    row[f"{stage}_at_limit_frac"] = at_limit.mean() if stage in joints else np.nan
                rows_j.append(row)

    if not rows_d:
        sys.exit("No recordings in this file.")
    for rows, suffix in ((rows_d, "distances"), (rows_j, "joints")):
        csv_path = path.with_name(f"{path.stem}_{suffix}.csv")
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f"Wrote {csv_path}")

    print_summary(protocol, rows_d, rows_j)


def mean_spread(values):
    values = np.asarray(values, dtype=float)
    if np.all(np.isnan(values)):
        return "      -     "
    spread = f"±{np.nanstd(values):4.1f}" if len(values) > 1 else "     "
    return f"{np.nanmean(values):6.1f}{spread} "


def print_summary(protocol, rows_d, rows_j):
    print("\nMean over repetitions ± spread between repetitions. Distances in mm, angles in degrees.")
    for p in protocol:
        d_rows = [r for r in rows_d if r["pose"] == p["name"]]
        j_rows = [r for r in rows_j if r["pose"] == p["name"]]
        if not d_rows:
            continue
        reps = len({r["rep"] for r in d_rows})
        sweep = p["kind"] == "sweep"
        print(f"\n== {p['name']} ({reps} rep{'s' if reps > 1 else ''}) ==")

        if p["distances"] and not sweep:
            print(f"  {'distance':14s}{'glove':>12s}{'target':>12s}{'commanded':>12s}{'measured':>12s}")
            for d in p["distances"]:
                sel = [r for r in d_rows if r["distance"] == d]
                cells = "".join(mean_spread([r[f"{s}_mm"] for r in sel])
                                for s in ("glove", "target", "commanded", "measured"))
                print(f"  {d:14s}{cells}")

        if p["joints"]:
            if sweep:
                print(f"  {'joint':14s}{'commanded range':>20s}{'measured range':>20s}{'limits':>16s}")
            else:
                print(f"  {'joint':14s}{'commanded':>12s}{'measured':>12s}{'jitter':>8s}")
            for joint in p["joints"]:
                sel = [r for r in j_rows if r["joint"] == joint]
                if sweep:
                    def span(stage):
                        lo = np.nanmin([r[f"{stage}_min_deg"] for r in sel])
                        hi = np.nanmax([r[f"{stage}_max_deg"] for r in sel])
                        return f"{lo:6.1f} to {hi:6.1f}"
                    limits = f"{sel[0]['lower_deg']:.0f} to {sel[0]['upper_deg']:.0f}"
                    print(f"  {joint:14s}{span('commanded'):>20s}{span('measured'):>20s}{limits:>16s}")
                else:
                    jitter = np.nanmean([r["measured_std_deg"] for r in sel])
                    print(f"  {joint:14s}{mean_spread([r['commanded_deg'] for r in sel])}"
                          f"{mean_spread([r['measured_deg'] for r in sel])}{jitter:8.2f}")

        # any joint (not only the listed ones) that sat on a limit for most of the recording
        stuck = sorted({r["joint"] for r in j_rows
                        if not sweep and r["commanded_at_limit_frac"] > 0.5})
        if stuck:
            print(f"  commanded within {LIMIT_MARGIN_DEG:g} deg of a joint limit: {', '.join(stuck)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    rec = sub.add_parser("record", help="guided recording of the test poses (needs the teleop running)")
    rec.add_argument("-o", "--out", help="output .h5 file (default: teleop_test_<time>.h5 in the current directory)")
    rec.add_argument("--chirality", default="right")
    rec.add_argument("-r", "--reps", type=int, default=3, help="how many times to go through the pose list")
    rec.add_argument("--duration", type=float, default=2.0, help="seconds recorded per static pose")
    rec.add_argument("--sweep-duration", type=float, default=10.0, help="seconds recorded for a moving test")
    rec.add_argument("--delay", type=float, default=1.0, help="seconds between pressing Enter and recording")
    rec.add_argument("--spacer-mm", type=float, default=20.0, help="width of the spacer used in the gap_* poses")
    rec.add_argument("-p", "--poses", nargs="+", help="record only these poses")
    rec.add_argument("-d", "--domain", type=int, help="ROS_DOMAIN_ID the teleop runs in (default: from the environment)")
    rec.add_argument("-m", "--rmw", help="RMW_IMPLEMENTATION the teleop runs with, e.g. rmw_cyclonedds_cpp")
    rec.add_argument("--force", action="store_true", help="record even if a needed topic has no messages")
    rec.add_argument("-l", "--list", action="store_true", help="print the pose list and exit")
    rec.set_defaults(func=record)

    rep = sub.add_parser("report", help="compute the measures from a recording")
    rep.add_argument("file")
    rep.set_defaults(func=report)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
