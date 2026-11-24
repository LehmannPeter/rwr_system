"""Shared helpers for encode_archive.py and reconstruct_raw.py (no ROS needed).

Archive layout for one raw recording <stem>.h5 (the logger's raw format: one HDF5 group per topic,
one dataset per message, named by its receive time in ns):

    <stem>.h5                 every non-image topic copied unchanged, plus the group ARCHIVE_GROUP
                              with, per camera topic, the message names (timestamps) in recording
                              order and the HDF5 storage settings needed to write the raw file again
    <stem>.<camera>.mp4       one video per camera topic, frames in timestamp order
    <stem>.meta.json          copied from the raw folder when it exists (written by APE at save time)
    <stem>.archive.json       written last: settings, frame counts, PSNR of the check, sha256 of each file

An archive is complete only when <stem>.archive.json exists and its "ok" is true.
"""
import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path

import numpy as np

ARCHIVE_GROUP = "_video_archive"
FORMAT_VERSION = 1

# libx264 4:4:4 at CRF 12 with a keyframe every 30 frames, chosen on 2026-11-24 from a validation on the
# 110 Nov 19 recordings: archive about 5.7 % of the raw file (about 2.4 MB per second of recording, both
# cameras), PSNR about 45 dB after the 224x224 resize used in training. CRF 18 is 2.4 % / 42 dB, but its
# effect on the DINOv2 patch features was clearly larger (wrist camera cosine 0.94 vs 0.97 at CRF 12),
# and the raw recordings are deleted after the transfer, so the safer setting is the default.
DEFAULT_VIDEO_ARGS = ["-c:v", "libx264", "-preset", "medium", "-profile:v", "high444", "-pix_fmt", "yuv444p",
                      "-crf", "12", "-g", "30"]


def find_ffmpeg(explicit=None):
    """--ffmpeg, then $FFMPEG, then ffmpeg on PATH, then the binary shipped with imageio-ffmpeg."""
    for candidate in (explicit, os.environ.get("FFMPEG"), shutil.which("ffmpeg")):
        if candidate:
            return candidate
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        raise SystemExit("No ffmpeg found: install it, set $FFMPEG, pass --ffmpeg, or pip install imageio-ffmpeg.")


def camera_slug(topic):
    """'/oakd_front_view/color' -> 'oakd_front_view'; other topics -> path with '/' replaced by '__'."""
    t = topic.strip("/")
    if t.endswith("/color"):
        t = t[: -len("/color")]
    return t.replace("/", "__")


def message_groups(h5):
    """Paths of all groups that directly hold datasets (one per topic in the raw format)."""
    import h5py

    found = []

    def visit(name, obj):
        if isinstance(obj, h5py.Group) and any(isinstance(v, h5py.Dataset) for v in obj.values()):
            found.append("/" + name)

    h5.visititems(visit)
    return found


def is_image_topic(group):
    """A colour camera topic: every message is an H x W x 3 uint8 image of the same size."""
    import h5py

    first = None
    for v in group.values():
        if not isinstance(v, h5py.Dataset):
            return False
        if first is None:
            first = v
            if not (v.dtype == np.uint8 and v.ndim == 3 and v.shape[2] == 3):
                return False
        elif v.shape != first.shape or v.dtype != first.dtype:
            raise ValueError(f"{group.name}: images of different sizes; this archive format needs one size per camera")
    return first is not None


def sorted_names(group):
    """Message names sorted by their integer timestamp (the order of the frames in the video)."""
    return sorted(group.keys(), key=int)


def sha256(path, block=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(block):
            h.update(chunk)
    return h.hexdigest()


def psnr(a, b):
    mse = np.mean((a.astype(np.float32) - b.astype(np.float32)) ** 2)
    return float("inf") if mse == 0 else float(10 * np.log10(255.0 ** 2 / mse))


class FrameReader:
    """Decode a video to RGB frames one at a time through an ffmpeg pipe."""

    def __init__(self, ffmpeg, path, width, height, threads=4):
        self.size = width * height * 3
        self.shape = (height, width, 3)
        self.proc = subprocess.Popen(
            [ffmpeg, "-nostdin", "-loglevel", "error", "-threads", str(threads), "-i", str(path),
             "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
            stdout=subprocess.PIPE)

    def read(self):
        buf = self.proc.stdout.read(self.size)
        if len(buf) < self.size:
            return None
        return np.frombuffer(buf, np.uint8).reshape(self.shape)

    def close(self):
        rest = self.proc.stdout.read()
        self.proc.stdout.close()
        code = self.proc.wait()
        return code, len(rest)


def write_json_atomic(path, data):
    tmp = Path(str(path) + ".tmp")
    tmp.write_text(json.dumps(data, indent=1))
    tmp.replace(path)
