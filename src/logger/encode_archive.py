#!/usr/bin/env python3
"""Turn raw recordings (logger HDF5) into a compact archive: one MP4 per camera plus an HDF5 with
everything else. reconstruct_raw.py turns an archive back into a raw-format HDF5 that
syncronize_data.py processes like the original.

What is exact and what is not:
- every non-image topic, every attribute, the camera intrinsics and the message names (receive
  timestamps) of the images are copied bit for bit;
- the camera images are compressed with libx264 4:4:4 CRF 12 (lossy, about 45 dB PSNR after the
  224x224 resize used in training; --crf to change). Each video is decoded again after encoding and compared with
  the raw frames; the result is in <stem>.archive.json.

Plain Python (h5py, numpy, ffmpeg); no ROS. Layout: see video_archive_common.py.

    python encode_archive.py raw/<session>            -o archive/<session>
    python encode_archive.py raw/<session>/<stem>.h5  -o archive/<session>
"""
import argparse
import concurrent.futures
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import h5py
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from video_archive_common import (ARCHIVE_GROUP, DEFAULT_VIDEO_ARGS, FORMAT_VERSION, FrameReader, camera_slug,  # noqa: E402
                                  find_ffmpeg, is_image_topic, message_groups, psnr, sha256, sorted_names,
                                  write_json_atomic)


def _copy_attrs(src, dst):
    for k, v in src.attrs.items():
        dst.attrs[k] = v


class Progress:
    """<out>/.<stem>.progress.json while a recording is encoded: {"done": frames, "total": frames, "stage": ...}.
    APE shows it in the demo list. A dot file, so rsync and the archive tools ignore it; removed at the end."""

    def __init__(self, path, total):
        self.path, self.total, self.done, self.last = Path(path), max(1, int(total)), 0, 0.0
        self.write("starting", force=True)

    def step(self, stage, n=1):
        self.done += n
        self.write(stage)

    def write(self, stage, force=False):
        if not force and time.time() - self.last < 0.5:
            return
        self.last = time.time()
        try:
            write_json_atomic(self.path, {"stage": stage, "done": self.done, "total": self.total, "pid": os.getpid(),
                                          "updated": round(self.last, 1)})
        except OSError:
            pass

    def remove(self):
        for f in (self.path, Path(str(self.path) + ".tmp")):
            try:
                f.unlink()
            except OSError:
                pass


def _encode_topic(ffmpeg, group, names, video_path, fps, threads, video_args, progress=None):
    """Pipe the frames of one camera topic into ffmpeg, one frame at a time (low memory)."""
    h, w, _ = group[names[0]].shape
    cmd = [ffmpeg, "-nostdin", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
           "-r", str(fps), "-i", "-", *video_args, "-threads", str(threads), "-an", "-movflags", "+faststart",
           str(video_path)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    try:
        for name in names:
            proc.stdin.write(np.ascontiguousarray(group[name][()]).tobytes())
            if progress:
                progress.step("encoding")
    except BrokenPipeError:
        pass
    proc.stdin.close()
    if proc.wait() != 0:
        raise RuntimeError(f"ffmpeg failed on {group.name} (exit code {proc.returncode})")


def _check_topic(ffmpeg, group, names, video_path, threads, progress=None):
    """Decode the video and compare every frame with the raw image. Returns frame count and PSNR."""
    h, w, _ = group[names[0]].shape
    reader = FrameReader(ffmpeg, video_path, w, h, threads)
    values, count = [], 0
    try:
        for name in names:
            frame = reader.read()
            if frame is None:
                break
            values.append(psnr(group[name][()], frame))
            count += 1
            if progress:
                progress.step("checking the video")
    finally:
        code, extra_bytes = reader.close()
    if code != 0:
        raise RuntimeError(f"ffmpeg could not decode {video_path.name} (exit code {code})")
    extra = extra_bytes // (w * h * 3)
    finite = [v for v in values if np.isfinite(v)] or [99.0]
    return {"frames_decoded": count + extra, "psnr_mean": float(np.mean(finite)), "psnr_min": float(np.min(finite))}


def encode_file(raw_path, out_dir, ffmpeg, threads=4, video_args=None, min_psnr=30.0, check=True):
    """Archive one raw recording. Returns the summary dict that is also written to <stem>.archive.json."""
    raw_path, out_dir = Path(raw_path), Path(out_dir)
    video_args = list(video_args or DEFAULT_VIDEO_ARGS)
    stem = raw_path.stem
    out_dir.mkdir(parents=True, exist_ok=True)
    summary_path = out_dir / f"{stem}.archive.json"
    work = out_dir / f".{stem}.partial"   # everything is written here first, then moved in place
    if work.exists():
        shutil.rmtree(work)
    work.mkdir()
    t0 = time.time()
    progress = None
    summary = {"format_version": FORMAT_VERSION, "source": raw_path.name, "source_bytes": raw_path.stat().st_size,
               "video_args": video_args, "cameras": {}, "files": {}, "ok": False, "errors": []}
    try:
        with h5py.File(raw_path, "r") as src, h5py.File(work / f"{stem}.h5", "w") as dst:
            _copy_attrs(src, dst)
            index = dst.create_group(ARCHIVE_GROUP)
            index.attrs["format_version"] = FORMAT_VERSION
            topics = message_groups(src)
            images = [t for t in topics if is_image_topic(src[t])]
            progress = Progress(out_dir / f".{stem}.progress.json", sum(len(src[t]) for t in images) * (2 if check else 1))
            # Copy everything that is not a camera image, unchanged (h5py copies data, attributes,
            # chunking and compression exactly). Groups on the way to image topics are recreated
            # with their attributes; their other children are copied.
            def copy_tree(sgroup, dgroup):
                for key, node in sgroup.items():
                    path = node.name
                    if path in images:
                        continue
                    if isinstance(node, h5py.Group) and any(i == path or i.startswith(path + "/") for i in images):
                        sub = dgroup.require_group(key)
                        _copy_attrs(node, sub)
                        copy_tree(node, sub)
                    else:
                        sgroup.copy(node, dgroup, name=key)
            copy_tree(src, dst)

            for topic in images:
                group = src[topic]
                names = sorted_names(group)
                ts = np.array([int(n) for n in names], dtype=np.int64)
                if any(str(t) != n for t, n in zip(ts, names)):
                    raise ValueError(f"{topic}: message names are not plain integer timestamps")
                first = group[names[0]]
                for name in names:
                    if len(group[name].attrs):
                        raise ValueError(f"{topic}/{name}: image datasets with attributes are not supported")
                fps = int(round(1e9 / np.median(np.diff(ts)))) if len(ts) > 1 else 30
                slug = camera_slug(topic)
                video = work / f"{stem}.{slug}.mp4"
                _encode_topic(ffmpeg, group, names, video, fps, threads, video_args, progress)

                entry = index.create_group(slug)
                entry.attrs["topic"] = topic
                entry.attrs["video"] = video.name
                entry.attrs["frame_shape"] = first.shape
                entry.attrs["dtype"] = str(first.dtype)
                entry.attrs["nominal_fps"] = fps
                # storage settings of the raw image datasets, so reconstruct_raw.py can write the same layout
                entry.attrs["compression"] = first.compression or ""
                entry.attrs["compression_opts"] = -1 if first.compression_opts is None or not np.isscalar(first.compression_opts) else int(first.compression_opts)
                entry.attrs["shuffle"] = bool(first.shuffle)
                entry.attrs["chunks"] = np.array(first.chunks if first.chunks else [0], dtype=np.int64)   # [0]: contiguous
                ga = entry.create_group("group_attrs")   # attributes of the topic group itself
                _copy_attrs(group, ga)
                entry.create_dataset("timestamps", data=ts)

                cam = {"topic": topic, "frames": len(names), "shape": list(first.shape), "nominal_fps": fps,
                       "video_bytes": video.stat().st_size}
                if check:
                    cam.update(_check_topic(ffmpeg, group, names, video, threads, progress))
                    if cam["frames_decoded"] != len(names):
                        summary["errors"].append(f"{slug}: {cam['frames_decoded']} frames decoded, {len(names)} recorded")
                    if cam["psnr_min"] < min_psnr:
                        summary["errors"].append(f"{slug}: lowest frame PSNR {cam['psnr_min']:.1f} dB < {min_psnr} dB")
                summary["cameras"][slug] = cam
            if not images:
                summary["errors"].append("no camera topic found")

        meta = raw_path.with_name(stem + ".meta.json")
        if meta.is_file():
            shutil.copy2(meta, work / meta.name)
        for f in sorted(work.iterdir()):
            summary["files"][f.name] = {"bytes": f.stat().st_size, "sha256": sha256(f)}
        summary["archive_bytes"] = sum(v["bytes"] for v in summary["files"].values())
        summary["seconds"] = round(time.time() - t0, 1)
        summary["ok"] = not summary["errors"]
        for f in list(work.iterdir()):
            os.replace(f, out_dir / f.name)
        write_json_atomic(summary_path, summary)   # last: its presence marks a complete archive
        return summary
    finally:
        shutil.rmtree(work, ignore_errors=True)
        if progress:
            progress.remove()


def is_archived(raw_path, out_dir):
    """True when a complete, checked archive of this recording exists in out_dir."""
    raw_path, out_dir = Path(raw_path), Path(out_dir)
    try:
        s = json.loads((out_dir / f"{raw_path.stem}.archive.json").read_text())
        return bool(s.get("ok")) and s.get("source_bytes") == raw_path.stat().st_size and all(
            (out_dir / name).is_file() and (out_dir / name).stat().st_size == info["bytes"] for name, info in s["files"].items())
    except Exception:
        return False


def _job(args):
    raw, out, ffmpeg, threads, video_args, min_psnr, check = args
    try:
        s = encode_file(raw, out, ffmpeg, threads, video_args, min_psnr, check)
        cams = ", ".join(f"{k} {v['video_bytes'] / 1e6:.1f} MB" + (f" {v['psnr_mean']:.1f} dB" if "psnr_mean" in v else "")
                         for k, v in s["cameras"].items())
        return raw, s["ok"], f"{'ok' if s['ok'] else 'PROBLEMS'}: {cams}; {s['seconds']} s" + (
            "" if s["ok"] else "; " + "; ".join(s["errors"]))
    except Exception as exc:
        return raw, False, f"FAILED: {exc}"


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("inputs", nargs="+", help="raw .h5 files or folders of them")
    p.add_argument("-o", "--output", required=True, help="archive folder (created if needed)")
    p.add_argument("--skip-existing", action="store_true", help="skip recordings that already have a complete, checked archive")
    p.add_argument("--jobs", type=int, default=1, help="recordings encoded at the same time")
    p.add_argument("--threads", type=int, default=4, help="ffmpeg threads per recording")
    p.add_argument("--crf", type=int, help="override the CRF (default 12)")
    p.add_argument("--min-psnr", type=float, default=30.0, help="fail when any frame's PSNR is below this (dB)")
    p.add_argument("--no-check", action="store_true", help="do not decode and compare after encoding")
    p.add_argument("--ffmpeg", help="ffmpeg binary (default: $FFMPEG, PATH, then imageio-ffmpeg)")
    args = p.parse_args(argv)

    ffmpeg = find_ffmpeg(args.ffmpeg)
    video_args = list(DEFAULT_VIDEO_ARGS)
    if args.crf is not None:
        video_args[video_args.index("-crf") + 1] = str(args.crf)
    files = []
    for i in args.inputs:
        i = Path(i)
        files += sorted(i.glob("*.h5")) if i.is_dir() else [i]
    out = Path(args.output)
    todo = [f for f in files if not (args.skip_existing and is_archived(f, out))]
    print(f"{len(files)} recording(s), {len(files) - len(todo)} already archived, {len(todo)} to encode with {ffmpeg}", flush=True)
    jobs = [(f, out, ffmpeg, args.threads, video_args, args.min_psnr, not args.no_check) for f in todo]
    failed = 0
    with concurrent.futures.ProcessPoolExecutor(max_workers=max(1, args.jobs)) as ex:
        for raw, ok, text in ex.map(_job, jobs):
            failed += not ok
            print(f"{Path(raw).name}: {text}", flush=True)
    if failed:
        print(f"{failed} recording(s) failed", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
