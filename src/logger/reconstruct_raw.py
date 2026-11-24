#!/usr/bin/env python3
"""Turn an archive written by encode_archive.py back into a raw-format HDF5 (the logger's layout),
so syncronize_data.py can process it like the original recording.

Every non-image topic and all attributes come back bit for bit; the images come from the MP4s
(lossy). By default the image datasets get the storage settings of the original raw file (e.g. gzip);
--image-compression none or lzf is much faster when the file is only an input for syncronize_data.py.

Plain Python (h5py, numpy, ffmpeg); no ROS.

    python reconstruct_raw.py archive/<session>               -o raw_reconstructed/<session>
    python reconstruct_raw.py archive/<session>/<stem>.h5     -o raw_reconstructed/<session> --image-compression lzf
"""
import argparse
import concurrent.futures
import json
import os
import shutil
import sys
from pathlib import Path

import h5py
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from video_archive_common import ARCHIVE_GROUP, FrameReader, find_ffmpeg  # noqa: E402


def _copy_attrs(src, dst):
    for k, v in src.attrs.items():
        dst.attrs[k] = v


def reconstruct_file(archive_h5, out_dir, ffmpeg, threads=4, image_compression="keep"):
    archive_h5, out_dir = Path(archive_h5), Path(out_dir)
    folder, stem = archive_h5.parent, archive_h5.stem
    summary = json.loads((folder / f"{stem}.archive.json").read_text())
    if not summary.get("ok"):
        raise RuntimeError(f"{stem}: the archive is marked as not ok: {summary.get('errors')}")
    out_dir.mkdir(parents=True, exist_ok=True)
    target = out_dir / f"{stem}.h5"
    tmp = out_dir / f".{stem}.h5.part"
    try:
        with h5py.File(archive_h5, "r") as src, h5py.File(tmp, "w") as dst:
            _copy_attrs(src, dst)
            for key in src:
                if key != ARCHIVE_GROUP:
                    src.copy(src[key], dst, name=key)
            for slug, entry in src[ARCHIVE_GROUP].items():
                topic = entry.attrs["topic"]
                group = dst.require_group(topic)
                _copy_attrs(entry["group_attrs"], group)
                ts = entry["timestamps"][()]
                h, w, c = (int(v) for v in entry.attrs["frame_shape"])
                options = {}
                if image_compression == "keep":
                    comp = entry.attrs["compression"]
                    if comp:
                        options["compression"] = comp
                        opts = int(entry.attrs["compression_opts"])
                        if opts >= 0:
                            options["compression_opts"] = opts
                    if bool(entry.attrs["shuffle"]):
                        options["shuffle"] = True
                    chunks = tuple(int(v) for v in entry.attrs["chunks"])
                    if chunks != (0,):
                        options["chunks"] = chunks
                elif image_compression == "lzf":
                    options["compression"] = "lzf"
                reader = FrameReader(ffmpeg, folder / entry.attrs["video"], w, h, threads)
                try:
                    for i, t in enumerate(ts):
                        frame = reader.read()
                        if frame is None:
                            raise RuntimeError(f"{entry.attrs['video']}: only {i} of {len(ts)} frames could be decoded")
                        group.create_dataset(str(int(t)), data=frame, **options)
                finally:
                    code, extra = reader.close()
                if code != 0 or extra:
                    raise RuntimeError(f"{entry.attrs['video']}: decoding ended with code {code}, {extra} extra bytes")
        os.replace(tmp, target)
    finally:
        if tmp.exists():
            tmp.unlink()
    meta = folder / f"{stem}.meta.json"
    if meta.is_file():
        shutil.copy2(meta, out_dir / meta.name)
    return target


def _job(args):
    archive_h5, out, ffmpeg, threads, comp = args
    try:
        return archive_h5, True, str(reconstruct_file(archive_h5, out, ffmpeg, threads, comp))
    except Exception as exc:
        return archive_h5, False, f"FAILED: {exc}"


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("inputs", nargs="+", help="archive .h5 files (next to their .mp4 and .archive.json) or archive folders")
    p.add_argument("-o", "--output", required=True, help="folder for the reconstructed raw .h5 files")
    p.add_argument("--image-compression", choices=["keep", "none", "lzf"], default="keep",
                   help="storage of the image datasets: as in the original raw file (default), none, or lzf")
    p.add_argument("--skip-existing", action="store_true")
    p.add_argument("--jobs", type=int, default=1)
    p.add_argument("--threads", type=int, default=4, help="ffmpeg decoding threads per file")
    p.add_argument("--ffmpeg")
    args = p.parse_args(argv)
    ffmpeg = find_ffmpeg(args.ffmpeg)
    files = []
    for i in args.inputs:
        i = Path(i)
        files += sorted(f.with_name(f.name[: -len(".archive.json")] + ".h5") for f in i.glob("*.archive.json")) if i.is_dir() else [i]
    out = Path(args.output)
    todo = [f for f in files if not (args.skip_existing and (out / f.name).exists())]
    print(f"{len(files)} archive(s), {len(todo)} to reconstruct", flush=True)
    failed = 0
    with concurrent.futures.ProcessPoolExecutor(max_workers=max(1, args.jobs)) as ex:
        for f, ok, text in ex.map(_job, [(f, out, ffmpeg, args.threads, args.image_compression) for f in todo]):
            failed += not ok
            print(f"{Path(f).name}: {text}", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
