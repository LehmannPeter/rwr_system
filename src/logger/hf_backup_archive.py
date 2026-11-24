#!/usr/bin/env python3
"""Back up the video archive to the team's private Hugging Face dataset repo (run on the Training machine).

Uploads only
    archive/<session>/*            (MP4s, non-image h5, .archive.json, .meta.json; never the .hf.json markers)
    manifest.csv                   (unless --no-manifest, or while manifest_inbox/ has unmerged changes;
                                    this is the only place that uploads manifest.csv to the Hub)
    datasets/*/version.csv
from the data root, keeping the paths (same layout as rwr_il/scripts/rwr_data.py uses on the Hub). Then it asks the
Hub for the sha256 of every uploaded archive file, compares it with the local file, and for each demo whose files
all match writes  archive/<session>/<demo>.hf.json  next to the archive files:
    {"repo", "revision", "at", "verified_on_hub": true, "files": {name: {"sha256", "bytes"}}}
APE's "Clear transferred demos" on the Robot PC fetches these markers and, by default, only deletes demos that have
one listing exactly its local archive files.

    python hf_backup_archive.py                                   # all sessions in ~/rwr_task2/data/archive
    python hf_backup_archive.py --session s04_2026-11-25_opA --dry-run
    python hf_backup_archive.py --data ~/rwr_task2/data --repo <user>/<repo>

Needs huggingface_hub (uses upload_large_folder where it exists, otherwise upload_folder, which in
huggingface_hub >= 2 is the resumable multi-commit upload) and a token with write access to the repo
(`hf auth login` once, or $HF_TOKEN). Refuses to upload to a public repo (course rule: no data sharing
between teams). Re-running is safe: files already on the Hub are skipped, demos with a valid marker are skipped.
"""
import argparse
import hashlib
import json
import os
import sys
import time
from pathlib import Path

MARKER_SUFFIX = ".hf.json"
IGNORE = [".*", "**/.*", "*.part", "*.tmp", "*" + MARKER_SUFFIX, "**/*" + MARKER_SUFFIX]


def sha256(path, block=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(block):
            h.update(chunk)
    return h.hexdigest()


def git_blob_sha1(path):
    """The git object id of a file, which the Hub reports for files that are not stored with LFS/Xet."""
    data = Path(path).read_bytes()
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def demo_files(session_dir, stem):
    """{name: Path} of one demo's archive files as listed in its .archive.json (+ the .archive.json), or an error."""
    summary_path = session_dir / f"{stem}.archive.json"
    summary = json.loads(summary_path.read_text())
    if not summary.get("ok"):
        return None, "archive marked as not ok: " + "; ".join(summary.get("errors", []))
    files = {summary_path.name: summary_path}
    for name, info in summary.get("files", {}).items():
        p = session_dir / name
        if not p.is_file():
            return None, f"{name} missing"
        if p.stat().st_size != info["bytes"] or sha256(p) != info["sha256"]:
            return None, f"{name} differs from its .archive.json (damaged copy?)"
        files[name] = p
    return files, None


def unmerged_inbox(data):
    """True when a manifest sent by the Robot PC is newer than the last `rwr_data.py merge-manifest`."""
    inbox = data / "manifest_inbox"
    sent = [f.stat().st_mtime for f in inbox.glob("*.csv")] if inbox.is_dir() else []
    if not sent:
        return False
    stamp = inbox / "last_merge.txt"
    return not stamp.exists() or max(sent) > stamp.stat().st_mtime


def marker_valid(session_dir, stem, files):
    try:
        m = json.loads((session_dir / f"{stem}{MARKER_SUFFIX}").read_text())
    except Exception:
        return False
    return m.get("verified_on_hub") and all(m["files"].get(n, {}).get("sha256") == sha256(p) for n, p in files.items())


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--data", default=os.environ.get("RWR_DATA_DIR", "~/rwr_task2/data"), help="data root (default ~/rwr_task2/data)")
    p.add_argument("--repo", help="<user or org>/<name> (default: <data>/hf_repo.txt)")
    p.add_argument("--session", action="append", help="only these sessions (repeatable); default all in <data>/archive")
    p.add_argument("--no-manifest", action="store_true", help="do not upload manifest.csv")
    p.add_argument("--force", action="store_true", help="upload and verify demos again even if they have a valid marker")
    p.add_argument("--dry-run", action="store_true", help="only list what would be uploaded")
    args = p.parse_args(argv)

    data = Path(args.data).expanduser()
    repo = args.repo or (data / "hf_repo.txt").read_text().strip()
    archive = data / "archive"
    sessions = args.session or sorted(d.name for d in archive.iterdir() if d.is_dir() and not d.name.startswith("."))

    todo, skipped, problems = [], 0, []   # todo: (session_dir, stem, {name: path})
    for s in sessions:
        sd = archive / s
        for summary in sorted(sd.glob("*.archive.json")):
            stem = summary.name[: -len(".archive.json")]
            files, err = demo_files(sd, stem)
            if err:
                problems.append(f"{s}/{stem}: {err}")
                continue
            if not args.force and marker_valid(sd, stem, files):
                skipped += 1
                continue
            todo.append((sd, stem, files))
    paths = [f"archive/{sd.name}/{name}" for sd, _, files in todo for name in files]
    upload_manifest = not args.no_manifest
    if upload_manifest and unmerged_inbox(data):
        print("manifest_inbox/ has changes that are not merged yet: manifest.csv is NOT uploaded this time. "
              "Run `rwr_data.py --data <data> merge-manifest` first.", flush=True)
        upload_manifest = False
    extra = (["manifest.csv"] if upload_manifest else []) + ["datasets/*/version.csv"]
    size = sum(p.stat().st_size for _, _, files in todo for p in files.values())
    print(f"{repo}: {len(todo)} demo(s) to back up ({size / 1e9:.2f} GB, {len(paths)} files), {skipped} already backed up, "
          f"{len(problems)} with problems; plus {', '.join(extra)}", flush=True)
    for line in problems:
        print("  PROBLEM " + line, flush=True)
    if args.dry_run or not todo:
        return 1 if problems else 0

    from huggingface_hub import HfApi
    api = HfApi()
    api.create_repo(repo, repo_type="dataset", private=True, exist_ok=True)   # creates it private; never changes an existing repo
    if not api.repo_info(repo, repo_type="dataset").private:
        raise SystemExit(f"{repo} is public. Refusing to upload recordings (course rule: data may not be shared between teams).")

    allow = paths + extra
    if hasattr(api, "upload_large_folder"):   # huggingface_hub 0.25 - 1.x
        api.upload_large_folder(repo_id=repo, folder_path=data, repo_type="dataset", allow_patterns=allow, ignore_patterns=IGNORE)
    else:                                     # huggingface_hub >= 2: upload_folder is the streamed, resumable upload
        api.upload_folder(repo_id=repo, folder_path=data, repo_type="dataset", allow_patterns=allow, ignore_patterns=IGNORE,
                          commit_message=f"Archive backup: {len(todo)} demo(s) from {', '.join(sorted({sd.name for sd, _, _ in todo}))}")
    revision = api.repo_info(repo, repo_type="dataset").sha

    # Verify: what the Hub stores must have the same content as the local files.
    remote = {}
    for i in range(0, len(paths), 100):
        for f in api.get_paths_info(repo, paths[i:i + 100], repo_type="dataset", revision=revision):
            remote[f.path] = f
    ok_demos, failed = 0, []
    for sd, stem, files in todo:
        bad, entries = [], {}
        for name, path in files.items():
            f = remote.get(f"archive/{sd.name}/{name}")
            digest = sha256(path)
            if f is None:
                bad.append(f"{name} not on the Hub")
            elif getattr(f, "lfs", None) is not None:
                if f.lfs.sha256 != digest:
                    bad.append(f"{name} sha256 differs on the Hub")
            elif f.blob_id != git_blob_sha1(path):
                bad.append(f"{name} differs on the Hub")
            entries[name] = {"sha256": digest, "bytes": path.stat().st_size}
        if bad:
            failed.append(f"{sd.name}/{stem}: " + "; ".join(bad))
            continue
        marker = {"repo": repo, "revision": revision, "at": time.strftime("%Y-%m-%d %H:%M:%S"), "verified_on_hub": True, "files": entries}
        tmp = sd / f".{stem}{MARKER_SUFFIX}.tmp"
        tmp.write_text(json.dumps(marker, indent=1))
        tmp.replace(sd / f"{stem}{MARKER_SUFFIX}")
        ok_demos += 1
    print(f"Backed up and verified on the Hub: {ok_demos} demo(s) at revision {revision[:10]}.", flush=True)
    for line in failed:
        print("  NOT VERIFIED " + line, flush=True)
    return 1 if failed or problems else 0


if __name__ == "__main__":
    sys.exit(main())
