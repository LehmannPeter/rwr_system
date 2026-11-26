#!/usr/bin/env python3
"""Back up the video archive to the team's Hugging Face dataset repo (run on the Training machine).

Uploads only these files from the data root, keeping the paths (same layout as rwr_il/scripts/rwr_data.py uses):
    archive/<session>/*            (MP4s, non-image h5, .archive.json, .meta.json of every complete archive;
                                    never the .hf.json markers)
    manifest.csv                   (unless --no-manifest, or while manifest_inbox/ has unmerged changes)
    datasets/*/version.csv         (the episode lists of the dataset versions, not their files)
    provenance/archive_backup.json (written here: git commits of rwr_system and rwr_il, huggingface_hub version,
                                    time, demos per session)
Never processed/, raw/ or the files of datasets/: the file list is built explicitly, nothing is globbed.

Before uploading, it asks the Hub which archive files are already there with the same content (a demo whose files
are all there only gets its marker). After uploading, it checks again: for files stored with LFS/Xet the Hub's
sha256; for small files stored in git (the .json files) it downloads them and computes the sha256. Only a demo whose
every file matches gets  archive/<session>/<demo>.hf.json  next to its archive files:
    {"repo", "revision", "at", "verified_on_hub": true, "files": {name: {"sha256", "bytes"}}}
APE's "Clear transferred demos" on the Robot PC fetches these markers and (with archive_require_hf_backup) only
deletes demos that have one listing exactly its local archive files.

Visibility: the course rule is no data sharing between teams, so it refuses to upload to a public repo unless
--allow-public is given (the team's decision of 2026-11-25 to keep the repo public for now). To go private later:
make the repo private on the Hub and remove --allow-public from the command that runs this. A new repo is always
created private; the visibility of an existing repo is never changed.

    python hf_backup_archive.py --dry-run                         # what would be uploaded (asks the Hub, read-only)
    python hf_backup_archive.py --allow-public                    # needed while the repo is public
    python hf_backup_archive.py --session s04_2026-11-25_Clarence --dry-run
    python hf_backup_archive.py --data ~/rwr_task2/data --repo <user>/<repo>

Needs huggingface_hub (>= 1; with hf_xet, upload_folder is the streamed upload that resumes an interrupted run)
and a token with write access (`hf auth login` once, or $HF_TOKEN). Re-running is safe: demos with a valid marker
are skipped without asking the Hub, files already on the Hub are not sent again. Only one run at a time.
"""
import argparse
import fcntl
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

MARKER_SUFFIX = ".hf.json"
PROVENANCE = "provenance/archive_backup.json"
IGNORE = [".*", "**/.*", "*.part", "*.tmp", "*" + MARKER_SUFFIX, "**/*" + MARKER_SUFFIX]
REPO = Path(__file__).resolve().parent.parent.parent


def sha256(path, block=1 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(block):
            h.update(chunk)
    return h.hexdigest()


def git_info(repo):
    def run(*a):
        return subprocess.run(["git", "-C", str(repo), *a], capture_output=True, text=True).stdout.strip()
    if not Path(repo, ".git").exists():
        return None
    return {"commit": run("rev-parse", "HEAD"), "branch": run("branch", "--show-current"),
            "dirty": bool(run("status", "--porcelain", "--untracked-files=no"))}


def git_blob_sha1(path):
    """The git object id of a file, which the Hub reports for small files stored in git (not LFS/Xet)."""
    data = Path(path).read_bytes()
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def changed_on_hub(api, repo, revision, data, rel_paths):
    """The metadata files (small, stored in git) that are missing on the Hub or differ from the local file."""
    remote = {f.path: f for f in api.get_paths_info(repo, rel_paths, repo_type="dataset", revision=revision)} if revision else {}
    return [rp for rp in rel_paths if rp not in remote or getattr(remote[rp], "blob_id", None) != git_blob_sha1(data / rp)]


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
            return None, f"{name} missing (still arriving?)"
        if p.stat().st_size != info["bytes"] or sha256(p) != info["sha256"]:
            return None, f"{name} differs from its .archive.json (damaged or incomplete copy?)"
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
    return bool(m.get("verified_on_hub")) and set(m.get("files", {})) == set(files) and all(
        m["files"][n].get("sha256") == sha256(p) for n, p in files.items())


def hub_check(api, repo, revision, demos, scratch):
    """For each (session_dir, stem, files): ({name: problem} of missing or differing files, {name: {sha256, bytes}}), vs the
    Hub at `revision`. LFS/Xet files: the sha256 the Hub reports; git files: downloaded and hashed."""
    paths = [f"archive/{sd.name}/{n}" for sd, _, files in demos for n in files]
    remote = {}
    for i in range(0, len(paths), 100):
        for f in api.get_paths_info(repo, paths[i:i + 100], repo_type="dataset", revision=revision):
            remote[f.path] = f
    out = {}
    for sd, stem, files in demos:
        bad, entries = {}, {}
        for name, path in files.items():
            rp, digest = f"archive/{sd.name}/{name}", sha256(path)
            entries[name] = {"sha256": digest, "bytes": path.stat().st_size}
            f = remote.get(rp)
            if f is None or not hasattr(f, "size"):
                bad[name] = "not on the Hub"
            elif getattr(f, "lfs", None) is not None:
                if f.lfs.sha256 != digest:
                    bad[name] = "another sha256 on the Hub"
            else:
                got = api.hf_hub_download(repo, rp, repo_type="dataset", revision=revision, local_dir=scratch)
                if sha256(got) != digest:
                    bad[name] = "another sha256 on the Hub"
        out[(sd, stem)] = (bad, entries)
    return out


def write_marker(sd, stem, repo, revision, entries):
    marker = {"repo": repo, "revision": revision, "at": time.strftime("%Y-%m-%d %H:%M:%S"), "verified_on_hub": True,
              "files": entries}
    tmp = sd / f".{stem}{MARKER_SUFFIX}.tmp"
    tmp.write_text(json.dumps(marker, indent=1))
    tmp.replace(sd / f"{stem}{MARKER_SUFFIX}")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--data", default=os.environ.get("RWR_DATA_DIR", "~/rwr_task2/data"), help="data root (default ~/rwr_task2/data)")
    p.add_argument("--repo", help="<user or org>/<name> (default: <data>/hf_repo.txt)")
    p.add_argument("--session", action="append", help="only these sessions (repeatable); default all in <data>/archive")
    p.add_argument("--no-manifest", action="store_true", help="do not upload manifest.csv")
    p.add_argument("--allow-public", action="store_true",
                   help="upload even though the repo is public (explicit opt-in; remove it once the repo is private)")
    p.add_argument("--rwr-il", default="~/rwr_task2/rwr_il", help="rwr_il checkout whose commit goes into the provenance file")
    p.add_argument("--force", action="store_true", help="check demos on the Hub again even if they have a valid marker")
    p.add_argument("--dry-run", action="store_true", help="only list what would be uploaded (asks the Hub, read-only)")
    p.add_argument("--offline", action="store_true", help="with --dry-run: do not ask the Hub either")
    args = p.parse_args(argv)

    data = Path(args.data).expanduser().resolve()
    repo = args.repo or (data / "hf_repo.txt").read_text().strip()
    archive = data / "archive"
    sessions = args.session or sorted(d.name for d in archive.iterdir() if d.is_dir() and not d.name.startswith("."))
    lock_dir = Path(tempfile.gettempdir()) / "rwr_hf_backup"
    lock_dir.mkdir(exist_ok=True)
    lock = open(lock_dir / "lock", "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("Another hf_backup_archive.py is running: nothing to do.", flush=True)
        return 0

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

    upload_manifest = not args.no_manifest and (data / "manifest.csv").is_file()
    if upload_manifest and unmerged_inbox(data):
        print("manifest_inbox/ has changes that are not merged yet: manifest.csv is NOT uploaded this time. "
              "Run `rwr_data.py --data <data> merge-manifest` first.", flush=True)
        upload_manifest = False
    extra = (["manifest.csv"] if upload_manifest else []) + sorted(
        str(f.relative_to(data)) for f in (data / "datasets").glob("*/version.csv") if not f.parent.name.startswith("."))
    git = {"rwr_system": git_info(REPO), "rwr_il": git_info(Path(args.rwr_il).expanduser())}
    try:
        import huggingface_hub
        hub_version = huggingface_hub.__version__
    except ImportError:
        hub_version = None
    per_session = {}
    for sd, _, _ in todo:
        per_session[sd.name] = per_session.get(sd.name, 0) + 1
    provenance = {"written_at": time.strftime("%Y-%m-%d %H:%M:%S %Z"), "by": "rwr_system src/logger/hf_backup_archive.py",
                  "git": git, "huggingface_hub": hub_version, "repo": repo,
                  "demos_in_this_run": per_session, "demos_already_backed_up": skipped}

    print(f"{repo}: {len(todo)} demo(s) without a valid marker, {skipped} already backed up, {len(problems)} with "
          f"problems; plus {len(extra)} metadata file(s) and {PROVENANCE}", flush=True)
    for line in problems:
        print("  PROBLEM " + line, flush=True)
    if args.dry_run and args.offline:
        for sd, stem, files in todo:
            print(f"  would upload archive/{sd.name}/{stem}.* ({len(files)} files, {sum(f.stat().st_size for f in files.values()) / 1e6:.1f} MB)")
        return 1 if problems else 0

    from huggingface_hub import HfApi
    from huggingface_hub.errors import RepositoryNotFoundError
    api = HfApi()
    try:
        info = api.repo_info(repo, repo_type="dataset")
        private, revision = bool(info.private), info.sha
    except RepositoryNotFoundError:
        private, revision = None, None   # will be created private
    if private is False and not args.allow_public:
        print(f"{repo} is PUBLIC. Refusing to upload recordings (course rule: data may not be shared between teams). "
              "Pass --allow-public to upload anyway (team decision), or make the repo private first.", flush=True)
        if not args.dry_run:
            return 2
    elif private is False:
        print(f"WARNING: {repo} is PUBLIC; uploading because --allow-public was given.", flush=True)

    # demos whose files are all on the Hub already (e.g. uploaded by a run that died before writing markers)
    with tempfile.TemporaryDirectory(prefix="hf_check_") as scratch:
        before = hub_check(api, repo, revision, todo, scratch) if revision and todo else {}
    already = [(sd, st, files) for sd, st, files in todo if (sd, st) in before and not before[(sd, st)][0]]
    upload = [(sd, st, files) for sd, st, files in todo if (sd, st) not in before or before[(sd, st)][0]]
    # only the files that are not on the Hub yet with the same content
    paths = [f"archive/{sd.name}/{n}" for sd, st, files in upload for n in files
             if (sd, st) not in before or n in before[(sd, st)][0]]
    size = sum((data / rp).stat().st_size for rp in paths)
    print(f"On the Hub already with the same sha256: {len(already)} demo(s) (marker only). To upload: {len(upload)} demo(s), "
          f"{len(paths)} archive file(s), {size / 1e9:.2f} GB; plus {', '.join(extra + [PROVENANCE])}", flush=True)

    if args.dry_run:
        by_session = {}
        for rp in paths:
            s = rp.split("/")[1]
            n, b = by_session.get(s, (0, 0))
            by_session[s] = (n + 1, b + (data / rp).stat().st_size)
        for s, (n, b) in sorted(by_session.items()):
            print(f"  archive/{s}/: {n} files, {b / 1e9:.3f} GB")
        for sd, st, _ in already:
            print(f"  marker only: archive/{sd.name}/{st}{MARKER_SUFFIX}")
        for rp in extra:
            print(f"  {rp} ({(data / rp).stat().st_size / 1e3:.1f} kB)")
        print(f"  {PROVENANCE}: " + json.dumps(provenance), flush=True)
        print("Dry run: nothing uploaded, no marker written.", flush=True)
        return 1 if problems else 0

    for sd, st, files in already:
        write_marker(sd, st, repo, revision, before[(sd, st)][1])
    if not upload:   # no new demos: send only metadata that changed, and no new provenance file (no empty commits)
        extra = changed_on_hub(api, repo, revision, data, extra)
        if not extra:
            print(f"Nothing to upload: archives and metadata are on the Hub. Markers written: {len(already)}.", flush=True)
            return 1 if problems else 0
        print("Only metadata changed: " + ", ".join(extra), flush=True)

    prov_path = data / PROVENANCE
    prov_path.parent.mkdir(exist_ok=True)
    tmp = prov_path.with_name(prov_path.name + ".tmp")
    tmp.write_text(json.dumps(provenance, indent=1))
    tmp.replace(prov_path)
    if revision is None:
        api.create_repo(repo, repo_type="dataset", private=True, exist_ok=True)   # a new repo is always private
    allow = paths + extra + [PROVENANCE]
    api.upload_folder(repo_id=repo, folder_path=data, repo_type="dataset", allow_patterns=allow, ignore_patterns=IGNORE,
                      commit_message=f"Archive backup: {len(upload)} demo(s)"
                      + (f" from {', '.join(sorted({sd.name for sd, _, _ in upload}))}" if upload else ""))
    revision = api.repo_info(repo, repo_type="dataset").sha

    with tempfile.TemporaryDirectory(prefix="hf_check_") as scratch:
        after = hub_check(api, repo, revision, upload, scratch) if upload else {}
    ok_demos, failed = len(already), []
    for sd, stem, files in upload:
        bad, entries = after[(sd, stem)]
        if bad:
            failed.append(f"{sd.name}/{stem}: " + "; ".join(f"{n}: {why}" for n, why in bad.items()))
            continue
        write_marker(sd, stem, repo, revision, entries)
        ok_demos += 1
    print(f"Backed up and verified on the Hub: {ok_demos} demo(s) at revision {revision[:10]}.", flush=True)
    for line in failed:
        print("  NOT VERIFIED " + line, flush=True)
    return 1 if failed or problems else 0


if __name__ == "__main__":
    sys.exit(main())
