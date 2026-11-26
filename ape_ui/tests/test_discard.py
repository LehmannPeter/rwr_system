"""Delivery 2 of the background work (card 3): each demo is copied as soon as it is encoded, and demos can be discarded
(and the discard undone) next to View. Without ROS and without a Training machine: the server runs with its stand-in
monitor, demos are synthetic raw recordings, and a local folder stands in for the Training machine. Its after-transfer
step runs rwr_data.py merge-manifest there, so the Robot PC's keep value is seen arriving in the merged manifest.
Env: RWR_WS = an empty folder with src/rwr_system -> this checkout; APE_RSYNC = an rsync >= 3.2.3;
RWR_DATA_TOOL = rwr_il/scripts/rwr_data.py with the keep rule (merge-manifest).
Prints one line per check; "FAIL" marks a check that did not hold."""
import csv, json, os, shlex, sys, threading, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
WS = os.environ["RWR_WS"]
DEST_DIR = os.path.join(WS, "training_machine")
# the stand-in Training machine's after-transfer step: merge the inbox into its manifest (what after_transfer.sh does first)
AFTER = os.path.join(WS, "after_transfer_standin.sh")
os.makedirs(DEST_DIR, exist_ok=True)
with open(AFTER, "w") as f:
    f.write(f"#!/bin/bash\n{shlex.quote(sys.executable)} {shlex.quote(os.environ['RWR_DATA_TOOL'])} --data {shlex.quote(DEST_DIR)} "
            f"merge-manifest >> {shlex.quote(os.path.join(WS, 'merge.log'))} 2>&1\n")
os.chmod(AFTER, 0o755)
os.environ["APE_ARCHIVE_AFTER_TRANSFER"] = AFTER
import test_background as T   # the harness: server on port 8768, make_raw, api, wait, check (its run() is not used)
S = T.S
from pathlib import Path
import background
background.COPY_RETRY_S = 3   # retries of a copy that did not verify, quickly here
SES, RAW, DEST = T.SES, T.RAW, Path(DEST_DIR)
check, api, wait, make_raw, demos = T.check, T.api, T.wait, T.make_raw, T.demos
COLS = ["session", "file", "colour", "operator", "arena_pose", "cube_cell", "cube_orientation", "lighting", "description", "keep", "notes"]


def add_rows(*stems):
    """manifest.csv rows as APE's save writes them (rwr_data.py scan), on the Robot PC and, for the merge, an empty
    manifest on the stand-in Training machine."""
    for path in (S.DATA / "manifest.csv", DEST / "manifest.csv"):
        rows = list(csv.DictReader(open(path))) if path.exists() else []
        have = {r["file"] for r in rows}
        rows += [dict.fromkeys(COLS, "") | {"session": SES, "file": s + ".h5", "colour": "red", "keep": "keep"}
                 for s in stems if s + ".h5" not in have]
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, COLS, lineterminator="\n"); w.writeheader(); w.writerows(rows)


def keep_in(path, stem):
    try:
        return next((r["keep"] for r in csv.DictReader(open(path)) if r["file"] == stem + ".h5"), None)
    except FileNotFoundError:
        return None


def state(stem):
    return ((demos().get(stem) or {}).get("archive") or {}).get("state")


def save(stem, **kw):
    add_rows(stem); make_raw(stem, **kw); S.bg.wake()


def bglog():
    return open(S.bg.log_file).read()


def run():
    try:
        local = lambda cmd: list(cmd) + ["--local-dest", str(DEST)]
        orig_t, orig_c = S.station.transfer_command, S.station.copy_command
        S.station.transfer_command = lambda ds: local(orig_t(ds))
        S.station.copy_command = lambda ds, stems: local(orig_c(ds, stems))
        S.station.clear_command = lambda ds: local(S.station._archive_job_args("clear", ds))
        S.station.copy_enabled = lambda: True   # archive_host stands for the local folder here
        time.sleep(2)
        check("status lists the discard feature", "discard" in api("/api/status", post=False)["features"])

        print("--- a saved demo is encoded, then copied on its own right away")
        a = "20261126_160101"
        save(a)
        check("verified on the Training machine without pressing Transfer", wait(lambda: state(a) == "verified", 180), state(a))
        check("no transfer job was used", "transfer" not in str(S.job.dataset or ""), S.job.dataset)
        log = bglog()
        check("the copy took only that demo (--no-encode --demo)", f"copy {SES}/{a}:" in log and "--no-encode" in log and f"--demo {a}" in log)
        check("its archive and manifest.csv are on the Training machine", (DEST / "archive" / SES / f"{a}.archive.json").is_file()
              and (DEST / "manifest_inbox" / "robot_pc.csv").is_file())
        check("the after-transfer step ran there (merge)", wait(lambda: (Path(WS) / "merge.log").is_file() and "Merged" in (Path(WS) / "merge.log").read_text(), 30))
        check("copy came before the check of that demo", log.index(f"copy {SES}/{a}:") < log.index(f"check {SES}/{a}:") if f"check {SES}/{a}:" in log else True)

        print("--- the second demo is copied without the first one's files being compared again")
        b = "20261126_160202"
        save(b)
        check("second demo verified", wait(lambda: state(b) == "verified", 180), state(b))
        line = [l for l in bglog().splitlines() if l.startswith(f"--- ") and f"copy {SES}/{b}:" in l][-1]
        check("its copy names only that demo", f"--demo {b}" in line and f"--demo {a}" not in line, line[-120:])

        print("--- discard before encoding: never encoded, copied or checked")
        S.operate.rec = "recording"   # holds the background worker, so the demo waits
        c = "20261126_160303"
        save(c)
        r = api(f"/api/datasets/{SES}/demos/{c}/keep", {"keep": "discard"})
        check("discard accepted", r.get("keep") == "discard" and "not copied" in r.get("note", ""), f"{r}")
        check("manifest.csv on this PC says discard", keep_in(S.DATA / "manifest.csv", c) == "discard")
        S.operate.rec = "idle"; S.bg.wake(); time.sleep(8)
        m = demos()[c]
        check("shown as discarded, no work waiting", m["discarded"] and m["work"] is None and not m["archive"], f"{m['work']}, {m['archive']}")
        check("not encoded, not checked", not (S.DATA / "archive" / SES / f"{c}.archive.json").exists() and not m["check"])
        d = next(x for x in api("/api/datasets", post=False)["datasets"] if x["name"] == SES)
        check("session counts it as discarded and as removable by Clear", d["discarded"] == 1 and d["discard_clearable"] == 1 and d["to_encode"] == 0, f"{d['discarded']}, {d['discard_clearable']}, {d['to_encode']}")

        print("--- discard while it is being encoded: the encode stops and is not redone")
        e = "20261126_160404"
        save(e, size=(360, 640))
        check("encoding started", wait(lambda: S.bg.current is not None and S.bg.current["stem"] == e, 60, 0.05))
        api(f"/api/datasets/{SES}/demos/{e}/keep", {"keep": "discard"})
        check("encode stopped", wait(lambda: S.bg.current is None or S.bg.current["stem"] != e, 10, 0.05))
        time.sleep(6)
        check("no archive, no leftovers, not started again", not (S.DATA / "archive" / SES / f"{e}.archive.json").exists()
              and not list((S.DATA / "archive" / SES).glob(f".{e}*")) and bglog().count(f"encode {SES}/{e}:") == 1)
        check("log says why", f"encode {SES}/{e} stopped: its demo was discarded" in bglog())

        print("--- discard after the copy: manifest.csv goes over at once, the merge there takes the Robot PC's keep value")
        check("Training machine had it as keep", keep_in(DEST / "manifest.csv", a) == "keep", keep_in(DEST / "manifest.csv", a))
        r = api(f"/api/datasets/{SES}/demos/{a}/keep", {"keep": "discard"})
        check("note says it stays there but leaves dataset versions", "left out of dataset versions" in r.get("note", ""), r.get("note"))
        check("merged manifest there says discard", wait(lambda: keep_in(DEST / "manifest.csv", a) == "discard", 60), keep_in(DEST / "manifest.csv", a))
        check("merge log shows the change", "keep: 'keep' -> 'discard'" in (Path(WS) / "merge.log").read_text())
        check("its archive is still on the Training machine", (DEST / "archive" / SES / f"{a}.archive.json").is_file())
        m = demos()[a]
        check("listed as discarded, on the Training machine", m["discarded"] and m["archive"]["state"] == "verified")

        print("--- undo")
        r = api(f"/api/datasets/{SES}/demos/{a}/keep", {"keep": "keep"})
        check("undo after copy: keep again there", wait(lambda: keep_in(DEST / "manifest.csv", a) == "keep", 60), keep_in(DEST / "manifest.csv", a))
        api(f"/api/datasets/{SES}/demos/{e}/keep", {"keep": "keep"})
        check("undo before copy: encoded and copied as usual", wait(lambda: state(e) == "verified", 240), state(e))
        check("bad keep value refused", api(f"/api/datasets/{SES}/demos/{a}/keep", {"keep": "maybe"}).get("ERR") == 400)
        check("demo without a manifest row refused", api(f"/api/datasets/{SES}/demos/20261126_999999/keep", {"keep": "discard"}).get("ERR") == 400)

        print("--- a copy that does not verify is retried a few times, then left to Transfer new demos")
        g = "20261126_160505"
        (DEST / "archive" / SES).mkdir(parents=True, exist_ok=True)
        (DEST / "archive" / SES / f"{g}.h5").write_bytes(b"something else under the same name")   # --ignore-existing keeps it
        save(g)
        check("not verified", wait(lambda: state(g) == "not_verified", 180), state(g))
        tries = lambda: bglog().count(f"copy {SES}/{g}:")
        wait(lambda: tries() >= background.COPY_TRIES, 60)
        time.sleep(3 * background.COPY_RETRY_S + 2)
        check(f"copied {background.COPY_TRIES} times, then no more", tries() == background.COPY_TRIES, f"{tries()} copies")

        print("--- Transfer new demos leaves discarded demos out")
        h = "20261126_160606"
        S.operate.rec = "recording"
        save(h)
        api(f"/api/datasets/{SES}/demos/{h}/keep", {"keep": "discard"})
        S.operate.rec = "idle"
        api(f"/api/datasets/{SES}/transfer")
        check("transfer job ran", wait(lambda: S.job.running and S.job.dataset == "transfer " + SES, 60, 0.05) and wait(lambda: not S.job.running, 180), f"exit {S.job.exit_code}")
        wait(lambda: S.bg.current is None and not S.job.running, 60)   # a copy may still follow the transfer
        check("discarded demos neither encoded nor copied", not (S.DATA / "archive" / SES / f"{h}.archive.json").exists()
              and not list((DEST / "archive" / SES).glob(f"{h}.*")) and not list((DEST / "archive" / SES).glob(f"{c}.*")))

        print("--- Clear: verified demos lose their raw file; discarded ones too, copied or not")
        before = {s: (RAW / f"{s}.h5").is_file() for s in (a, b, c, e, g, h)}
        r = api(f"/api/datasets/{SES}/clear")
        check("clear ran", "ERR" not in r and wait(lambda: not S.job.running, 120), f"{r} exit {S.job.exit_code}")
        gone = {s: not (RAW / f"{s}.h5").is_file() for s in (a, b, c, e, g, h)}
        check("verified demos cleared (a, b, e)", gone[a] and gone[b] and gone[e], f"{gone}")
        check("discarded, never copied: raw deleted (c, h)", gone[c] and gone[h] and before[c] and before[h], f"{gone}")
        check("kept demo that did not verify stays (g)", not gone[g], f"{gone}")
        m = demos()[c]
        check("discarded demo listed as deleted here (never copied), with its manifest row", m["discarded"] and (m["archive"] or {}).get("state") == "deleted"
              and "never copied" in m["archive"]["text"]
              and keep_in(S.DATA / "manifest.csv", c) == "discard", f"{m['archive']}")
        d = next(x for x in api("/api/datasets", post=False)["datasets"] if x["name"] == SES)
        check("nothing discarded left to clear", d["discard_clearable"] == 0, d["discard_clearable"])
        check("a discarded demo whose raw file is gone and that has no archive cannot be undone from the page", not demos()[c]["viewable"])
    except Exception:
        import traceback; traceback.print_exc(); T.results.append(False)
    finally:
        print(f"=== {sum(T.results)} of {len(T.results)} checks ok", flush=True)
        S.cleanup(); os._exit(0 if all(T.results) else 1)


threading.Thread(target=run, daemon=True).start()
S.uvicorn.run(S.app, host="127.0.0.1", port=8768, log_level="critical")
