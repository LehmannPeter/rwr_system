# Test checklist: video archive, transfer and clearing (Robot PC, branch `video-archive`; sections 7–8: `archive-only`; section 9: background encoding)

Since 2026-11-26 (section 9) the button *Process and check new demos* is called *Transfer new demos* and nothing is
processed on the Robot PC; sections 1–8 describe the earlier behaviour.

Nothing here moves the robot. Do it before or after a recording session, with APE running and no launch needed.
Use a test session (e.g. `s90_<date>_test`) with 2–3 short demos; real sessions only after every step passed.

## 0. Setup
- [ ] `git fetch && git switch video-archive` in `~/Projects/rwr_system`; restart APE (`run.sh`).
- [ ] `pixi run python -c "import h5py, numpy"` works; `ffmpeg -hide_banner -encoders | grep libx264` lists libx264
      (else `pixi run pip install imageio-ffmpeg` and retry).
- [ ] `rsync --version` on the Robot PC is 3.2.3 or newer.
- [ ] Training machine has no system rsync; use the persistent copy: `ssh <alias> '~/.local/rsync/rsync --version'`
      prints 3.2.7. Set `"archive_remote_rsync": "~/.local/rsync/rsync"`.
- [ ] `ssh -o BatchMode=yes <alias> true` from the Robot PC succeeds without any prompt.
- [ ] Add to `ape_ui/static/local/station.json` (do not commit it): `"archive_host": "<alias>"`,
      `"archive_remote_rsync": "~/.local/rsync/rsync"` (and `"archive_data_root"` only if not `~/rwr_task2/data`).
      Leave `archive_transfer_processed` and `archive_require_hf_backup` at their defaults (true).

## 1. Archive without transfer (optional, first run)
- [ ] With `archive_host` removed, *Process and check new demos* on the test session: log shows
      "Archiving…", one line per demo `ok: … MB … dB`, then "archived only, nothing transferred".
- [ ] Column *Archive / Training machine* says *archived*; hover shows PSNR ≈ 43 dB per camera (full resolution; the lowest frame must be ≥ 30 dB or the demo is marked failed).
- [ ] `ls data/archive/<session>/`: per demo `.h5`, two `.mp4`, `.archive.json` (+ `.meta.json`); together ≈ 6 % of the raw file.
- [ ] Encoding took < 1 min per demo (about 30 s each here, 2 in parallel) and the live view / UI stayed responsive.

## 2. Transfer and verification
- [ ] Set `archive_host`, press *Process and check new demos* again: processing skips existing files, rsync lists
      the copied files, log ends "Verified on the Training machine: N of N demo(s)".
- [ ] Column shows *verified* for every demo (hover: "not backed up on Hugging Face yet"); *Clear transferred demos*
      becomes active.
- [ ] On the Training machine: `~/rwr_task2/data/archive/<session>/` and `processed/<session>_10hz/` exist,
      `sha256sum` of one mp4 matches the `sha256` in its `.archive.json`.
- [ ] On the Training machine `~/rwr_task2/data/manifest_inbox/robot_pc.csv` equals the Robot PC's `manifest.csv`;
      `rwr_data.py --data ~/rwr_task2/data merge-manifest --dry-run` lists the test demos as new rows
      (`merge-manifest` is on the rwr_il branch `manifest-merge`).
- [ ] Press *Process* a third time: nothing is re-encoded or re-sent, still all verified.

## 3. Hugging Face requirement
- [ ] Press *Clear transferred demos* before any HF backup: log lists every demo under "Kept … not backed up on
      Hugging Face", nothing is deleted.
- [ ] On the Training machine: `python src/logger/hf_backup_archive.py --session <session> --dry-run`, then without
      `--dry-run` (needs the HF token there). It ends "Backed up and verified on the Hub: N demo(s)" and writes
      `archive/<session>/<demo>.hf.json`.
- [ ] *Process* again (or Clear): the column shows *verified + HF*.

## 4. Mismatch protection (before trusting the delete)
- [ ] After step 3 (all demos *verified + HF*), on the Training machine append one byte to one copied file of one
      demo (`printf x >> ~/rwr_task2/data/archive/<session>/<demo>.oakd_front_view.mp4`).
- [ ] *Clear transferred demos* → confirm. Log: that demo is "missing or different", is **not** deleted; the others are.
- [ ] Column: the others show *cleared here*; their raw/processed/archive files are gone locally, their `.meta.json`
      and `manifest.csv` rows are still there, dataset names of the remaining demos unchanged.
- [ ] Repair the remote file (delete it on the Training machine), *Process* again → re-sent, *verified + HF* (the
      marker from step 3 still matches the original content); *Clear* now deletes it too.

## 5. Reconstruction on the Training machine
- [ ] `python src/logger/reconstruct_raw.py ~/rwr_task2/data/archive/<session> -o /tmp/recon --image-compression lzf`
- [ ] `python src/logger/syncronize_data.py /tmp/recon --cfg_file experiments/cfgs/logger/datacollection_right.yaml --compress --output_folder /tmp/recon_10hz`
      runs in plain Python (no ROS) and the result opens in APE's player / `inspect_demonstration.py`.

## 6. Guards
- [ ] *Clear* is disabled while recording, saving, with an unsaved demo and while a job runs; while a policy runs it
      is refused with a message.
- [ ] With the network cable to the Training machine unplugged: *Process* still archives, the log shows an rsync/ssh
      error, demos stay *not verified*, *Clear* deletes nothing.

## 7. Switch to archive-only transfer (between sessions, after sections 0–6 passed)
Do this only when no *Process* job runs and every session that was copied with processed files is fully transferred
(column *verified* for all its demos), so no session is half copied with and half without processed files.
- [ ] `git fetch && git switch archive-only` (or the branch it was merged into); restart APE (`run.sh`).
- [ ] In `ape_ui/static/local/station.json` set `"archive_transfer_processed": false`; leave
      `archive_require_hf_backup` as it is (see section 8). Restart APE.
- [ ] Record 2–3 short demos in a new test session (e.g. `s91_<date>_test`), *Process and check new demos*:
      the log shows only the `archive/` rsync (no `processed/` rsync), then "Verified on the Training machine: N of N".
- [ ] Set `"archive_after_transfer": "~/rwr_task2/after_transfer.sh"` in `station.json`, restart APE, *Process* again.
      The job log ends with "After-transfer step started on the Training machine" (not "not started").
- [ ] On the Training machine `~/rwr_task2/logs/after_transfer/latest.log` shows a run started at that time: steps
      merge-manifest, hf-backup, rebuild each "ok", and "after_transfer done". `processed/s91_<date>_test_10hz/` now
      holds one file per test demo (root attribute `rebuilt_from_video_archive`), the manifest has the test demos,
      and `archive/s91_<date>_test/<demo>.hf.json` exists for each.
- [ ] *Process* once more: a new `latest.log` shows the run with nothing to do (0 to rebuild, nothing uploaded).
- [ ] *Clear transferred demos* (with `archive_require_hf_backup: false` for this test only, or after section 8):
      raw, processed and archive files of the test demos are gone here; `.meta.json` and manifest rows stay.

## 8. Hugging Face backup required again
Only after the archive backup runs on the Training machine (`hf_backup_archive.py` ended with
"Backed up and verified on the Hub: N demo(s)" for the sessions that are still on this PC).
- [ ] In `station.json` set `"archive_require_hf_backup": true` (or remove the key: true is the default); restart APE.
- [ ] *Process* (or *Clear* on a session without HF backup yet): demos without a marker show *verified*, not
      *verified + HF*; *Clear* lists them under "Kept … not backed up on Hugging Face" and deletes nothing of them.
- [ ] After `hf_backup_archive.py` has run for that session on the Training machine: *Process* again shows
      *verified + HF*; *Clear* deletes them.

Report results in `handoff/ROBOT_PC_STATE.md`.

## 9. Background encoding, View, *Transfer new demos* (between sessions)
Nothing here moves the robot except recording the test demos, which needs the usual two people. Get the person's OK
before any *Clear*.
- [ ] The checkout APE runs from is on the commit with `ape_ui/background.py`; APE restarted (`run.sh`, full path).
- [ ] Card 3: the buttons are *Transfer new demos*, *Clear transferred demos*, *Stop*; there is no *Upload to
      Hugging Face* button and no *Processed GB* column.
- [ ] Record a new test session (e.g. `s92_<date>_test`) with 3 short demos (about 30 s each). After each save,
      without pressing anything: the demo appears with *waiting to encode*, then a progress bar *encoding NN %*, then
      *encoded* with its MB; afterwards the *Check* column goes from *waiting* to *ok* (or problems).
- [ ] Start the next recording while a demo is still *encoding*: the info line says "Background encoding paused: a
      demo is being recorded" and the bar stops; it continues right after *Save*. `work/ape_background.log` shows
      "suspended" and "continued" for that demo. Note how long one encode takes (times of its `--- encode` and
      `--- exit` lines, minus the time it was suspended).
- [ ] *View* on a demo right after its save (before it is encoded): it plays; the frame counter runs at about 30 per
      second of recording.
- [ ] **Stream gaps with and without encoding.** Restart APE with encoding allowed during recordings:
      `APE_ENCODE_WHILE_RECORDING=1 ~/Projects/rwr_system/ape_ui/run.sh` (ask the person first: a restart stops
      APE's launches). Record 3 more demos of the same length in the same session, each started while the previous one
      is still *encoding* (a policy must not run). Then restart APE normally (without the variable). Compare the
      largest gap per stream of the two groups:
      `python3 -c "import json,glob,sys; [print(f.split('/')[-2], {k.split('/')[-1] if 'color' not in k else k.split('/')[1]: round(v['max_gap_s'],3) for k,v in json.load(open(f))['streams'].items()}) for f in sorted(glob.glob(sys.argv[1]+'/*/summary.json'))]" ~/rwr_humble_ws/work/ape_previews/s92_<date>_test`
      Report both groups' numbers per stream (the first 3 demos: encoding held; the last 3: encoding running).
- [ ] Start a policy in preview (card 4) while a demo is being encoded: the info line says "Background encoding
      paused: a policy is running", the demo goes back to *waiting to encode*; after stopping the policy it is
      encoded again. (Skip if no policy is at hand.)
- [ ] Press *Transfer new demos*: if a demo is still being encoded the line says "Transfer asked for", and the
      transfer starts right after it. The bar above the table fills to 100 %, the demos show *copying…* and then
      *verified*; the log (card 3, *Transfer and clear log*) shows only the `archive/` rsync, "Verified on the Training
      machine: N of N demo(s)" and, if `archive_after_transfer` is set, "After-transfer step started".
- [ ] Get the person's OK, then *Clear transferred demos*: the demos show *cleared here*, `data/raw/<session>/` no
      longer holds their `.h5`, `data/archive/<session>/` still does, and *View* still plays them (title "(from the
      video archive)").
- [ ] Report back: the encode times, the gap table from above, whether the live view or the arm stalled while
      recording, and anything unexpected.
