# Instructions for the Robot PC

Written only on the Mac. The session on the Robot PC carries out every instruction whose ID is not yet listed in `ROBOT_PC_STATE.md`, and records the result there. Rules: `handoff/README.md`.

Template:

```
## <date>-<nn>: <title>
Why: <one line>
Steps: <numbered, exact commands where possible>
Report back: <what to write into ROBOT_PC_STATE.md>
```

---

## 2026-11-21-01: Identify this machine and report its setup
Why: the Mac session knows nothing about the Robot PC yet.
Steps:
1. Write `robot_pc` into `.machine` at the root of this repo and of the `rwr_il` checkout.
2. Fill every row of the "Setup" table in `ROBOT_PC_STATE.md`. For the GPU give the `nvidia-smi` model and memory, or "none".
Report back: the filled table.

## 2026-11-23-01: Cut the failed last 10 s off the first session 03 episode
Why: in `s03_2026-11-21_Clarence/20261121_215255.h5` the arm stops following the glove about 9 s before the end (the command runs up to 77 cm away from the arm while the arm holds the cube in the air). The person asked to remove that part from the raw and the processed file. The Mac has no raw file and only a read token for Hugging Face, so it has to happen here.
Steps:
1. Get the tool without changing the branch of the `rwr_il` checkout: `cd <rwr_il checkout> && git fetch origin && git worktree add ~/rwr_il_training_levers origin/training-levers` (if the folder exists already: `git -C ~/rwr_il_training_levers checkout --detach origin/training-levers`).
2. Dry run, in the pixi environment (needs `h5py`, `numpy`): `python3 ~/rwr_il_training_levers/scripts/cut_episode.py s03_2026-11-21_Clarence 20261121_215255.h5 --end-frame 1447`. It must print "remove 100 frames (10.0 s)" and the cut-off time stamp `1795294309933333109`. If either differs, stop and report; do not apply.
3. Check the free disk space: the rewrite needs room for a second copy of the raw file. Then ask the person at the station for their go-ahead and run the same command with `--apply`. The originals stay next to the new files as `*.h5.uncut`.
4. Upload the cut processed file in place of the old one: `pixi run hf upload PeterLehmann/rwr-task2 ~/rwr_humble_ws/data/processed/s03_2026-11-21_Clarence_10hz/20261121_215255.h5 processed/s03_2026-11-21_Clarence_10hz/20261121_215255.h5 --repo-type dataset`, then `rwr_data.py push --no-raw` for the manifest note. (The raw files of session 03 are not on Hugging Face; leave that as it is unless the person says otherwise.)
5. Do not delete the `.uncut` files. Tell the person they are there and how large they are; deleting them is their decision.
Report back: the output of step 2 and step 3 (frames kept and removed per raw topic), the sizes before and after, whether the upload succeeded, and where the `.uncut` files are.

## 2026-11-25-01: Video archive: set up the transfer, archive s01–s03 if their raw files still exist, run the test checklist
Why: the video archive code (branch `video-archive`; `policy-eval` contains the same code) adds to APE's "Process and check new demos" an MP4 archive of every raw recording (about 6 % of its size), a checksum-verified copy to the Training machine and a guarded "Clear transferred demos". Most raw recordings of s01–s03 never reached Hugging Face, so if they are still on this PC they are the only copy.
When: only **after** the team has finished testing the `policy-eval` branch (both repos) and has set up ssh to the Training machine. Do not start while that test is still going on.
Where this instruction lives: on `main`. If this checkout is on another branch (e.g. `policy-eval`), read the current version with `git show origin/main:handoff/TO_ROBOT_PC.md` instead of the file in the checkout, and commit the report on `main` through a worktree as described at the end.
Steps:
1. Ask the person at the station before you start: steps 6–7 run APE jobs that keep the CPU busy (do not run them during recording or a policy run), and the checklist in step 7 contains deletion tests. No step moves the robot or the hand.
2. Stay on the current branch (no switch); check that it is clean and has the archive code:
   ```bash
   cd ~/Projects/rwr_system && git branch --show-current && git log -1 --oneline    # expected: policy-eval; write both down for the report
   git status --short    # must be empty; otherwise stop and report (blocked), do not stash or discard anything
   git fetch origin
   for f in ape_ui/archive_job.py src/logger/encode_archive.py; do
     if [ -f "$f" ] && git diff --quiet origin/video-archive -- "$f"; then echo "$f: same as video-archive"; else echo "$f: MISSING or DIFFERENT"; fi
   done
   ```
   Both lines must say "same as video-archive". If either says MISSING or DIFFERENT, the checkout is on a branch without the archive code (or with another version of it): stop and report (blocked); do not switch branches. `rwr_il` stays on its current branch: the archive does not use it.
3. ssh to the Training machine, host alias `rwr-instance`:
   - If `ssh -o BatchMode=yes rwr-instance true` already returns at once without any prompt and `ssh -G rwr-instance | grep -i identityfile` shows `~/.ssh/rwr_collect` (the team set it up before the policy-eval test), skip the setup below and only do the version checks in the last bullet.
   - Copy the gateway key from the team zip into `~/.ssh/` (`chmod 600`). Gateway host, user and port are in the zip; never copy them into this repo.
   - Make a new passwordless key for the transfers: `ssh-keygen -t ed25519 -N "" -f ~/.ssh/rwr_collect -C "robot-pc rwr_collect"`.
   - Send `~/.ssh/rwr_collect.pub` (the public key only, never the private one) to Johan or David in the team chat; either of them adds it to `~/.ssh/authorized_keys` of the account on the Training machine. Wait for their confirmation.
   - Add to `~/.ssh/config`, with the values from the team zip in place of the `<…>`:
     ```
     Host rwr-gateway
         HostName <gateway host>
         User <gateway user>
         IdentityFile ~/.ssh/<gateway key file>
         IdentitiesOnly yes
     Host rwr-instance
         HostName <instance host>
         User <user of that account on the instance>
         ProxyJump rwr-gateway
         IdentityFile ~/.ssh/rwr_collect
         IdentitiesOnly yes
     ```
   - Check: `ssh -o BatchMode=yes rwr-instance true` returns at once without any prompt, and `ssh rwr-instance '~/.local/rsync/rsync --version'` prints rsync 3.2.7. Also `rsync --version` on this PC is 3.2.3 or newer.
4. In `ape_ui/static/local/station.json` (git-ignored; keep `robot_ip`), add `"archive_host": "rwr-instance"`, `"archive_remote_rsync": "~/.local/rsync/rsync"` and, **for now**, `"archive_require_hf_backup": false`. Leave `archive_data_root` and `archive_transfer_processed` unset (defaults: `~/rwr_task2/data`, true). Restart APE after editing the file.
   - `archive_require_hf_backup: false` is temporary: the Hugging Face backup of the archive is not set up yet, and without it no *Clear* in the checklist could pass. While it is false, *Clear transferred demos* deletes this PC's copies as soon as they match on the Training machine, so a cleared demo then exists **only on the Training machine** (archive + processed), plus the processed copies already on Hugging Face (sessions s00–s03 only; nothing of new sessions). Remove the key again (default true) as soon as the backup runs there.
   - Check ffmpeg: `ffmpeg -hide_banner -encoders | grep libx264` (else `pixi run pip install imageio-ffmpeg`).
5. Check whether the raw files of s01–s03 still exist, in the data folder APE uses (APE takes `$RWR_WS/data`, default `~/rwr_humble_ws/data`; read it from the running APE rather than assuming it):
   ```bash
   DATA=$(curl -s localhost:8765/api/datasets | python3 -c "import json, sys, pathlib; d = json.load(sys.stdin)['datasets']; print(pathlib.Path(d[0]['processed_dir']).parent.parent if d else '')")
   echo "APE data root: $DATA"    # empty: APE is not running or lists no session; stop and report (blocked)
   cd "$DATA" && for s in raw/s01_* raw/s02_* raw/s03_*; do echo "$s: $(ls $s/*.h5 2>/dev/null | wc -l) h5, $(du -sh $s 2>/dev/null | cut -f1)"; done
   df -h "$DATA"
   ```
   Expected counts if complete: s01 20, s02 46, s03 5. Report the data root in `ROBOT_PC_STATE.md` if it is not `~/rwr_humble_ws/data`.
6. If any of them exist: archive them before anything else. In APE choose each of these sessions and press *Process and check new demos* (one at a time). This only reads the raw files and copies to the Training machine; it deletes nothing. Expected: every demo becomes *verified* (not *+ HF* yet). Exception: `s03 … 20261121_215255` may stay *not verified*: its files were cut separately on the Training machine and here (instruction 2026-11-23-01), so the two copies can differ byte for byte. That is correct, and it must not be cleared. If this PC's processed files of a session were deleted earlier, the job re-creates them, and they may differ byte for byte from the copies already on the Training machine. Those demos then show *not verified* ("processed … different"): report the counts and change nothing on either machine. Do **not** press *Clear transferred demos* for s01–s03. The `*.h5.uncut` originals left by instruction 2026-11-23-01 are not archived and not copied; leave them where they are.
7. Then follow `ape_ui/TEST_VIDEO_ARCHIVE.md` from section 0 (skip its first item, `git switch video-archive`: stay on the branch from step 2), using a new short test session with 2–3 demos, with two changes because `archive_require_hf_backup` is false: skip section 3 (Hugging Face requirement), and in section 4 start from all demos *verified* (not *verified + HF*). Get the person's OK before each step that deletes something (the *Clear* steps in section 4, and the remote byte change there, which Johan or David makes on the Training machine).
Report back in `ROBOT_PC_STATE.md` **on `main`**: this checkout is not on `main` (it stays on the branch from step 2), so edit and commit the report in a worktree of `main` (`git worktree add ~/rwr_system_main main`, then the usual commit of only that file and `git pull --rebase && git push` there):
- the branch and commit from step 2, and the two lines of its archive-code check;
- ssh: working yes/no, rsync versions on both sides (no host names, users or keys);
- the APE data root from step 5;
- s01–s03: number of raw h5 files and GB per session, and for each session the counts *archived / verified / not verified* with the reasons shown in APE;
- archive size vs raw size, and encoding time per demo;
- each checklist item: passed / failed (with the log line);
- anything that was blocked, and why.

## 2026-11-26-01: Evaluated runs: new folder layout, archive video settings, upload button
Why: evaluated policy runs are now kept in `data/eval/<policy run>/<checkpoint>/<run id>/` (e.g. `Ampere_20261121-180315_s01_v1_act/best_model/20261125-114115/`), their videos are encoded with the demo archive's settings (H.264 4:4:4, CRF 12, still 10 frames per second), and card 5 has a button *Upload to the Training machine*. The upload also sends `policy_log.h5` per run: the inference node's log of that run without the images (published actions, predicted chunks, temporal-ensembling buffer, about 1–5 MB per minute instead of about 1.4 GB). Code: commit `5a18b19`, merged into `main` as `996f076`; tests ran on the Training machine only. The Training machine's copy of the 20 runs of 2026-11-25 has already been moved into the new layout (checksums of all 60 videos and signals files unchanged).
Steps:
1. Ask the person at the station before step 3: it restarts APE, and a restart stops every launch APE started. No policy may be running, and no demo may be being recorded or saved.
2. In the `rwr_system` checkout APE runs from (on `policy-eval`): `git status --short`. If it lists changes to tracked files under `ape_ui/`, `src/logger/`, `experiments/` or `CLAUDE.md`, stop and report them (blocked); do not commit, stash or discard them. Otherwise: `git fetch origin && git merge --ff-only origin/main`. That checkout then has the same code as `main`. If the fast-forward is refused, stop and report the message (blocked).
3. Restart APE (`ape_ui/run.sh`). In its terminal output, expect "Evaluated runs: 20 moved into the folders of their policy and checkpoint." (or the number of runs that were in `data/eval/`), with no line listing a problem below it. Then check: `find ~/rwr_humble_ws/data/eval -maxdepth 1` shows only policy folders and `trials.csv`, and card 5 lists all runs as before, with an *Uploaded* column and the button *Upload to the Training machine* above the list.
4. Open one old run in card 5 and press *Watch the videos*: both videos must play. Then press *Upload to the Training machine*. The log in card 3 should:
   - send no video or `signals.h5` of the 20 runs again (they are on the Training machine with the same content);
   - write a `policy_log.h5` for every run whose inference log is still in `~/rwr_humble_ws/inference_logs/`;
   - end with "Verified on the Training machine: N of N run(s)".
   The *Uploaded* column then shows *yes* for every run. Do not delete anything in `inference_logs/`; whether to delete the logs is the person's decision.
5. Only if the person wants a live test now, with two people present and one at the stop button (this instruction does not authorise motion): one short live policy run (about 1 minute) with any checkpoint. During it, note the CPU load of the two `ffmpeg` processes (`top`, sorted by CPU) and whether *Prediction time* in card 4 changes compared with earlier runs. After it, in that run's `trial.json`: `video_frames_lost` (should be 0) and `video.oakd_front_view.video_args` (should contain `yuv444p`). Watch its videos in card 5, then upload it. If frames were lost or the policy paused more than before, restart APE with `APE_EVAL_VIDEO_PRESET=veryfast` in the environment (about a third of the CPU time, about 1 dB lower quality) and repeat the run once.
Report back in `ROBOT_PC_STATE.md`:
- the commit of the code checkout after step 2;
- the move line from APE's output (step 3), and any problem lines;
- whether both videos played (step 4);
- from the upload log (step 4): runs verified, `policy_log.h5` files written, runs without an inference log, and the size of one `policy_log.h5` next to the size of its inference log;
- if step 5 was done: the ffmpeg CPU load, prediction time before and during, `video_frames_lost`, MB per minute of the two videos, and whether `veryfast` was needed;
- anything blocked, and why.
