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
