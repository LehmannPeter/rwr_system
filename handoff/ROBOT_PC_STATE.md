# Robot PC state

Written only by the Claude Code session on the Robot PC. Replace the placeholders the first time; afterwards keep every section current. Rules: `handoff/README.md`.

Last updated: 2026-11-25

## Setup

| Item | Value |
|---|---|
| OS, GPU, free disk | Ubuntu 24.04.3 LTS (kernel 7.0.0-43); NVIDIA GeForce RTX 5060 Laptop GPU, 8 GB; 442 GB free of 937 GB |
| ROS workspace path | `~/rwr_humble_ws` (pixi, ROS 2 Humble, Python 3.12). `src/rwr_system` is a symlink to `~/Projects/rwr_system`. The teleop and recordings are normally started from the `ape_ui` web app (since 2026-11-21 in this repo: `~/Projects/rwr_system/ape_ui`; the robot's address is in the git-ignored `ape_ui/static/local/station.json`), which runs ROS in domain 7 with CycloneDDS |
| `rwr_system` commit, branch, uncommitted changes | `9218688`, `main`, pushed; nothing uncommitted |
| `rwr_il` commit, importable in the ROS environment (yes/no) | `7cfc042` (`main`); yes, `from srl_il.export.policy import get_policy_from_ckpt` works in the pixi env (warns that pytorch3d is not installed) |
| Folder where recordings are stored | Since 2026-11-21: `~/rwr_humble_ws/data/raw/<session>/`, processed in `data/processed/<session>_10hz/`, per-episode labels in `data/manifest.csv`, dataset versions in `data/datasets/<version>/` (set by `ape_ui`; tool: `rwr_il/scripts/rwr_data.py`, described in `rwr_il/CLAUDE.md`). Session names: `s<nn>_<date>_<operator>` |
| Camera IDs match `cube_sorting_task.yaml` (yes/no, checked when) | yes: both MXIDs in the config (front_view, wrist_view) were the devices the OAK-D node opened on 2026-11-21 |
| Franka end-effector parameters (values, screenshot file, date) | not recorded yet |
| Retargeter config (commit or local changes) | Committed in `9218688` (2026-11-21), in use: MANO adjustments `fit4` (refitted offline on a teleop-test recording, then finger abduction refitted on 13 hand-posed pairs), finger mcp/pip coupling off, thumb coupling `thumb2` on (`thumb_mcp` ≈ `thumb_dip`, weight 0.2). Tried on the hand 2026-11-21: `fit2` (finger bending good, tabletop and hook possible; pinches open slightly more than the glove; finger spread too small) and `fit4` (abduction closer). Saved alternatives in `~/rwr_humble_ws/retargeter_fits/`: `fit2` (`mano_adjustment_live_fit2_backup.yaml`), `fit5/` (original MANO values with mcp/pip and thumb2 coupling). Values before 2026-11-21: commit `c2202cb`, plus a backup in `~/rwr_humble_ws/retargeter_backups/2026-11-21_before_tuning/`. Fit scripts, pairs and results are in `retargeter_fits/` too |

## Recording sessions

| Dataset name / folder | Episodes per colour | Resampled | Copied off the Robot PC | Notes |
|---|---|---|---|---|
| `data/raw/s00_2026-11-19_setup` (until 2026-11-21: `datasets/cube_sorting_red`) | red: 6 | yes, `data/processed/s00_2026-11-19_setup_10hz` (6 files, 3.9 GB) | no | Recorded 2026-11-18 and 2026-11-19, before the retargeter changes of 2026-11-21. Raw folder is 20 GB. Marked `review` in `manifest.csv`, so no dataset version includes them |

## Policies tried on the robot

| Run / checkpoint | Preview OK | Inference time per step | Result | Notes |
|---|---|---|---|---|

## Open problems

- Retargeter changed on 2026-11-21 (see Setup). Demonstrations recorded before and after the change map hand poses differently. Baseline measurement with the old config: `~/rwr_humble_ws/data/basemine4.h5`, made with the new script `src/retargeter/retargeter_teleop_test.py` (uncommitted). With the old config, tabletop and hook hand poses were impossible and pinches closed 12–15 mm tighter than the glove target.
- `retargeter_collect_calibration_data.py` switched torque off while the retargeter was still sending positions, so the servos switched back on and the hand stayed stiff. Fixed (committed in `0a07e73`): it now waits until no hand command has arrived for 0.5 s. The installed copy is not executable, so `ros2 run retargeter retargeter_collect_calibration_data.py` fails; run it with `python` (`~/collect.sh` does this for the `ape_ui` ROS domain).
- Motor 17 (`thumb_cmc`) reported an angle sensor error once on 2026-11-21. The ORCA thumb was mechanically hard to place in some hand-posed poses.
- `retargeter_calibration.py` runs the retargeter for one call (2 optimiser steps) per sample, so the joint angles it fits against are far from converged (18° mean, up to 85°, measured on stored poses). Not changed.
- The teleop retargeter publishes at about 31 Hz although its timer is set to 200 Hz.

- 2026-11-21: starting the Franka controller ended in a `power_limit_violation` reflex several times. The NUC launch starts `panda_mpc_controller`, which pulls the elbow posture toward the Panda ready pose at full strength as soon as it activates. An offline replay of the public `multipanda_ros2` source at the arm's joint angles shows joints 1 and 3 snapping about 0.1 rad; that this snap trips the power limit is inferred, not reproduced. Workaround: hand-guide the arm near the ready pose before starting the controller.
- 2026-11-21: arm teleop moved sideways opposite to the glove. The directions come from the hard-coded coil orientation `X_W_bCP` in `experiments/run_teleop_rokoko.py`, not from the glove calibration. A live sample showed the operator's forward mapping to robot -x and right to robot +y, about 180 degrees off. Uncommitted: optional station file `~/.ros/teleop_coil_adjust_right.json` (`{"yaw_deg": 180}`), read by the teleop node at start and at every resume; without the file nothing changes. APE has a switch for it (*Reverse arm directions*, off by default). No offline test covers it. File not created yet, correction not tested on the robot. Demonstrations recorded with and without it differ only in how the operator steers, not in the recorded robot poses.
- Uncommitted on 2026-11-21: `--output_folder` option in `src/logger/syncronize_data.py`, and `rwr_il/scripts/rwr_data.py` (manifest, dataset versions, Hugging Face push and pull). The Training machine needs the `rwr_il` change pushed before it can run `pull`. Hugging Face login and repo are not set up yet; `push` and `pull` were only tested against a local stand-in.

## Video archive (instruction 2026-11-25-01, 2026-11-25)

- Checkout: `rwr_system` on `policy-eval`, commit `2699ca2` plus that day's uncommitted, finished policy-eval changes. With the person's OK they were committed as `7ab9261` (arm mover for *Move to center*, -0.05 s observation-age allowance, 6 Hz flow thresholds) before starting. Archive-code check: `ape_ui/archive_job.py: same as video-archive`, `src/logger/encode_archive.py: same as video-archive`. `rwr_il` not touched.
- ssh: working, `ssh -o BatchMode=yes rwr-instance true` returns without a prompt. rsync 3.2.7 on this PC and 3.2.7 (`~/.local/rsync/rsync`) on the Training machine. Deviation: the existing config logs in with the personal default key, not a dedicated `~/.ssh/rwr_collect` key; no new key was made.
- `station.json`: `archive_host`, `archive_remote_rsync` and `archive_require_hf_backup: false` (temporary, see below) added; `robot_ip` kept. ffmpeg: none on PATH; the one from `imageio-ffmpeg` in the pixi env is used and has libx264.
- APE data root: `~/rwr_humble_ws/data` (the default). 120 GB free before the archive.

| Session | Raw h5 / size | Archived | Verified | Not verified | Archive size |
|---|---|---|---|---|---|
| s01_2026-11-21_Clarence | 20 / 21.3 GB | 20 | 20 | 0 | 0.40 GB (1.9 %) |
| s02_2026-11-21_Clarence | 46 / 41.3 GB | 46 | 46 | 0 | 0.80 GB (1.9 %) |
| s03_2026-11-21_Clarence | 5 / 84.1 GB | 5 | 4 | 1 | 1.65 GB (2.0 %) |

- s03 `20261121_215255`: *not verified*, "processed … missing or different on the Training machine", as expected: instruction 2026-11-23-01 (the cut) was never carried out on this PC, so the local files are uncut and there are no `*.h5.uncut` files here. Must not be cleared. Nothing of s01–s03 was cleared or deleted.
- Archive is about 2 % of the raw size (not 6 %). Lowest frame PSNR 40.6 dB (limit 30). Encoding about 15 s per demo with 2 in parallel (s01: 20 demos in 2.5 min); copy about 4.6 MB/s. Whole job: s01 about 5 min, s02 9 min, s03 19 min.
- The `.meta.json` files and the raw/processed/archive files of s01–s03 are now on the Training machine (archive in `~/rwr_task2/data/archive/`).

Checklist `ape_ui/TEST_VIDEO_ARCHIVE.md`, test session `s90_2026-11-25_test`. With the person's OK it was made from copies of 3 s01 demos instead of new recordings (`20261121_173007`, `_173120`, `_174024`; `.meta.json` session changed, description "TEST COPY …"); manifest rows with `keep=discard` and a note "test copy … not for training".

| Item | Result |
|---|---|
| 0 Setup | passed (see above; branch switch skipped as instructed) |
| 1 Archive without transfer | skipped (optional; s01–s03 showed the encoding) |
| 2 Transfer and verification | passed: "Verified on the Training machine: 3 of 3 demo(s)"; *verified* in the column, *Clear* active; sha256 of `20261121_173007.oakd_front_view.mp4` on the Training machine equals its `.archive.json` (`20d45b59…`); `manifest_inbox/robot_pc.csv` equals the local manifest; `merge-manifest --dry-run` (rwr_il `training-levers`, contains `manifest-merge`) lists the 3 test demos as new rows and wrote nothing; third *Process*: "3 already archived, 0 to encode", rsync sent nothing, still 3 of 3 |
| 3 Hugging Face requirement | skipped (`archive_require_hf_backup: false`) |
| 4 Mismatch protection | passed. One byte appended on the Training machine (by this session over ssh, with the person's OK); *Clear*: "Verified … 2 of 3", tampered demo kept (*not verified*: "…oakd_front_view.mp4 missing or different on the Training machine"), the other two "removed 6 file(s)", 2.24 GB freed, *cleared here*; their `.meta.json` and manifest rows kept, dataset names `s90_red_0001`–`0003` unchanged. Remote file deleted, *Process* re-sent it (6.3 MB), *verified*; *Clear* then "20261121_173007: removed 6 file(s)", 1.07 GB freed |
| 5 Reconstruction | passed, on the Training machine with `rwr_il/.venv` python (no ROS; its `rwr_system` checkout has the same three logger scripts as `video-archive`): `reconstruct_raw.py` 3 files in 27 s, `syncronize_data.py` 9 s; against the processed file from the real raw data: same 9 keys, non-image arrays identical, images 43.4 / 42.9 dB; `inspect_demonstration.py` `ok: true`, no errors. Output in `/tmp` deleted afterwards |
| 6 Guards | *Clear* while a job runs: refused ("Another job is still running.", 409), button disabled in the page code. Recording, saving, unsaved demo and running policy: refused in the server code and disabled in the page code, not yet tried live. Network cut test: not done |

Left for the Training machine: `archive/s90_2026-11-25_test/` and `processed/s90_2026-11-25_test_10hz/` there are test duplicates of s01 data; delete them and do not merge the s90 manifest rows into training.

Cleared afterwards (2026-11-25 15:40, the person's decision, disk at 29 GB free): s01 (20 demos, 25.9 GB) and s02 (46 demos, 50.4 GB) with *Clear transferred demos*; it re-verified 20 of 20 and 46 of 46 on the Training machine right before deleting. Their raw, processed and archive files are now only on the Training machine (processed also on Hugging Face); `.meta.json` and manifest rows stay here. 96 GB free afterwards. s03 and s04 not cleared (s04 not processed yet: 6 demos, 87 GB).

`archive_require_hf_backup` is still false: a demo cleared now exists only on the Training machine (plus Hugging Face for processed s00–s03). Set it back to true once the archive backup to Hugging Face runs there.

Changed during this instruction, at the person's request (committed `e946c06` on `policy-eval`): *Process and check new demos* and *Upload to Hugging Face* may now run while a demo is recorded or saved. Jobs run at `nice 19`; ionice is not used because the NVMe has no I/O scheduler. The logger writes `<name>.h5.part` and renames it when complete. Uploads use wifi/VPN, the robot the wired link. *Clear* stays refused while recording. Live since the APE restart at about 14:30 (the logger part from the next station launch); not yet measured on a real recording.

## Handled instructions

| ID | Status (done / blocked) | Date | Result |
|---|---|---|---|
| 2026-11-21-01 | done | 2026-11-21 | `.machine` = `robot_pc` written in `rwr_system` and `rwr_il`; Setup table filled |
| 2026-11-25-01 | done (section 6 partly) | 2026-11-25 | s01–s03 archived and verified on the Training machine except s03 `20261121_215255` (expected); checklist sections 0, 2, 4, 5 passed, 1 and 3 skipped, 6 partly; see "Video archive" |
