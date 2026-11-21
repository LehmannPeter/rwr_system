# Robot PC state

Written only by the Claude Code session on the Robot PC. Replace the placeholders the first time; afterwards keep every section current. Rules: `handoff/README.md`.

Last updated: 2026-11-21

## Setup

| Item | Value |
|---|---|
| OS, GPU, free disk | Ubuntu 24.04.3 LTS (kernel 7.0.0-43); NVIDIA GeForce RTX 5060 Laptop GPU, 8 GB; 442 GB free of 937 GB |
| ROS workspace path | `~/rwr_humble_ws` (pixi, ROS 2 Humble, Python 3.12). `src/rwr_system` is a symlink to `~/Projects/rwr_system`. The teleop and recordings are normally started from the `ape_ui` web app (`~/Projects/ape_ui`), which runs ROS in domain 7 with CycloneDDS |
| `rwr_system` commit, branch, uncommitted changes | `22fb804`, `main`, up to date with `origin`. Uncommitted: the two retargeter configs (see below) |
| `rwr_il` commit, importable in the ROS environment (yes/no) | `7cfc042` (`main`); yes, `from srl_il.export.policy import get_policy_from_ckpt` works in the pixi env (warns that pytorch3d is not installed) |
| Folder where recordings are stored | Since 2026-11-21: `~/rwr_humble_ws/data/raw/<session>/`, processed in `data/processed/<session>_10hz/`, per-episode labels in `data/manifest.csv`, dataset versions in `data/datasets/<version>/` (set by `ape_ui`; tool: `rwr_il/scripts/rwr_data.py`, described in `rwr_il/CLAUDE.md`). Session names: `s<nn>_<date>_<operator>` |
| Camera IDs match `cube_sorting_task.yaml` (yes/no, checked when) | yes: both MXIDs in the config (front_view, wrist_view) were the devices the OAK-D node opened on 2026-11-21 |
| Franka end-effector parameters (values, screenshot file, date) | not recorded yet |
| Retargeter config (commit or local changes) | Local changes since 2026-11-21, in use and kept: MANO adjustments refitted offline to a teleop test recording (`fit2`), and `virtual_coupling` switched off in `retargeter_cfgs_orcahand_v2.yaml`. Tested on the hand 2026-11-21: finger bending good (tabletop and hook now possible); pinches open slightly more than the glove; finger spread smaller than the human hand. Later on 2026-11-21 replaced by `fit4` (same, with finger abduction refitted on 13 hand-posed pairs: fingers close together better, full spread slightly worse), being tried on the hand; `fit2` kept as `retargeter_fits/mano_adjustment_live_fit2_backup.yaml`. Previous version is commit `c2202cb`, plus a backup in `~/rwr_humble_ws/retargeter_backups/2026-11-21_before_tuning/`. Fit scripts, 13 hand-posed calibration pairs and results: `~/rwr_humble_ws/retargeter_fits/` |

## Recording sessions

| Dataset name / folder | Episodes per colour | Resampled | Copied off the Robot PC | Notes |
|---|---|---|---|---|
| `data/raw/s00_2026-11-19_setup` (until 2026-11-21: `datasets/cube_sorting_red`) | red: 6 | yes, `data/processed/s00_2026-11-19_setup_10hz` (6 files, 3.9 GB) | no | Recorded 2026-11-18 and 2026-11-19, before the retargeter changes of 2026-11-21. Raw folder is 20 GB. Marked `review` in `manifest.csv`, so no dataset version includes them |

## Policies tried on the robot

| Run / checkpoint | Preview OK | Inference time per step | Result | Notes |
|---|---|---|---|---|

## Open problems

- Retargeter changed on 2026-11-21 (see Setup). Demonstrations recorded before and after the change map hand poses differently. Baseline measurement with the old config: `~/rwr_humble_ws/data/basemine4.h5`, made with the new script `src/retargeter/retargeter_teleop_test.py` (uncommitted). With the old config, tabletop and hook hand poses were impossible and pinches closed 12–15 mm tighter than the glove target.
- `retargeter_collect_calibration_data.py` switched torque off while the retargeter was still sending positions, so the servos switched back on and the hand stayed stiff. Fixed locally (uncommitted): it now waits until no hand command has arrived for 0.5 s. The installed copy is not executable, so `ros2 run retargeter retargeter_collect_calibration_data.py` fails; run it with `python` (`~/collect.sh` does this for the `ape_ui` ROS domain).
- Motor 17 (`thumb_cmc`) reported an angle sensor error once on 2026-11-21. The ORCA thumb was mechanically hard to place in some hand-posed poses.
- Git on the Robot PC has no author name or email set, so commits made here fail.
- `retargeter_calibration.py` runs the retargeter for one call (2 optimiser steps) per sample, so the joint angles it fits against are far from converged (18° mean, up to 85°, measured on stored poses). Not changed.
- The teleop retargeter publishes at about 31 Hz although its timer is set to 200 Hz.

- 2026-11-21: starting the Franka controller ended in a `power_limit_violation` reflex several times. The NUC launch starts `panda_mpc_controller`, which pulls the elbow posture toward the Panda ready pose at full strength as soon as it activates. An offline replay of the public `multipanda_ros2` source at the arm's joint angles shows joints 1 and 3 snapping about 0.1 rad; that this snap trips the power limit is inferred, not reproduced. Workaround: hand-guide the arm near the ready pose before starting the controller.
- 2026-11-21: arm teleop moved sideways opposite to the glove. The directions come from the hard-coded coil orientation `X_W_bCP` in `experiments/run_teleop_rokoko.py`, not from the glove calibration. A live sample showed the operator's forward mapping to robot -x and right to robot +y, about 180 degrees off. Uncommitted: optional station file `~/.ros/teleop_coil_adjust_right.json` (`{"yaw_deg": 180}`), read by the teleop node at start and at every resume; without the file nothing changes. APE has a switch for it (*Reverse arm directions*, off by default). No offline test covers it. File not created yet, correction not tested on the robot. Demonstrations recorded with and without it differ only in how the operator steers, not in the recorded robot poses.
- Uncommitted on 2026-11-21: `--output_folder` option in `src/logger/syncronize_data.py`, and `rwr_il/scripts/rwr_data.py` (manifest, dataset versions, Hugging Face push and pull). The Training machine needs the `rwr_il` change pushed before it can run `pull`. Hugging Face login and repo are not set up yet; `push` and `pull` were only tested against a local stand-in.

## Handled instructions

| ID | Status (done / blocked) | Date | Result |
|---|---|---|---|
| 2026-11-21-01 | done | 2026-11-21 | `.machine` = `robot_pc` written in `rwr_system` and `rwr_il`; Setup table filled |
