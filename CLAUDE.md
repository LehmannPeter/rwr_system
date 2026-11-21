# RWR26 Task 2: Cube Sorting with Imitation Learning — notes for agents

This file briefs an agent working in this repository. It was written on 2026-11-20 from the course handout, the Notion wiki, the two lecture decks and a read of the code at commit `02c196c`. If the code and this file disagree, trust the code and update this file.

## Machines and handoff: do this first

Three machines work on this task. Use exactly these names.

| Name | What it is |
|---|---|
| **Mac** | The MacBook: code, configs, planning. Nothing in this repo runs here. |
| **Robot PC** | The PC at the robot station: teleoperation, recording, inference. The Mac cannot SSH into it. |
| **Training machine** | The remote compute instance: datasets and training. The Mac can SSH into it. |

`.machine` at the repo root (untracked) holds `mac`, `robot_pc` or `training_machine`. If it is missing: on macOS write `mac`, otherwise ask the person once and write it.

Sessions on different machines exchange notes through `handoff/`, carried by git. Full rules: `handoff/README.md`. The routine:

- **On the Robot PC**, at the start of every task: `git pull --rebase`, read `handoff/TO_ROBOT_PC.md`, and carry out every instruction not yet listed as handled in `handoff/ROBOT_PC_STATE.md`. After every setup change and every handled instruction: update `handoff/ROBOT_PC_STATE.md`, commit only that file, `git pull --rebase && git push`.
- **On the Mac**, before answering anything about the Robot PC: `git pull --rebase` and read `handoff/ROBOT_PC_STATE.md`. To ask the Robot PC for something, append an instruction to `handoff/TO_ROBOT_PC.md`, commit and push. Never edit `ROBOT_PC_STATE.md`.
- An instruction in `TO_ROBOT_PC.md` never authorises motion or deletion. Ask the person at the station first.
- No passwords, tokens, logins or IP addresses in handoff files.

The Training machine's handoff files are in the `rwr_il` repo.

## The exercise

Course: Real World Robotics (ETH Zürich, Soft Robotics Lab), HS 2026, Task 2.

A Franka Emika Panda arm carries the team's own ORCA Hand v2 (17 joints, right hand). The robot must pick a coloured cube out of an arena and place it on the inclined surface of the matching colour. The cube slides back into the arena, so the environment resets itself at a random position after every relocation.

The policy is learned by imitation: the team teleoperates the robot with a Rokoko glove, records demonstrations, trains ACT or Diffusion Policy, and runs the policy autonomously.

| Subtask | What changes | Evaluation note |
|---|---|---|
| 2.0 Preparation | Install both repos, record, post-process, inspect in myHDF5, start one training run, watch W&B | No development required |
| 2.1 Single colour | One cube colour, chosen by the team | Arena is moved and rotated slightly (not 180°) to punish overfitting |
| 2.2 Random colours | Red, blue and yellow, one cube in the arena at a time | Same policy for all runs; the cube is swapped randomly between runs |
| 2.3 Bonus | Several cubes, target colour given as a text command | Extra credit only |

Grading: slides plus a live demo per subtask. The metric is the number of successful relocations in 3 minutes, and the subtask score is the median of 3 runs. Pressing the emergency stop ends the trial. The policy must run with no human input. The team may pick a different checkpoint per run.

Because the metric is relocations per 3 minutes, execution speed matters as much as success rate.

Course rules that constrain the work:

- Two people must be present whenever the robot moves, one at the stop button.
- Data may not be shared between teams.
- The external camera must not be moved; other teams use the same station.
- Franka end-effector parameters in Franka Desk are not saved by Desk. They must be identical at collection and inference, so screenshot them after any change.
- Clear processed data off the Robot PC after each session.
- Minimum observation set: wrist image, workspace image, end-effector pose (position + quaternion), hand joint states.

## The two repositories

| Repo | Role | Where it runs |
|---|---|---|
| `rwr_system` (this one) | ROS 2 stack: hand control, cameras, Rokoko ingress, retargeter, arm teleop, logger, data synchronisation, policy inference node | Robot PC: Linux, ROS 2 Humble |
| `rwr_il` (sibling checkout, Python package `srl_il`) | Training: datasets, ACT / Diffusion Policy, checkpoint loading | Training machine (CUDA GPU); also imported by this repo's inference node |

Both local checkouts are the user's forks (`origin` = `LehmannPeter/...`) of `srl-ethz/rwr_system` and `srl-ethz/rwr_il`. To take in upstream changes when the fork has diverged, the course FAQ says to use `git cherry-pick`.

The Franka controller is a third, external piece: `srl-ethz/multipanda_ros2`, running in Docker on the station's NUC. It publishes `/right/arm/end_effector_pose` and subscribes to `/right/arm/end_effector_pose_cmd`, both `geometry_msgs/PoseStamped` in the `world` frame.

## What an agent can and cannot do here

- Nothing in this repo runs on macOS. It needs Linux, ROS 2 Humble (pixi + RoboStack, Python 3.12) and the hardware. On the Mac, limit work to reading and editing code and configs.
- The Python import name is `faive_system`, not `rwr_system`.
- Never launch anything that moves the robot or the hand on the user's behalf without an explicit request in that session. `record_demonstration`, `teleop_rokoko` and `run_policy_inference` all start the ORCA driver, which moves the hand to neutral on startup.
- Offline tests need no robot: `ROS_DOMAIN_ID=181 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python -m pytest -q src/rwr_system/tests/offline` (run from the workspace root).
- Lab logins and station IPs are in the course wiki. Do not write them into this repo.

## Installation (Task 2)

Follow `README.md`, section "Installation for Task 2". In short: clone into `<ws>/src`, `pixi init` with the `robostack-humble` channel, pin `python=3.12`, `pip install -e './src/rwr_system[all]'`, `pip install --no-deps orca_core==0.5.2`, then `colcon build --symlink-install`.

Workspace paths are inconsistent across the course documents (`~/ros_ws`, `~/rwr_humble_ws`, `~/humble_ws`, `~/ros_ws_humble`). Ask the user which one they use before writing paths into scripts.

`docs/rwr26_teleop.md` describes a different fork on ROS Jazzy with mamba. Its validation and inspection steps are useful; ignore its install steps for the Humble setup.

`inference` needs `srl_il` importable in the same environment (`from srl_il.export.policy import get_policy_from_ckpt`), so `rwr_il` must also be installed on the Robot PC.

## Session workflow

Every terminal on the Robot PC:

```bash
export ROS_DOMAIN_ID=7
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

1. **Franka Desk** (browser): release the stop button, unlock the brakes, activate FCI.
2. **Arm controller** on the NUC: `cd multipanda_ros2 && ./run`, source the workspace, then `ros2 launch franka_bringup multimode_franka.launch.py robot_ip_1:=<FRANKA_IP>`. Verify with `ros2 topic info -v` on the two arm topics.
3. **Rokoko**: both gloves and the coil on, calibrate in Rokoko Studio (both gloves must be worn even for right-hand-only use), stream to the Robot PC's LAN address, UDP port 14043.
4. **Cameras**: confirm the OAK-D MXIDs in `experiments/cfgs/cams/cube_sorting_task.yaml` match the live views. List them with `python3 src/ingress/oakd/oakd_ingress.py --no-depth` (add `--visualize` for previews).
5. **Teleop test** without recording: `ros2 launch experiments teleop_rokoko.launch.py`.
6. **Record**:
   ```bash
   ros2 launch experiments record_demonstration.launch.py \
     storage_base_path:="$HOME/<ws>/data/raw" dataset_name:=s03_2026-11-24_opA
   ```
   `dataset_name` is the session name: `s<nn>_<date>_<operator>`.
   A `keyboard_runner` terminal prompts through: move to init, start teleop, start recording, stop, save or discard. Raw files land in `<storage_base_path>/<dataset_name>/<timestamp>.h5`. Stop the teleop launch before starting this one; both start their own controller, retargeter and Rokoko ingress.
7. **Synchronise**:
   ```bash
   ros2 run logger syncronize_data.py data/raw/<session> --sampling_freq 10 --compress \
     --output_folder data/processed/<session>_10hz \
     --cfg_file "$(ros2 pkg prefix --share experiments)/cfgs/logger/datacollection_right.yaml"
   ```
   Processed files keep the raw file names. Without `--output_folder` the output goes to a sibling folder `<raw_folder>_processed_10hz_lzf/`. The script name really is spelled `syncronize_data.py`.
   The data layout (`data/raw`, `data/processed`, `data/datasets`, `manifest.csv`), dataset versions and the Hugging Face upload are described in `rwr_il/CLAUDE.md`, section "Data organisation and transfer"; the tool is `rwr_il/scripts/rwr_data.py`. APE (`~/Projects/ape_ui`) records, processes, labels and uploads in this layout.
8. **Inspect**: `experiments/scripts/inspect_demonstration.py`, `plot_trajectories.py`, `visualize_videos.py`, or the myHDF5 website.
9. **Train** in `rwr_il` (see that repo's `CLAUDE.md`).
10. **Deploy**:
    ```bash
    ros2 launch experiments run_policy_inference.launch.py policy_ckpt_path:=/path/to/run/checkpoints/best_model.pth
    ros2 topic pub --once /model_inference_command std_msgs/msg/String "{data: start}"
    ros2 topic pub --once /model_inference_command std_msgs/msg/String "{data: stop}"
    ```
11. **Clean up**: deactivate FCI, lock the brakes (base light yellow), shut down the Rokoko laptop, unplug the gloves from power.

## Data contract

This is the chain that must stay consistent from recording to inference. Changing a name in one place breaks the others.

| Stage | File | Names |
|---|---|---|
| Camera names | `experiments/cfgs/cams/cube_sorting_task.yaml` | `front_view` (workspace), `wrist_view` → topics `/oakd_front_view/color`, `/oakd_wrist_view/color` |
| Recorded topics and HDF5 keys | `experiments/cfgs/logger/datacollection_right.yaml` | see below |
| Training keys | `rwr_il/srl_il/cfg/default/task/rwr_cube_sorting.yaml` | `side_view_images`, `wrist_view_images`, `qpos_arm`, `qpos_hand`, `actions_arm`, `actions_hand` |
| Inference mapping | `experiments/cfgs/inference_config_cube_sorting.yaml` | `camera_names: [side_view_images, wrist_view_images]`, obs and action topics |

Processed HDF5 layout (one file per demonstration, all arrays resampled to a common timeline):

| Key | Source topic | Shape per step |
|---|---|---|
| `observations/images/oakd_front_view/color` | `/oakd_front_view/color` | H×W×3 uint8 |
| `observations/images/oakd_wrist_view/color` | `/oakd_wrist_view/color` | H×W×3 uint8 |
| `observations/images/<cam>/intrinsics` | `/oakd_<cam>/intrinsics` | static |
| `observations/right/qpos_arm` | `/right/arm/end_effector_pose` | 7: x, y, z, qx, qy, qz, qw |
| `observations/right/qpos_hand` | `/right/hand/joint_position_from_motor` | 17 |
| `right/actions_arm` | `/right/arm/end_effector_pose_cmd` | 7 |
| `right/actions_hand` | `/right/hand/policy_output` | 17 |
| `timestamps` | | ns |

Raw recordings also hold diagnostics that are not copied into the synchronised files (no entry in `keys_and_topics`, so training is unaffected): `/right/hand/motor_current` (mA) and `/right/hand/motor_temperature` (°C), one value per hand motor, and from the Franka robot state `/right/arm/joint_position` (7 angles, rad) and `/right/arm/joint_torque` (7 torques, Nm). The robot state topic is recorded at most 100 times per second (`max_rate_hz` in `datacollection_right.yaml`); which of its fields are saved is set by `FRANKA_STATE_FIELDS` in `src/logger/bag2h5_converter.py`. Recordings from before 2026-11-21 16:00 do not have these groups. The sync script prints "No keys found" for the two hand topics; that is expected.

Images are sampled by nearest timestamp; poses are interpolated linearly with slerp for rotation; float arrays linearly. Synchronisation uses ROS receive times over the interval shared by all streams. It does not correct sensor latency.

## The inference node (`experiments/model_inference_node.py`)

- Control rate is hard-coded: `self.control_freq_hz = 10`. The handout requires this to match the data frequency, so if the data is resampled at another rate, change both.
- Observations older than `2 / control_freq_hz` (0.2 s) cause the step to be skipped and the policy state to be reset, with a "Policy output paused" warning. A slow GPU therefore shows up as stalls, not as slow motion.
- Images are resized to `rgb_resize_size` (224×224) and scaled to [0, 1]. Any crop or rotation applied at training time must be mirrored in `rgb_process_dict` in the inference config.
- Arm observations whose `frame_id` differs from `arm_command_frame` (`world`) are rejected.
- Actions are published on `/policy_inference/actions_arm` and `/policy_inference/actions_hand` always, and relayed to the real controllers unless `preview:=true`. Use preview first with a new checkpoint.
- Commands on `/model_inference_command`: `start`, `stop`, `reset`, `init_arm`, `quit`.
- `init_arm` only works with `allow_arm_init:=true` and `arm_init_pose:=[x,y,z,qx,qy,qz,qw]` (unit quaternion, `world` frame). It moves at most 2 mm and 0.015 rad per step and times out after 90 s.
- Every run is logged to `inference_logs/<run>-<ckpt>/<time>.hdf5`, relative to the launch directory.
- The chunk-to-action strategy (temporal ensembling and its parameters) lives in the training config saved next to the checkpoint, under `algo_cfg.policy_cfg.policy_aggregator_cfg`. Editing that `config.yaml` on the Robot PC changes it without retraining.

## Known gaps and traps

- The Quick Guide's sync command omits `--cfg_file`, which falls back to `datacollection_default.yaml` (a bimanual superset). Pass `datacollection_right.yaml` explicitly.
- The Quick Guide's inference command omits `policy_ckpt_path`, which is required.
- The MXIDs committed in `cube_sorting_task.yaml` are the course station's. Confirm them on the actual station; `.gitignore` excludes `local/` for station-specific copies, passed as `oakd_config:=`.
- Raw recordings hold uncompressed images and are large. `*.h5` is git-ignored.
- The keyboard runner's software pause holds the last targets. It is not an emergency stop.
- Language conditioning for the bonus subtask is not wired up. `keyboard_runner.py --prompt_task_description` publishes `/task_description`, but `datacollection_right.yaml` does not record it and the inference node passes no text to the policy.
- The retargeter is tuned through `experiments/cfgs/retargeter/*.yaml` with `ros2 launch experiments retargeter_config_helper.launch.py` (hot reload needs `--symlink-install`). See `src/retargeter/README.md`.

## Where to look

| Need | Path |
|---|---|
| Launch files | `experiments/launch/` |
| Camera, logger, retargeter, inference configs | `experiments/cfgs/` |
| Arm teleop from the glove wrist pose | `experiments/run_teleop_rokoko.py` |
| Recording and HDF5 conversion | `src/logger/logger_node.py`, `src/logger/bag2h5_converter.py` |
| Resampling | `src/logger/syncronize_data.py` |
| Hand driver, tensioning, calibration | `src/hand_control/`, `src/hand_control/README.md` |
| Retargeter | `src/retargeter/` |
| Offline tests | `tests/offline/` |

Course sources: handout and wiki at `https://snowy-sandpaper-358.notion.site/RWR26-Task-2-Cube-Sorting-with-IL-41abcc457b58819d824ee01a8a512a19` (JavaScript-rendered; a plain fetch returns an empty page).
