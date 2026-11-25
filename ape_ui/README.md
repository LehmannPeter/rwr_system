# APE UI (RWR station dashboard)

A local web page for the RWR26 Task 2 station: start and stop the NUC controller, the
teleop launch and the recording launch, read their logs, see the robot status, and run
teleop and demo recording with buttons instead of the keyboard runner.

**The page is not a safety stop.** Keep a person at the black stop button.

## Start

```bash
~/Projects/rwr_system/ape_ui/run.sh
```

Then open <http://localhost:8765> in the browser on this laptop. Leave the terminal open.

On a new machine, set the robot's address once. It is station-specific and stays out of git
(`local/` is git-ignored); *Start controller* and the *Franka Desk* button both read it:

```bash
mkdir -p ~/Projects/rwr_system/ape_ui/static/local
echo '{"robot_ip": "<address from the course wiki>"}' > ~/Projects/rwr_system/ape_ui/static/local/station.json
```

## Use

1. **Franka controller (NUC):** press *Start controller*, type the NUC password into
   the box that appears and press Enter. The page enters the container, checks that no
   old controller is running, and launches the controller. The login stays open, so a
   restart after a reflex or stop-button press needs no password: *Restart controller*
   does stop and start in one click, and pauses teleop first if it is still running. It
   is refused while the black button is still pressed.
2. **Station launch:** press *Start launch*. It starts the gloves, retargeter, hand, arm
   teleop, both cameras and the logger. Nothing is recorded until you start a recording.
   *Hand only* starts just the gloves, the retargeter and the ORCA hand, to try the hand
   without the arm: no arm teleop, cameras or logger, and step 1 is not needed. Then use
   *Start hand teleop* in the Operate card.
3. **Operate card:** replaces the keyboard runner. Do not run the keyboard runner at the
   same time.
   - *Start teleop* (hold your hand still): teleop without recording. Then *Start
     recording* when ready, or *Pause teleop*.
   - *Start teleop + recording*: starts both at once.
   - *Stop recording and pause*, then *Save demo* or *Discard*. Wait for the save to finish.
   - *Demo labels* (top of the inputs column): *Cube colour* (red, blue, yellow, mixed),
     *Arena pose*, *Cube cell*, *Cube orientation* (c0, c1, c2). They stay as set until changed. Each saved demo gets a row in
     `data/manifest.csv` with them (the Save button shows the colour it will write).
   - The hand motors are only powered while teleop runs or the arm moves to a pose. They
     switch off when you pause, and after the launch has started, so they can cool down;
     the fingers are limp then. The *Hand motor torque* switch overrides this.
   - *Arm motion scale*: Accurate makes the arm move 60% of the glove motion. It can only
     be changed while paused.
   - *Reverse arm directions* (off by default): turn it on if the arm moves opposite to
     the hand. It turns the glove directions by 180 degrees about the vertical axis, so
     sideways and forward/back reverse together and up/down stays. It can only be changed
     while paused with no unsaved demo, and applies from the next *Start teleop*. Stored in
     `~/.ros/teleop_coil_adjust_right.json` (`yaw_deg`), which the teleop node reads at
     every resume; another angle can be written into the file by hand. Each saved demo's
     `.meta.json` records it as `glove_direction_yaw_deg`.
   - *Hand follows glove*, *Hand motor torque*.
   - *Custom pose*: bring the arm to the pose you want, pause, and press *Set*. *Move to
     custom* then returns the arm to it. The pose is
     stored in `~/.ros/teleop_start_pose_right.json`; delete the file to forget it.
   - *Center pose*: *Move to center* brings the arm to 45 cm forward, centred, 40 cm up,
     palm down (teleop node parameter `center_position`). It also works without the station
     launch: APE then starts the arm teleop node on its own (`ape_arm_mover`, process
     `mover`), which publishes arm targets only during a move. APE stops it before it starts
     a station launch, which brings its own teleop node. Only the Franka controller is needed.
   - *After stopping a recording*: Stay, Center or Custom. With Center or Custom,
     the arm moves there by itself as soon as you press *Stop recording and pause* (the
     fingers stay powered until it arrives). The choice is remembered
     (`~/.ros/ape_ui_settings.json`).
   - After reaching the position, each move spends up to 4 s settling: the arm on its own
     stops several degrees short in rotation (soft orientation control against joint
     friction, measured at about 7 degrees on 2026-11-19), so the teleop node nudges the
     command until the hand is within 1 degree of the target (node parameters
     `move_settle_*`; `move_settle_time: 0` switches it off).
   - Both moves only work while paused. The arm travels smoothly at about 10 cm/s and
     turns at about 30 degrees per second; *Stop move* halts it. Keep clear.
   - *Start teleop* needs the Franka controller running and the robot Ready.

4. **Live view:** both camera images (they run with the recording launch), one bar per
   finger (bar = measured, line = commanded, right = closed), the hottest hand motor, the
   arm position and its path seen from above (cleared at the start of each recording).
   *Hand details*, *Arm details* and *Other details* open on click and stay open:
   all joint values, motor currents and temperatures, arm pose commanded against measured,
   joint torques, external force, contact flags, stream rates and gaps, running nodes.
5. **Recorded demos:** lists every demo of a dataset with its size and check result.
   *Process and check new demos* synchronizes the demos that are not processed yet and
   runs the inspection on each; already processed demos are skipped. *View* plays a
   processed demo (both cameras, with a slider). Since 2026-11-25 processing (and the upload
   below) may run while you record: the job runs at the lowest CPU priority (`nice 19`), and
   the logger saves each demo as `<name>.h5.part` and renames it when complete, so a job never
   reads a half-written file. A demo saved while the job runs is picked up by the next *Process*.
   Not yet measured on a real recording (stream rates and gaps with and without a job).
   - *Process and check new demos* also archives and copies each demo to the Training machine
     (see "Video archive and transfer" below). The column *Archive / Training machine* shows
     per demo: not archived, archived, archive failed, not verified, verified, cleared here.
     Hover for details (PSNR of the archive check, which file differs).
   - *Clear transferred demos* deletes the raw, processed and archive files of the demos marked
     *verified* (course rule: clear the Robot PC after each session). Before deleting it compares
     every file with the Training machine again, byte for byte; a demo that does not match is
     kept. `<demo>.meta.json`, `manifest.csv` and the check results stay.
6. **Recalibrate hand at next start** (in *Hand details*): moves the calibration file to
   `~/rwr_humble_ws/hand_calibration_backups/`, so the hand calibrates itself when the
   next launch starts. Only possible while no launch is running. Do this only for a
   reason: demos from before and after a recalibration differ slightly.

7. **Run a policy** (card 4): runs a trained policy with `run_policy_inference.launch.py`.
   - Put each training run into `~/rwr_humble_ws/policies/<run>/`: its `config.yaml` and
     `checkpoints/*.pth`, in the layout `rwr_il` writes under `runs/`. Choose *Run* and
     *Checkpoint* in the card.
   - Stop the station launch first: the policy launch starts the cameras and the ORCA hand
     itself (the hand moves to neutral).
   - *Start preview*: the policy runs, but nothing is sent to the arm or the hand. The Live
     view shows what it would command (dashed line, ring, finger marks). Do this first with
     every new checkpoint; the card remembers which checkpoints were previewed.
   - *Start live*: the policy's outputs go to the arm and the hand once *Start policy* is
     pressed. Both steps ask for confirmation.
   - *Start policy* / *Stop policy* (also the big button in the Operate card, with a timer),
     *Reset policy state*, and *Move to center* (next to *Stop*): since 2026-11-25 the same
     move as in the Operate card (smooth, about 10 cm/s, with settling), through the arm
     mover, so it works with or without a policy launch. It is refused while the policy
     drives the arm, and *Start policy* is refused while the arm moves. While moving the
     button reads *Stop move*. (Before, the button only existed in a live launch and the
     inference node drove the arm with `init_arm` at about 2 cm/s; that path is still in
     the node and the page uses it only with a server from before this change.)
   - APE stops a live policy by itself when the robot leaves Ready (reflex, stop button,
     controller gone), so no targets pile up for the next controller start. Stop policy
     makes the arm hold its last target; it is not a safety stop.
   - **Checks** compare the station with the run's training data. The run's `config.yaml`
     names the dataset version (`dataset_name`); its demos are listed in
     `data/datasets/<version>/version.csv` and their settings in `raw/<session>/<demo>.meta.json`.
     Compared: control rate of the inference node against the demos' sampling rate, Franka
     end-effector and load parameters (read live from the robot), hand calibration and hand
     config, whether all demos were recorded with the same settings, observation and action
     names, which topic feeds which policy input, image size and cropping, the arm's distance
     from where the demos start, graphics card, and whether the checkpoint was previewed.
     A failed check ("does not match") blocks a live launch and a live policy start until it
     is confirmed in the page; a preview is always possible.
   - Each policy start is logged by the inference node to `~/rwr_humble_ws/inference_logs/`,
     with both images in every step (about 1.4 GB per minute, estimated from the node's code).

8. **Evaluate policy** (card 5): every live policy run is recorded and its placed cubes are
   counted, to compare checkpoints by numbers. The card does not start or stop anything and has
   no settings or buttons to press during a run.
   - A run is evaluated from *Start policy* to *Stop policy* in card 4 (or until the robot
     leaves Ready, or the policy launch stops). There is no time limit. Preview runs are not
     evaluated.
   - The cubes placed on the surface of their own colour are counted **live** from the workspace
     camera (`eval_detect.py --live`, a helper process at low priority that gets 10 frames per
     second). Card 5 shows the count next to the run's time. The helper runs whenever a policy
     launch runs, also in preview and before the policy is started, so the tracking can be tried
     by hand: put the cube on its surface and watch the count.
   - **Live view:** while the helper runs, the workspace image shows what it sees: the floor's
     outline (white), a ring on the cube (green while it is at its surface), the cube's path of
     the last 2.5 s, and the count. If the outline does not sit on the floor or the ring is not
     on the cube, the count cannot be right.
   - When the run has ended, the saved workspace video is counted once more (about 15 s for a
     3 minute run; the next run can be started meanwhile). The live count stays the result; if
     the recount gives another number, the run is marked with * and *Open* shows both.
   - Saved per run in `data/eval/<date-time>/`: `trial.json` (policy, placements found, result),
     `oakd_front_view.mp4` and `oakd_wrist_view.mp4` (10 frames per second, video time = run
     time), and `signals.h5` with every stream APE receives: measured arm pose, arm commands, the
     policy's arm and hand outputs, measured hand joints, hand commands, hand motor currents and
     temperatures, Franka joint angles, velocities, torques and external force, robot mode, and
     the policy's step timing. `data/eval/trials.csv` has one row per run.
   - *Evaluated runs* lists each run with its time, cubes placed, cubes placed in the first 3:00
     (what the course counts; for runs of 3:00 or more) and the seconds per cube.
   - *Open* shows one run: the times of the placements counted, the two videos to check them, −
     and + to correct the count (a + pressed while the videos are open adds a placement at the
     video's time), and a note.
   - **How the count works and how far it is checked.** The detector follows the cube on the
     wooden floor, where it stands out, and counts a placement when the cube reaches the surface
     of its colour and is then back on the floor, sliding away or lying somewhere else than
     before. The rule and its thresholds are in `eval_detect.py`. It was built on the red-cube
     demos of 2026-11-21 and its thresholds were set on them: it finds exactly one placement in
     each of the 66 single-relocation demos (arena poses a0, a1, a2) and 20 to 32 in the five
     3 minute demos, whose true numbers are not known. One of those gave 31 instead of 32 once
     its images had gone through the video compression used here. Counted the way the live
     helper does (the arena is taken from the last 30 s only), the demos give the same numbers
     and times, except one 3 minute demo: 19 instead of 20. In that demo the robot pushed the
     arena about 3 cm aside during the run. Not known: how often a failed attempt of a policy
     is counted by mistake (the demos hold almost no failures), blue and yellow cubes,
     daylight. A cube put on another surface is simply not counted.
   - *Measured automatically* (arm and hand motionless for 5 s or more, number of hand closings,
     arm path) is read from the signals by fixed rules in `eval_trials.py`, for information.
   - A live policy started outside the page (`ros2 topic pub`) is recorded as well, without its
     first moment.
   - Disk: a run's own files are small (videos and signals), but the inference node still
     writes its log, about 1.4 GB per minute.
   - Offline test without ROS: `tests/test_eval.py` (needs `fastapi` and `uvicorn`; with
     `APE_TEST_DEMO` it plays a recorded demo through the recorder and the live count).
     Try the detector on a file: `python eval_detect.py <oakd_front_view.mp4 or processed demo .h5>`.

The top card shows the robot state in words, with the recovery steps when the arm is in
reflex or user-stopped.

## Stopping

Stop teleop/recording and the controller with the page's buttons **before** closing the
server (Ctrl-C in its terminal). Ctrl-C also stops whatever the page started. Closing the
browser tab stops nothing.

## Notes

- Things started in a normal terminal are shown as "Running in a terminal"; the page
  cannot stop them and refuses to start a second copy.
- The page only listens on this laptop. `run.sh --host 0.0.0.0` opens it to every device
  on the network, without a password, including the start and stop buttons.
- The NUC password goes from the password box straight into ssh and is never stored. The
  box only accepts it while ssh is asking for it. The *Terminal* fold-out in the NUC card
  is a real terminal on the NUC, for typing a command by hand if a step goes wrong.
- APE never sends arm or hand commands itself. Its buttons call the same ROS services the
  keyboard runner calls (pause/resume, logger, retargeter, hand torque).
- The motion scale switch and the pose moves need the `set_accurate_mode`,
  `set_start_pose_here` and `move2center` services that were added to
  `rwr_system/experiments/run_teleop_rokoko.py`.
- Hand motor current and temperature need the two topics that were added to
  `rwr_system/src/hand_control/hand_control_node.py` (`/right/hand/motor_current`,
  `/right/hand/motor_temperature`). Without them the page shows "not reported".
- Processing uses the `--skip_existing` and `--max_parallel` options that were added to
  `rwr_system/src/logger/syncronize_data.py`, and its `--output_folder` option. Check results are stored in
  `~/rwr_humble_ws/work/ape_previews/`.
- `tests/mock_station.py` is a fake station for trying APE without the robot. It refuses
  to run on the robot's ROS domain. `tests/mock_inference.py` stands in for the policy launch
  (`APE_INFERENCE_TEST_CMD`); `tests/test_inference.py` covers the policy card and
  `tests/make_policy_config.py` writes a training `config.yaml` for it without training.
  Tests import `server.py`, so a test that calls a NUC endpoint opens a real ssh connection
  to the NUC unless it replaces `nuc.start_controller` first (as `test_inference.py` does).
- *Prediction time* in the policy card (shown while a policy runs, also in preview): the typical and the slowest time of a
  prediction over the last 10 s, and a bar for the oldest camera image or robot reading at the end of the slowest prediction,
  against the inference node's 200 ms limit (dashed line). Green below 150 ms, amber above, red when a step was dropped.
  It needs the `/policy_inference/timing` topic that was added to `rwr_system/experiments/model_inference_node.py`;
  with an older node the meter stays hidden. `summarise_timing` in `server.py` does the arithmetic.
- The policy card reads `APE_POLICY_DIR` (default `~/rwr_humble_ws/policies`). The checks are
  in `policy_checks.py`.
- Data layout (since 2026-11-21): section 2 takes a session number and an operator; the
  page builds the *session* name `s<nn>_<date>_<operator>` from them and today's date, and
  refuses a number that another session already uses. The operator list is the
  `<select id="ses-op">` in `static/index.html`. Recordings go to `~/rwr_humble_ws/data/raw/<session>/`,
  processed files to `data/processed/<session>_10hz/`. `data/manifest.csv` has one row per
  demo; dataset versions and the Hugging Face transfer are done by
  `rwr_il/scripts/rwr_data.py` (see `rwr_il/CLAUDE.md`).
- *Upload to Hugging Face* runs `rwr_data.py push` as the processing job: raw and processed
  recordings that are not uploaded yet, the manifest and the dataset version lists. It needs
  a one-time `pixi run hf auth login` and `rwr_data.py push --repo <user>/<name>` in a
  terminal. It may run during a recording, at the lowest CPU priority, like processing.
- The files as they were before the data layout change are in `backup_before_data_layout/`.
- Live view arm plots: path from above and from the side, measured (blue) and commanded
  (dashed, only while teleop runs), the custom pose as a diamond, plus the readouts *Arm
  behind its command* and *Distance to custom pose*.
- *Lighting* (day, night) is a fourth demo label. At every save APE also writes
  `data/raw/<session>/<demo>.meta.json` with the settings in use: labels, arm motion scale,
  the Franka end-effector and load parameters read from the robot state, the hand config
  and calibration files, both retargeter config files and the `rwr_system` commit.
- The start marker (diamond) and the readout *Distance to the start of this demo* use the
  arm pose at the moment a recording is started from the page; before the first recording
  they use the saved custom pose.
- *Description* is an optional free-text label for the next saved demo only (cleared after
  the save, stored in the `description` column of `manifest.csv`). The *Next demo* line
  under the labels and the *Name in dataset versions* column in section 3 show the file
  name a demo gets in dataset versions (`<session prefix>_<colour>_<number>.h5`), computed
  the same way as in `rwr_data.py`.

## Video archive and transfer

After synchronizing and checking, the processing job runs `archive_job.py transfer` for the session:

1. **Archive** (`src/logger/encode_archive.py`, niced, 2 recordings at a time): each raw recording becomes
   `data/archive/<session>/<demo>.<camera>.mp4` (libx264 4:4:4, CRF 12, keyframe every 30 frames) plus
   `<demo>.h5` with every other topic unchanged and the image timestamps, `<demo>.meta.json`, and
   `<demo>.archive.json` (settings, sha256 of each file, PSNR of every frame against the raw image).
   About 6 % of the raw file size (about 2.4 MB per second of recording). `src/logger/reconstruct_raw.py` turns it back into a raw-format file that
   `syncronize_data.py` processes like the original (all non-image data bit for bit; images lossy).
2. **Copy** with `rsync -a --ignore-existing` to the Training machine: `archive/<session>/` and (unless
   `archive_transfer_processed` is false) `processed/<session>_10hz/` under its data root. Files that already
   exist there are never overwritten.
3. **Verify** with `rsync --checksum --dry-run`: a demo is *verified* when all its archive files (and its
   processed file) exist on the Training machine with the same content. The same run fetches the Hugging Face
   markers (below) and shows *verified + HF* for demos whose archive is backed up there.
4. **Manifest**: `manifest.csv` is copied to `manifest_inbox/robot_pc.csv` on the Training machine (replaced every
   time). The Training machine's own `manifest.csv` is never overwritten: `rwr_data.py merge-manifest` there adds
   new rows, keeps its own values (e.g. `keep` or labels corrected there), fills empty fields and lists
   disagreements in `manifest_conflicts.csv`.
5. **Hugging Face backup** (on the Training machine, by hand or scheduled there):
   `python src/logger/hf_backup_archive.py` uploads `archive/<session>/`, `manifest.csv` (the merged one; skipped
   while `manifest_inbox/` has unmerged changes) and `datasets/*/version.csv` to the private repo in `data/hf_repo.txt`, compares the Hub's sha256 of every
   archive file with the local file and writes `archive/<session>/<demo>.hf.json` for each demo that matches.

Setup, once per station, in the git-ignored `static/local/station.json` (next to `robot_ip`):

```json
{"robot_ip": "...", "archive_host": "<ssh alias of the Training machine>", "archive_data_root": "~/rwr_task2/data",
 "archive_remote_rsync": "~/.local/rsync/rsync", "archive_transfer_processed": true, "archive_require_hf_backup": true}
```

- `archive_remote_rsync`: the Training machine has no system rsync; an unpacked rsync 3.2.7 with a wrapper
  lives in `~/.local/rsync/rsync` there (home directory, survives restarts of the instance).
- `archive_transfer_processed` (default true): also copy and verify the processed files (lossless at 10 Hz).
  With false only the archive is copied; clearing then deletes the local processed files too, and they can be
  rebuilt on the Training machine with `reconstruct_raw.py` + `syncronize_data.py` (images from the video).
- `archive_require_hf_backup` (default true): *Clear transferred demos* only deletes demos that are verified on
  the Training machine **and** whose archive is on Hugging Face with the same sha256 (`<demo>.hf.json`).

`archive_host` must log in with a key and without any prompt (the job uses ssh `BatchMode`). Both machines
need rsync 3.2.3 or newer (`--mkpath`). ffmpeg with libx264 must be available on the Robot PC (`ffmpeg`
on PATH, `$FFMPEG`, or `pip install imageio-ffmpeg`). Without `archive_host` the job only archives.
The same settings can be given as `$APE_ARCHIVE_HOST`, `$APE_ARCHIVE_DATA_ROOT`, `$APE_REMOTE_RSYNC`,
`$APE_ARCHIVE_TRANSFER_PROCESSED`, `$APE_ARCHIVE_REQUIRE_HF_BACKUP` (0/1).
Status files: `work/archive_status/<session>/<demo>.json`. By hand:
`python ape_ui/archive_job.py {transfer,verify,clear} --data data --session <session> --status-dir work/archive_status --host <alias>`.

The Hugging Face upload is unchanged: it uploads what is on disk, so upload before clearing if the raw
recordings should also go to Hugging Face.
