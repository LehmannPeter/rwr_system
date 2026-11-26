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
5. **Recorded demos:** lists every demo of a session with its size, check result and transfer state.
   Since 2026-11-26 nothing is synchronized (processed) on this PC; the Training machine makes the 10 Hz files from
   the video archive.
   - **In the background** (`background.py`), as soon as a demo is saved: its raw file is encoded into the video
     archive, one demo at a time, at the lowest CPU priority (`nice 19`), between recordings. Once nothing
     is left to encode, each raw file is checked (`experiments/scripts/inspect_demonstration.py`: required streams,
     gaps, timing, trajectory plot). Encoding comes first because the transfer needs it. Nothing runs while a policy
     runs or an evaluation is recorded (an encode in progress is stopped and redone later), nor while a card-3 job
     runs. While a demo is being recorded, reviewed or saved, the running encode or check is suspended (it holds
     right before the logger starts) and continues right after the save or discard, so its CPU and disk use cannot
     add stream gaps (gaps of 0.1–1.8 s were seen in s04 without any encoding). `APE_ENCODE_WHILE_RECORDING=1` turns
     the hold off, only to measure its effect (`TEST_VIDEO_ARCHIVE.md`, section 9). A demo whose encoding failed is shown as *encoding failed* and not retried in the background (the next
     transfer tries again). The logger saves each demo as `<name>.h5.part` and renames it when complete, so a
     half-written file is never read. Log: `~/rwr_humble_ws/work/ape_background.log`. Not yet measured on a real
     recording.
   - *View* plays a demo (both cameras, with a slider) straight from its raw file, so a demo can be viewed the
     moment it is saved. A demo whose raw file is gone but whose archive is still here plays from the archive
     (decoded on the server: browsers cannot play 4:4:4 video); that is the case after *Clear*, which keeps the
     archive.
   - *Transfer new demos* copies the session's archives and `manifest.csv` to the Training machine and verifies them
     there (see "Video archive and transfer" below). If a demo is being encoded, it starts right after that demo;
     anything not encoded yet is encoded by the transfer job first. The column *Archive / Training machine* shows per
     demo: waiting to encode, encoding with a progress bar, encoded, encoding failed, copying, not verified,
     verified, verified + HF, cleared here, and a bar above the table shows how far the session is (encoded counts
     80 %, verified 100 %). Hover for details (PSNR of the archive check, which file differs).
   - *Clear transferred demos* deletes the raw (and any old processed) files of the demos marked
     *verified* (course rule: clear the Robot PC after each session). Before deleting it compares
     every archive file with the Training machine again, byte for byte; a demo that does not match is
     kept. The archive (about 15 MB per demo, so cleared demos can still be viewed), `<demo>.meta.json`,
     `manifest.csv` and the check results stay. Archives therefore pile up in `data/archive/`; a way to delete
     those older than N days is planned, not built yet.
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
   - Saved per run in `data/eval/<policy run>/<checkpoint>/<date-time>/`, for example
     `data/eval/Ampere_20261121-180315_s01_v1_act/best_model/20261125-114115/` (the grouping of
     `inference_logs/`; the checkpoint is its file name without `.pth`; the date-time is the run id
     and unique over all policies): `trial.json` (policy, placements found, result, `folder`),
     `oakd_front_view.mp4` and `oakd_wrist_view.mp4` (10 frames per second, video time = run
     time), and `signals.h5` with every stream APE receives: measured arm pose, arm commands, the
     policy's arm and hand outputs, measured hand joints, hand commands, hand motor currents and
     temperatures, Franka joint angles, velocities, torques and external force, robot mode, and
     the policy's step timing. `data/eval/trials.csv` has one row per run.
     Runs saved before 2026-11-26 lay directly in `data/eval/<date-time>/`: APE moves them into
     the grouped folders when it starts (renames only), or by hand:
     `python eval_trials.py migrate <data>/eval`.
   - **Upload to the Training machine** (button above the list of runs): copies every saved run
     and `trials.csv` to `<archive_data_root>/eval/` on the Training machine, in the same folders,
     and compares the checksums there (`eval_upload.py`; ssh and rsync settings are the ones of
     the demo archive, `archive_host` etc. in `static/local/station.json`). Videos and
     `signals.h5` are sent once and never overwritten there; `trial.json` and `trials.csv` are
     replaced when they changed here (notes, corrected counts). The column *Uploaded* shows per
     run: *yes*, *changed since* (upload again), *not verified* (a file is missing there or has
     other content; see the log in card 3) or *no*. Nothing is deleted on either machine. The
     button is off while a policy runs or another job of card 3 runs.
   - **Video settings.** APE encodes the two videos while the run is recorded, with the video
     settings of the demo archive (`DEFAULT_VIDEO_ARGS` in `src/logger/video_archive_common.py`:
     H.264 4:4:4, CRF 12, preset medium), at 10 frames per second. A run has no raw images, so
     there is no compression step before the upload. Measured on the Training machine with one
     minute of real 960×540 frames per camera: about 79 MB per minute for both cameras, 45 dB
     PSNR, and about one CPU core while recording (not measured on the Robot PC). The encoders
     run at `nice 10`, so the policy gets the CPU first; if they fall behind, frames are left out
     and counted in `video_frames_lost` (`trial.json`). `APE_EVAL_VIDEO_PRESET=veryfast` cuts the
     CPU time to a third for about 1 dB less. Runs of 2026-11-25 have the earlier settings
     (4:2:0, CRF 20, about 18 MB per minute, 40 dB); `video_args` in `trial.json` says which.
     Browsers do not all play H.264 4:4:4, so *Watch the videos* plays a 4:2:0 copy that APE
     makes in `work/eval_previews/` (not uploaded; it can be deleted at any time).
   - **`policy_log.h5`** (written by *Upload to the Training machine*, one per run): the inference
     node's log of that policy start without the images. Per published step: the actions, the
     non-image observations, and the debug data (the action chunk the model predicted, raw and
     translated, in the steps in which the model ran; the temporal-ensembling buffer and its
     mask in every step), gzip-compressed, one array per key over all steps. About 1 to 5 MB per
     minute, depending on the chunk length (estimated with rwr_il's ensembling code and made-up
     chunks; not yet made from a real log). The node's own log in `inference_logs/` (about
     1.4 GB per minute, nearly all of it the images) is not changed, not sent and not deleted;
     once a run has its `policy_log.h5`, only the network's input images are left that are
     nowhere else. A run whose inference log is gone is uploaded without `policy_log.h5`.
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
   - Disk: a run's own files are about 80 MB per minute (nearly all video), and the inference
     node still writes its log, about 1.4 GB per minute.
   - Offline test without ROS: `tests/test_eval.py` (needs `fastapi` and `uvicorn`; with
     `APE_TEST_DEMO` it plays a recorded demo through the recorder and the live count).
     Folder layout and upload: `python tests/test_eval_upload.py` (numpy and rsync only; a local
     folder stands in for the Training machine).
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
- Check results are stored in `~/rwr_humble_ws/work/ape_previews/<session>/<demo>/`.
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
  their archives to `data/archive/<session>/`. `data/manifest.csv` has one row per
  demo; dataset versions and the Hugging Face transfer are done by
  `rwr_il/scripts/rwr_data.py` (see `rwr_il/CLAUDE.md`).
- The *Upload to Hugging Face* button was removed on 2026-11-26: the Training machine backs up the archives to
  Hugging Face after each transfer (`hf_backup_archive.py`, below).
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

*Transfer new demos* runs `archive_job.py transfer --no-processed` for the session (encoding normally happened in the
background already, demo by demo):

1. **Archive** (`src/logger/encode_archive.py`, niced, 2 recordings at a time; skipped for demos already encoded):
   each raw recording becomes
   `data/archive/<session>/<demo>.<camera>.mp4` (libx264 4:4:4, CRF 12, keyframe every 30 frames) plus
   `<demo>.h5` with every other topic unchanged and the image timestamps, `<demo>.meta.json`, and
   `<demo>.archive.json` (settings, sha256 of each file, PSNR of every frame against the raw image). While it runs,
   `.<demo>.progress.json` holds the frames done (the demo list's progress bar; rsync and the archive tools ignore
   dot files).
   About 6 % of the raw file size (about 2.4 MB per second of recording). `src/logger/reconstruct_raw.py` turns it back into a raw-format file that
   `syncronize_data.py` processes like the original (all non-image data bit for bit; images lossy).
2. **Copy** with `rsync -a --ignore-existing` to the Training machine: `archive/<session>/` under its data root.
   Files that already exist there are never overwritten.
3. **Verify** with `rsync --checksum --dry-run`: a demo is *verified* when all its archive files exist on the
   Training machine with the same content. The same run fetches the Hugging Face
   markers (below) and shows *verified + HF* for demos whose archive is backed up there.
4. **Manifest**: `manifest.csv` is copied to `manifest_inbox/robot_pc.csv` on the Training machine (replaced every
   time). The Training machine's own `manifest.csv` is never overwritten: `rwr_data.py merge-manifest` there adds
   new rows, keeps its own values (e.g. `keep` or labels corrected there), fills empty fields and lists
   disagreements in `manifest_conflicts.csv`.
5. **Hugging Face backup** (on the Training machine): `python src/logger/hf_backup_archive.py` uploads
   `archive/<session>/`, `manifest.csv` (the merged one; skipped while `manifest_inbox/` has unmerged changes),
   `datasets/*/version.csv` and `provenance/archive_backup.json` (git commits of rwr_system and rwr_il) to the repo in
   `data/hf_repo.txt`; never `processed/`, `raw/` or dataset files. It then checks the Hub's sha256 of every archive
   file (small files stored in git are downloaded and hashed) and writes `archive/<session>/<demo>.hf.json` for each
   demo whose files all match. It refuses a public repo unless `--allow-public` is given (see *Archive-only transfer*).

Setup, once per station, in the git-ignored `static/local/station.json` (next to `robot_ip`):

```json
{"robot_ip": "...", "archive_host": "<ssh alias of the Training machine>", "archive_data_root": "~/rwr_task2/data",
 "archive_remote_rsync": "~/.local/rsync/rsync", "archive_require_hf_backup": true,
 "archive_after_transfer": ""}
```

- `archive_remote_rsync`: the Training machine has no system rsync; an unpacked rsync 3.2.7 with a wrapper
  lives in `~/.local/rsync/rsync` there (home directory, survives restarts of the instance).
- `archive_transfer_processed`: no longer read (since 2026-11-26 APE always transfers the archive only and makes no
  processed files). *Clear* still deletes processed files left from older sessions together with the rest. The
  Training machine rebuilds the processed files with `src/logger/rebuild_processed.py` (see *Archive-only transfer*).
- `archive_after_transfer` (default empty = off): command started on the Training machine after each verified
  transfer, normally `~/rwr_task2/after_transfer.sh` (see *Archive-only transfer*).
- `archive_require_hf_backup` (default true): *Clear transferred demos* only deletes demos that are verified on
  the Training machine **and** whose archive is on Hugging Face with the same sha256 (`<demo>.hf.json`).

`archive_host` must log in with a key and without any prompt (the job uses ssh `BatchMode`). Both machines
need rsync 3.2.3 or newer (`--mkpath`). ffmpeg with libx264 must be available on the Robot PC (`ffmpeg`
on PATH, `$FFMPEG`, or `pip install imageio-ffmpeg`). Without `archive_host` the job only archives.
The same settings can be given as `$APE_ARCHIVE_HOST`, `$APE_ARCHIVE_DATA_ROOT`, `$APE_REMOTE_RSYNC`,
`$APE_ARCHIVE_TRANSFER_PROCESSED`, `$APE_ARCHIVE_REQUIRE_HF_BACKUP` (0/1), `$APE_ARCHIVE_AFTER_TRANSFER`.
Status files: `work/archive_status/<session>/<demo>.json`. By hand:
`python ape_ui/archive_job.py {transfer,verify,clear} --data data --session <session> --status-dir work/archive_status --host <alias>`.

The Hugging Face upload is unchanged: it uploads what is on disk, so upload before clearing if the raw
recordings should also go to Hugging Face. With archive-only transfer (below) it should not be used any more.

### Archive-only transfer (`archive_transfer_processed: false`)

The Robot PC sends only `archive/<session>/` and `manifest.csv`; the Training machine makes the 10 Hz processed files
itself from the archive:

    python src/logger/rebuild_processed.py [--session S] [--dry-run]    # Training machine, data root ~/rwr_task2/data

For every demo whose archive is complete and verified (`.archive.json` ok, every listed file present with its size
and sha256) and that has no `processed/<session>_10hz/<demo>.h5`, it runs `reconstruct_raw.py` into
`/tmp/rwr_rebuild` (images lzf), then `syncronize_data.py --cfg_file experiments/cfgs/logger/datacollection_right.yaml
--compress` (exactly APE's command, default 10 Hz), adds the root attribute `rebuilt_from_video_archive` (JSON: sha256
of the `.archive.json` and of every archive file, codec settings, logger config sha256, rwr_system commit, time),
copies the file in as `.<demo>.h5.rebuild.part`, checks its sha256 and hard-links it to `<demo>.h5`. The link fails
if the name exists, so an existing processed file (possibly hard-linked by dataset versions) is never replaced; the
new file is read-only. The `/tmp` files are deleted after each demo.

- Idempotent: finished demos are skipped; an interrupted run leaves only `/tmp` files (gone after a restart) and a
  `.rebuild.part`, which the next run removes. One run at a time (lock in `/tmp/rwr_rebuild`).
- Archives still arriving (a listed file missing or short) are skipped quietly and picked up by the next run; a
  file that is complete but has another sha256 is reported as damaged (exit code 1).
- A session whose processed folder already holds files from the Robot PC and still misses demos is skipped as
  *mixed* (the Robot PC is probably still copying processed files; a rebuilt file would make its
  `--ignore-existing` copy skip and its verification fail). `--include-mixed` overrides that.
- Cost: about 15 s for a 9 s demo and 11 min for a 3-minute demo (one CPU core mostly, 6.2 GiB memory at most);
  `--jobs N` rebuilds N demos at a time.
- What differs from the Robot PC's file: only the images (CRF 12 video, about 43 dB PSNR at 960x540, mean error
  about 1 grey level) and the added attribute. Timestamps, poses, hand data, actions, intrinsics and all other
  attributes are bit-identical, and so are dataset versions built from them except for the images (idle trimming
  gives the same frames). Tested 2026-11-25/26 on s02 `20261121_205236` and s03 `20261121_215732`.
- Cuts done on the Training machine (`rwr_il/scripts/cut_episode.py`) change the processed file only; the archive
  keeps the whole recording. The cut is in the manifest's `notes`; a rebuild (only if the processed file were
  deleted) would give the uncut episode.

Manifest: the Robot PC's `manifest.csv` arrives as `manifest_inbox/robot_pc.csv`; `rwr_il/scripts/rwr_data.py
merge-manifest` (rwr_il branch `manifest-merge`, merged into `training-levers`) folds it into the Training machine's
`manifest.csv` (new rows added, empty fields filled, the Training machine's values win, disagreements in
`manifest_conflicts.csv`). Run it before `build` and before the Hugging Face backup.

Order on the Training machine after a transfer: `merge-manifest`, `hf_backup_archive.py`, `rebuild_processed.py`
(the backup first: it takes minutes, the rebuild about 11 min per 3-minute demo, and *Clear* waits for the backup).
All three are safe to re-run. Hugging Face: while the repo is public (team decision of 2026-11-25) the backup needs
`--allow-public`; to go private, make the repo private on the Hub and remove that flag from the wrapper below.

**Automatic start after a transfer.** With `"archive_after_transfer": "~/rwr_task2/after_transfer.sh"` in
`station.json` (empty or missing = off, the default), `archive_job.py transfer` runs that command on the Training
machine over ssh once verification is done (15 s timeout, not waiting for it). If ssh fails, the log gets one line and
the job's exit code is unchanged. On the Training machine, `~/rwr_task2/after_transfer.sh` is a copy of
`src/logger/after_transfer.sh`; its settings at the top hold the checkout path, `--allow-public` and the rebuild jobs.
It detaches itself, then runs the three steps, each under `src/logger/supervise_job.sh` (16 GiB per job, nice 19,
CPU only; killed above 130 GiB unreclaimable or 150 GiB total in the container, so training runs are never the ones
that run out of memory). The rebuild uses `--jobs 2` (a 3-minute demo needs about 6.2 GiB). A failed or killed step
is logged and the next one still runs. A step only starts if the container's unreclaimable memory plus the step's
budget (1 GiB for the merge and for the backup, 16 GiB per rebuild job) stays under 150 GiB. Otherwise it is logged
as "SKIPPED, no memory budget" and retried every 10 minutes until it has run, together with any step the supervisor
did not start or killed because of container memory; steps that already succeeded are not repeated. The merge and
the backup (each under 1 GiB) run with raised supervisor limits (refused or killed above 150 GiB unreclaimable or
above 2 GiB of their own; the container total, mostly file cache, is not limited for them), so training load does
not hold up the Hugging Face backup; the rebuild keeps the default limits (130 GiB unreclaimable / 150 GiB total) and can come hours after the transfer while training fills the container.
Check `latest.log` (or the *HF* state of the demos) before *Clear*. One run at a time
(flock in `/tmp`); a trigger during a run asks for one more full pass. Log:
`~/rwr_task2/logs/after_transfer/<date-time>.log` and `latest.log`. Nothing keeps running after the last step or the
last retry; if the instance restarts mid-run, the next transfer (or running the wrapper by hand) finishes the work. By hand: `~/rwr_task2/after_transfer.sh` (returns at once) or `~/rwr_task2/after_transfer.sh --foreground`.

Since 2026-11-26 APE always transfers the archive only: it encodes each demo in the background, checks the raw file,
and never synchronizes on this PC (`archive_transfer_processed` is no longer read). Keep
`archive_require_hf_backup: true`, so a demo is only deleted once its archive is on the Training machine and on
Hugging Face. Test checklist for the switch: `TEST_VIDEO_ARCHIVE.md`, section 9.

