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
     palm down (teleop node parameter `center_position`).
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
   processed demo (both cameras, with a slider). Processing is refused while a demo is
   being recorded or saved.
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
     *Reset policy state*, and in a live launch *Move to center*: the same center pose
     as *Move to center* in the Operate card, with the same rotation correction. The teleop
     node is not running in a policy launch, so the inference node drives the arm
     (`init_arm`, about 2 cm per second). The correction is read when the launch starts.
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
  terminal, and is refused while a demo is being recorded or saved.
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
