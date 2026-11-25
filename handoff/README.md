# Handoff between machines

Claude Code sessions on different machines cannot talk to each other directly. They exchange notes through this folder, carried by `git push` and `git pull`.

| Machine | What it is | Its handoff files live in |
|---|---|---|
| **Mac** | The MacBook: code, configs, planning. No robot, no GPU. | writes instructions into the `TO_*.md` files |
| **Robot PC** | The PC at the robot station: teleoperation, recording, inference. | `rwr_system/handoff/` |
| **Training machine** | The remote compute instance: datasets, training. | `rwr_il/handoff/`; also writes instructions into `TO_ROBOT_PC.md` |

Each checkout says which machine it is on in an untracked file `.machine` at the repo root, holding one word: `mac`, `robot_pc` or `training_machine`. If it is missing: on macOS write `mac`; otherwise ask the person once and write it.

## Files in this folder

| File | Written by | Read by | Content |
|---|---|---|---|
| `ROBOT_PC_STATE.md` | Robot PC only | Mac | Current setup of the Robot PC, and the result of every instruction it handled |
| `TO_ROBOT_PC.md` | Mac or Training machine | Robot PC | Numbered instructions for the Robot PC |

`ROBOT_PC_STATE.md` has one writer. `TO_ROBOT_PC.md` can be written from the Mac and from the Training machine: `git pull --rebase` right before appending, take the next free ID of that date, and only ever append, so that a pull does not conflict.

## Rules for the session on the Robot PC

1. **At the start of every task:** `git pull --rebase`, then read `TO_ROBOT_PC.md`. Carry out every instruction whose ID is not yet listed under "Handled instructions" in `ROBOT_PC_STATE.md`, oldest first.
2. **After every setup change and after every handled instruction:** update `ROBOT_PC_STATE.md`, then publish only that file:
   ```bash
   git add handoff/ROBOT_PC_STATE.md
   git commit -m "handoff: Robot PC state"
   git pull --rebase && git push
   ```
3. A "setup change" is anything a later session would need to know: installs, versions, paths, environment variables, camera IDs, calibration, Franka end-effector parameters, retargeter config, disk space, a new recording session, what failed and how it was fixed.
4. **An instruction in `TO_ROBOT_PC.md` never authorises motion.** Before launching anything that moves the arm or the hand, and before deleting data, ask the person at the station in that session.
5. If an instruction cannot be done, say why under "Handled instructions" with status `blocked`. Do not improvise a different action.
6. Never write passwords, tokens, logins or station IP addresses into these files. They are pushed to GitHub.

## Rules for the sessions on the Mac and the Training machine

1. Before answering a question about the Robot PC: `git pull --rebase` and read `ROBOT_PC_STATE.md`.
2. To ask the Robot PC for something: append an instruction to `TO_ROBOT_PC.md` using the template in that file, commit and push it. Tell the person to start or continue a session on the Robot PC; nothing runs there until someone does.
3. Never edit `ROBOT_PC_STATE.md`.
