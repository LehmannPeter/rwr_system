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
