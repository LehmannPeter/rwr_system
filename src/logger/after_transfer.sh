#!/bin/bash
# After an archive transfer (Training machine): merge-manifest -> HF backup -> rebuild processed files.
#
# Installed as ~/rwr_task2/after_transfer.sh (a copy of src/logger/after_transfer.sh; the settings below live in that
# copy). APE's archive job on the Robot PC starts it over ssh after verifying a transfer (station.json
# "archive_after_transfer": "~/rwr_task2/after_transfer.sh"); it can also be run by hand at any time:
#     ~/rwr_task2/after_transfer.sh [--session S]          # returns at once; the work runs detached
#     ~/rwr_task2/after_transfer.sh --foreground           # same, but waits (for tests and by hand)
#
# - One run at a time (flock on $LOCK in /tmp: released when the process dies, wiped by a restart). A trigger that
#   arrives during a run leaves $PENDING, and the running one does one more pass at the end.
# - Each step runs under supervise_job.sh (16 GiB per job, nice 19, CPU only, killed above 130 GiB unreclaimable or
#   150 GiB total in the container). A failed or killed step is logged and the next step still runs; all three are
#   idempotent, so the next trigger redoes whatever is missing.
# - Log: $LOGDIR/<date-time>.log (+ latest.log). Nothing keeps running after the last step.
set -u

# ---- settings (override by environment for tests) ----------------------------------------------------------------
CHECKOUT=${CHECKOUT:-~/rwr_task2/rwr_system_archive_only}        # rwr_system checkout with src/logger/*
RWR_IL=${RWR_IL:-~/rwr_task2/rwr_il}                             # rwr_il checkout with merge-manifest
PY=${PY:-$RWR_IL/.venv/bin/python}
DATA=${DATA:-~/rwr_task2/data}
MERGE_SCRIPT=${MERGE_SCRIPT:-$RWR_IL/scripts/rwr_data.py}
HF_FLAGS=${HF_FLAGS:---allow-public}       # the repo is public (team decision 2026-11-25); remove once it is private
REBUILD_JOBS=${REBUILD_JOBS:-2}            # 2 x ~6.2 GiB (3-minute demo) fits the 16 GiB per-job cap
REBUILD_JOB_GIB=${REBUILD_JOB_GIB:-16}
LOGDIR=${LOGDIR:-~/rwr_task2/logs/after_transfer}
LOCK=${LOCK:-/tmp/rwr_after_transfer.lock}
PENDING=${PENDING:-/tmp/rwr_after_transfer.pending}
# -------------------------------------------------------------------------------------------------------------------

SELF=$(readlink -f "$0")
mkdir -p "$LOGDIR"
if [ "${1:-}" != "--foreground" ]; then           # detach: the caller (ssh) returns at once
  setsid nohup "$SELF" --foreground "$@" </dev/null >/dev/null 2>&1 &
  echo "after_transfer: started; log in $LOGDIR/latest.log"
  exit 0
fi
shift
ARGS="$*"
SUP="$CHECKOUT/src/logger/supervise_job.sh"
touch "$PENDING"                                  # this trigger's request

run_pass() {   # one pass of the three steps; output appended to $LOG
  local pass=$1 t code
  echo "=== $(date '+%F %T') pass $pass"
  t=$LOG_BASE.p$pass
  step() {     # step <name> [env] <command...>
    local name=$1; shift
    echo "--- $(date '+%T') $name: $*"
    "$@"; code=$?
    [ -f "$t.$name.log" ] && sed 's/^/    /' "$t.$name.log" | grep -v '^    \(Processing topic\|Saved \|Topic \|Submitting\|File processed\|Output folder\|All files\|Processed data\)' | tail -60
    # the supervisor's own verdict is in its .mem file (exit codes 2/3 could also come from the job itself)
    if grep -q "KILLED" "$t.$name.mem" 2>/dev/null; then s="KILLED by a memory limit ($(grep KILLED "$t.$name.mem" | tail -1))"
    elif grep -q "not starting" "$t.$name.mem" 2>/dev/null; then s="NOT STARTED (container memory above the limits)"
    elif [ "$code" -eq 0 ]; then s=ok
    else s="FAILED (exit $code)"; fi
    echo "--- $(date '+%T') $name: $s"
    rm -f "$t.$name.log" "$t.$name.mem"
  }
  step merge-manifest "$SUP" "$t.merge-manifest" "$PY" "$MERGE_SCRIPT" --data "$DATA" merge-manifest
  # shellcheck disable=SC2086
  step hf-backup "$SUP" "$t.hf-backup" "$PY" "$CHECKOUT/src/logger/hf_backup_archive.py" --data "$DATA" $HF_FLAGS
  step rebuild env SUPERVISE_JOB_GIB="$REBUILD_JOB_GIB" "$SUP" "$t.rebuild" "$PY" "$CHECKOUT/src/logger/rebuild_processed.py" \
       --data "$DATA" --jobs "$REBUILD_JOBS"
}

while true; do
  exec 9>"$LOCK"
  if ! flock -n 9; then
    [ -L "$LOGDIR/latest.log" ] && echo "=== $(date '+%F %T') trigger ($ARGS): a run is in progress; one more pass requested" >> "$LOGDIR/latest.log"
    exit 0
  fi
  if [ -z "${LOG:-}" ]; then
    LOG_BASE="$LOGDIR/$(date +%Y%m%d-%H%M%S)"; LOG="$LOG_BASE.log"
    ln -sfn "$(basename "$LOG")" "$LOGDIR/latest.log"
    echo "=== $(date '+%F %T') after_transfer start ($ARGS), checkout $CHECKOUT ($(git -C "$CHECKOUT" rev-parse --short HEAD 2>/dev/null)), data $DATA" >> "$LOG"
  fi
  pass=${pass:-0}
  while [ -e "$PENDING" ]; do
    rm -f "$PENDING"; pass=$((pass + 1))
    run_pass $pass >> "$LOG" 2>&1
  done
  flock -u 9; exec 9>&-
  [ -e "$PENDING" ] || break                      # a trigger came between the last check and the unlock: go again
done
echo "=== $(date '+%F %T') after_transfer done ($pass pass(es))" >> "$LOG"
