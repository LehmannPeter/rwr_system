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
# - Memory budget: a step only starts if the container's unreclaimable memory (anon + shmem) plus the step's budget
#   ($*_NEED_GIB: 1 GiB for the merge and the backup, 16 GiB per rebuild job) stays under $START_LIMIT_GIB (150 GiB).
#   Otherwise it is logged as skipped. A skipped step, one the supervisor did not start, and one it killed because of
#   container memory are retried every $RETRY_EVERY s (default 10 min) until each has run ($RETRY_HOURS > 0 sets a
#   time limit); steps that already succeeded in this run are not repeated (a new trigger asks for a full pass).
#   The merge and the backup (each well under 1 GiB) run with raised supervisor limits: refused or killed above
#   150 GiB unreclaimable in the container or above 2 GiB of their own; the container total (mostly file cache) is not
#   limited for them. The rebuild keeps the default 130 GiB unreclaimable / 150 GiB total (team decision 2026-11-26,
#   so the backup is not blocked by training load).
# - Log: $LOGDIR/<date-time>.log (+ latest.log). Nothing keeps running after the last step or the last retry.
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
LIGHT_LIMITS=${LIGHT_LIMITS:-SUPERVISE_UNREC_GIB=150 SUPERVISE_TOTAL_GIB=0 SUPERVISE_RSS_GIB=2}   # merge + backup
LOCK=${LOCK:-/tmp/rwr_after_transfer.lock}
PENDING=${PENDING:-/tmp/rwr_after_transfer.pending}
MERGE_NEED_GIB=${MERGE_NEED_GIB:-1}        # budget; measured peak 0.0 GiB
HF_NEED_GIB=${HF_NEED_GIB:-1}              # budget; measured peak 0.5 GiB
REBUILD_NEED_GIB=${REBUILD_NEED_GIB:-$((REBUILD_JOBS * REBUILD_JOB_GIB))}   # 16 GiB per job
START_LIMIT_GIB=${START_LIMIT_GIB:-150}    # unreclaimable + budget must stay under this to start a step
RETRY_EVERY=${RETRY_EVERY:-600}
RETRY_HOURS=${RETRY_HOURS:-0}              # 0 = retry until every step has run
RETRY_FOR=${RETRY_FOR:-$((RETRY_HOURS * 3600))}   # seconds (tests set it directly); 0 = no limit
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

gib() { awk -v b="$1" 'BEGIN{printf "%.1f", b/2^30}'; }
CGROUP=${SUPERVISE_CGROUP:-/sys/fs/cgroup}   # overridable for tests only (also read by supervise_job.sh)
mem_unrec() { awk '$1=="anon"||$1=="shmem"{s+=$2} END{print s+0}' "$CGROUP/memory.stat"; }
mem_total() { cat "$CGROUP/memory.current"; }

OK_STEPS=" "     # steps that succeeded in this run (skipped by retries)
BLOCKED=""       # steps of the last pass that had no memory (retried)
run_pass() {   # one pass of the three steps; output appended to $LOG
  local pass=$1 t code
  echo "=== $(date '+%F %T') pass $pass"
  t=$LOG_BASE.p$pass
  BLOCKED=""
  step() {     # step <name> <GiB needed> [env] <command...>
    local name=$1 need=$2 un tot; shift 2
    case "$OK_STEPS" in *" $name "*) echo "--- $(date '+%T') $name: already ok in this run"; return;; esac
    un=$(mem_unrec); tot=$(mem_total)
    if [ $((un + need * 1073741824)) -ge $((START_LIMIT_GIB * 1073741824)) ]; then
      echo "--- $(date '+%T') $name: SKIPPED, no memory budget (unreclaimable $(gib "$un") GiB + budget $need GiB >= $START_LIMIT_GIB GiB; total $(gib "$tot") GiB); retried"
      BLOCKED="$BLOCKED $name"; return
    fi
    echo "--- $(date '+%T') $name: budget ok (unreclaimable $(gib "$un") GiB + $need GiB < $START_LIMIT_GIB GiB)"
    echo "--- $(date '+%T') $name: $*"
    "$@"; code=$?
    [ -f "$t.$name.log" ] && sed 's/^/    /' "$t.$name.log" | grep -v '^    \(Processing topic\|Saved \|Topic \|Submitting\|File processed\|Output folder\|All files\|Processed data\)' | tail -60
    # the supervisor's own verdict is in its .mem file (exit codes 2/3 could also come from the job itself)
    if grep -q "KILLED (container" "$t.$name.mem" 2>/dev/null; then
      s="KILLED because of container memory, will retry ($(grep KILLED "$t.$name.mem" | tail -1))"; BLOCKED="$BLOCKED $name"
    elif grep -q "KILLED" "$t.$name.mem" 2>/dev/null; then s="KILLED by a memory limit ($(grep KILLED "$t.$name.mem" | tail -1))"
    elif grep -q "not starting" "$t.$name.mem" 2>/dev/null; then s="NOT STARTED (container memory above the limits), will retry"; BLOCKED="$BLOCKED $name"
    elif [ "$code" -eq 0 ]; then s=ok; OK_STEPS="$OK_STEPS$name "
    else s="FAILED (exit $code)"; fi
    echo "--- $(date '+%T') $name: $s"
    rm -f "$t.$name.log" "$t.$name.mem"
  }
  # shellcheck disable=SC2086
  step merge-manifest "$MERGE_NEED_GIB" env $LIGHT_LIMITS "$SUP" "$t.merge-manifest" "$PY" "$MERGE_SCRIPT" --data "$DATA" merge-manifest
  # shellcheck disable=SC2086
  step hf-backup "$HF_NEED_GIB" env $LIGHT_LIMITS "$SUP" "$t.hf-backup" "$PY" "$CHECKOUT/src/logger/hf_backup_archive.py" --data "$DATA" $HF_FLAGS
  step rebuild "$REBUILD_NEED_GIB" env SUPERVISE_JOB_GIB="$REBUILD_JOB_GIB" "$SUP" "$t.rebuild" "$PY" "$CHECKOUT/src/logger/rebuild_processed.py" \
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
  while true; do
    while [ -e "$PENDING" ]; do                   # a new trigger: a full pass
      rm -f "$PENDING"; pass=$((pass + 1)); OK_STEPS=" "; deadline=$(( $(date +%s) + RETRY_FOR ))
      run_pass $pass >> "$LOG" 2>&1
    done
    [ -n "$BLOCKED" ] || break
    if [ "$RETRY_FOR" -gt 0 ] && [ "$(date +%s)" -ge "$deadline" ]; then
      echo "=== $(date '+%F %T') giving up after $((RETRY_FOR / 60)) min: still no memory for:$BLOCKED. The next transfer (or ~/rwr_task2/after_transfer.sh) tries again." >> "$LOG"
      break
    fi
    echo "=== $(date '+%F %T') still to run:$BLOCKED; retrying in $((RETRY_EVERY / 60)) min" >> "$LOG"
    sleep "$RETRY_EVERY"
    [ -e "$PENDING" ] && continue                 # a trigger came while waiting: full pass
    pass=$((pass + 1)); run_pass $pass >> "$LOG" 2>&1   # retry: only the steps that are not ok yet
  done
  flock -u 9; exec 9>&-
  [ -e "$PENDING" ] || break                      # a trigger came between the last check and the unlock: go again
done
echo "=== $(date '+%F %T') after_transfer done ($pass pass(es))" >> "$LOG"
