#!/bin/bash
# Run a side job on the Training machine in its own session under hard memory limits, and kill it at once if the
# container's memory gets tight (training runs share the 160 GiB cgroup and must never be the ones that die).
#   supervise_job.sh <log prefix> <command...>          -> <prefix>.log (output), <prefix>.mem (memory peaks)
# - every process of the job: address space capped at $SUPERVISE_JOB_GIB GiB (prlimit --as, default 16), nice 19, no GPU
# - every 5 s: container memory.current, unreclaimable memory (anon + shmem) and the job's total RSS
# - the whole job is killed if unreclaimable > 130 GiB, memory.current > 150 GiB, or the job's RSS > $SUPERVISE_JOB_GIB GiB
# Exit code: the job's, 2 = killed by a limit, 3 = memory already above the limits, not started.
# The container limits can be raised for small jobs only (after_transfer.sh does this for the merge and the
# Hugging Face backup): $SUPERVISE_UNREC_GIB (default 130), $SUPERVISE_TOTAL_GIB (default 150; 0 = no limit on the
# total, which is mostly file cache), and the job's RSS limit $SUPERVISE_RSS_GIB (default: the address-space cap
# $SUPERVISE_JOB_GIB).
set -u
P=$1; shift
JOB_GIB=${SUPERVISE_JOB_GIB:-16}
RSS_GIB=${SUPERVISE_RSS_GIB:-$JOB_GIB}
UNREC_GIB=${SUPERVISE_UNREC_GIB:-130}; TOTAL_GIB=${SUPERVISE_TOTAL_GIB:-150}
LIMIT_UNREC=$((UNREC_GIB * 1024 * 1024 * 1024)); LIMIT_CG=$((TOTAL_GIB * 1024 * 1024 * 1024)); LIMIT_JOB=$((RSS_GIB * 1024 * 1024))   # bytes / KiB
[ "$TOTAL_GIB" -gt 0 ] || LIMIT_CG=$((1 << 62))   # 0: the total (memory.current) is not limited
CGROUP=${SUPERVISE_CGROUP:-/sys/fs/cgroup}   # overridable for tests only
CG=$CGROUP/memory.current
unrec() { awk '$1=="anon"||$1=="shmem"{s+=$2} END{print s+0}' "$CGROUP/memory.stat"; }
export CUDA_VISIBLE_DEVICES="" PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=${OPENBLAS_NUM_THREADS:-2} OMP_NUM_THREADS=${OMP_NUM_THREADS:-2}
if [ "$(cat $CG)" -gt $LIMIT_CG ] || [ "$(unrec)" -gt $LIMIT_UNREC ]; then echo "$(date -u +%T) memory already above the limits: not starting" | tee -a "$P.mem"; exit 3; fi
setsid nice -n 19 prlimit --as=$((JOB_GIB * 1024 * 1024 * 1024)) -- "$@" > "$P.log" 2>&1 &
PID=$!; sleep 1; SID=$(ps -o sid= -p $PID | tr -d ' ')
peak_cg=0; peak_un=0; peak_job=0; n=0
while kill -0 $PID 2>/dev/null; do
  cg=$(cat $CG); un=$(unrec); job=$(ps -o rss= -s "$SID" 2>/dev/null | awk '{s+=$1} END {print s+0}')
  [ "$cg" -gt "$peak_cg" ] && peak_cg=$cg; [ "$un" -gt "$peak_un" ] && peak_un=$un; [ "$job" -gt "$peak_job" ] && peak_job=$job
  if [ "$cg" -gt $LIMIT_CG ] || [ "$un" -gt $LIMIT_UNREC ] || [ "$job" -gt $LIMIT_JOB ]; then
    if [ "$job" -gt $LIMIT_JOB ]; then why="the job passed its own cap"; else why="container memory"; fi
    echo "$(date -u +%T) KILLED ($why): memory.current $((cg >> 30)) GiB, unreclaimable $((un >> 30)) GiB, job RSS $((job >> 20)) GiB" | tee -a "$P.mem"
    kill -TERM -- -"$SID" 2>/dev/null; sleep 3; kill -KILL -- -"$SID" 2>/dev/null; exit 2
  fi
  n=$((n + 1)); [ $((n % 12)) -eq 0 ] && echo "$(date -u +%T) memory.current $(awk "BEGIN{printf \"%.1f\", $cg/2^30}") GiB, unreclaimable $(awk "BEGIN{printf \"%.1f\", $un/2^30}") GiB, job RSS $(awk "BEGIN{printf \"%.2f\", $job/2^20}") GiB" >> "$P.mem"
  sleep 5
done
wait $PID; code=$?
echo "$(date -u +%T) exit $code; peak memory.current $(awk "BEGIN{printf \"%.1f\", $peak_cg/2^30}") GiB, peak unreclaimable $(awk "BEGIN{printf \"%.1f\", $peak_un/2^30}") GiB, peak job RSS $(awk "BEGIN{printf \"%.2f\", $peak_job/2^20}") GiB (limits: job $RSS_GIB GiB, unreclaimable $UNREC_GIB GiB, total $([ "$TOTAL_GIB" -gt 0 ] && echo "$TOTAL_GIB GiB" || echo "not limited"))" | tee -a "$P.mem"
exit $code
