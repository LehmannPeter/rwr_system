#!/bin/bash
# Run a side job on the Training machine in its own session under hard memory limits, and kill it at once if the
# container's memory gets tight (training runs share the 160 GiB cgroup and must never be the ones that die).
#   supervise_job.sh <log prefix> <command...>          -> <prefix>.log (output), <prefix>.mem (memory peaks)
# - every process of the job: address space capped at $SUPERVISE_JOB_GIB GiB (prlimit --as, default 16), nice 19, no GPU
# - every 5 s: container memory.current, unreclaimable memory (anon + shmem) and the job's total RSS
# - the whole job is killed if unreclaimable > 130 GiB, memory.current > 150 GiB, or the job's RSS > $SUPERVISE_JOB_GIB GiB
# Exit code: the job's, 2 = killed by a limit, 3 = memory already above the limits, not started.
set -u
P=$1; shift
JOB_GIB=${SUPERVISE_JOB_GIB:-16}
LIMIT_UNREC=$((130 * 1024 * 1024 * 1024)); LIMIT_CG=$((150 * 1024 * 1024 * 1024)); LIMIT_JOB=$((JOB_GIB * 1024 * 1024))   # bytes / KiB
CG=/sys/fs/cgroup/memory.current
unrec() { awk '$1=="anon"||$1=="shmem"{s+=$2} END{print s+0}' /sys/fs/cgroup/memory.stat; }
export CUDA_VISIBLE_DEVICES="" PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=${OPENBLAS_NUM_THREADS:-2} OMP_NUM_THREADS=${OMP_NUM_THREADS:-2}
if [ "$(cat $CG)" -gt $LIMIT_CG ] || [ "$(unrec)" -gt $LIMIT_UNREC ]; then echo "$(date -u +%T) memory already above the limits: not starting" | tee -a "$P.mem"; exit 3; fi
setsid nice -n 19 prlimit --as=$((JOB_GIB * 1024 * 1024 * 1024)) -- "$@" > "$P.log" 2>&1 &
PID=$!; sleep 1; SID=$(ps -o sid= -p $PID | tr -d ' ')
peak_cg=0; peak_un=0; peak_job=0; n=0
while kill -0 $PID 2>/dev/null; do
  cg=$(cat $CG); un=$(unrec); job=$(ps -o rss= -s "$SID" 2>/dev/null | awk '{s+=$1} END {print s+0}')
  [ "$cg" -gt "$peak_cg" ] && peak_cg=$cg; [ "$un" -gt "$peak_un" ] && peak_un=$un; [ "$job" -gt "$peak_job" ] && peak_job=$job
  if [ "$cg" -gt $LIMIT_CG ] || [ "$un" -gt $LIMIT_UNREC ] || [ "$job" -gt $LIMIT_JOB ]; then
    echo "$(date -u +%T) KILLED: memory.current $((cg >> 30)) GiB, unreclaimable $((un >> 30)) GiB, job RSS $((job >> 20)) GiB" | tee -a "$P.mem"
    kill -TERM -- -"$SID" 2>/dev/null; sleep 3; kill -KILL -- -"$SID" 2>/dev/null; exit 2
  fi
  n=$((n + 1)); [ $((n % 12)) -eq 0 ] && echo "$(date -u +%T) memory.current $(awk "BEGIN{printf \"%.1f\", $cg/2^30}") GiB, unreclaimable $(awk "BEGIN{printf \"%.1f\", $un/2^30}") GiB, job RSS $(awk "BEGIN{printf \"%.2f\", $job/2^20}") GiB" >> "$P.mem"
  sleep 5
done
wait $PID; code=$?
echo "$(date -u +%T) exit $code; peak memory.current $(awk "BEGIN{printf \"%.1f\", $peak_cg/2^30}") GiB, peak unreclaimable $(awk "BEGIN{printf \"%.1f\", $peak_un/2^30}") GiB, peak job RSS $(awk "BEGIN{printf \"%.2f\", $peak_job/2^20}") GiB (cap $JOB_GIB GiB)" | tee -a "$P.mem"
exit $code
