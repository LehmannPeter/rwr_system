"""Stand-in for the policy launch when testing APE's evaluation of policy runs on a machine without ROS (tests/test_eval.py).
It takes the inference node's commands on its terminal instead of the ROS topic and prints the node's log lines."""
import sys, time
say = lambda level, text: print(f"[model_inference_node.py-1] [{level}] [{time.time():.3f}] [policy_publisher]: {text}", flush=True)
print("About to load policy", flush=True)
on = False
for line in sys.stdin:
    c = line.strip().lower()
    if c == "start" and not on:
        on = True; say("INFO", "Started new log file: inference_logs/mock-run/mock.hdf5"); say("INFO", "Policy started.")
    elif c == "stop" and on:
        on = False; say("INFO", "Policy stopped.")
