"""Stand-in for `ros2 launch experiments run_policy_inference.launch.py` when testing APE without
the robot (test ROS domain only). Takes the launch's own arguments (preview:=true|false, ...),
answers /model_inference_command and prints the log lines the real inference node prints.
Env: MOCK_LOAD_S = seconds the "policy" takes to load, MOCK_PAUSE = 1 prints a pause warning,
MOCK_PREDICT_MS = the prediction time it reports on /policy_inference/timing."""
import os, sys, time
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, String

if os.environ.get("ROS_DOMAIN_ID", "0") == "7":
    sys.exit("Refusing to run the mock inference node on the real robot's ROS domain (7).")
args = dict(a.split(":=", 1) for a in sys.argv[1:] if ":=" in a)
preview = args.get("preview", "false") == "true"
print(f"[mock] arguments: {args}", flush=True)
print("About to load policy", flush=True)
time.sleep(float(os.environ.get("MOCK_LOAD_S", "2")))
rclpy.init(); n = rclpy.create_node("policy_publisher")
st = {"on": False, "init_until": None, "steps": 0}
out_arm = n.create_publisher(PoseStamped, "/policy_inference/actions_arm", 10)
out_hand = n.create_publisher(Float32MultiArray, "/policy_inference/actions_hand", 10)
out_timing = n.create_publisher(Float32MultiArray, "/policy_inference/timing", 10)
ctl_arm = None if preview else n.create_publisher(PoseStamped, "/right/arm/end_effector_pose_cmd", 1)
ctl_hand = None if preview else n.create_publisher(Float32MultiArray, "/right/hand/policy_output", 1)
say = lambda level, text: print(f"[model_inference_node.py-1] [{level}] [{time.time():.3f}] [policy_publisher]: {text}", flush=True)

def command(msg):
    c = msg.data.strip().lower()
    if c == "start" and not st["on"]:
        if st["init_until"]: return say("WARN", "Arm initialization is pending or failed. Finish it, retry init_arm, or reset.")
        st["on"], st["steps"] = True, 0; say("INFO", "Policy started.")
    elif c == "stop":
        if st["init_until"]: st["init_until"] = None; say("ERROR", "Arm initialization stopped: stopped by operator.")
        if st["on"]: st["on"] = False; say("INFO", "Policy stopped.")
    elif c == "reset": say("INFO", "Policy state reset.")
    elif c == "init_arm":
        if preview or args.get("allow_arm_init") != "true" or st["on"]: return say("WARN", "init_arm requires live mode, allow_arm_init, and an idle policy.")
        st["init_until"] = time.time() + 2.0; say("INFO", "Arm initialization started.")

def step():
    if st["init_until"] and time.time() > st["init_until"]:
        st["init_until"] = None; say("INFO", "Arm reached init pose. Ready for start.")
    if not st["on"]: return
    st["steps"] += 1
    if os.environ.get("MOCK_PAUSE") == "1" and st["steps"] == 5: say("WARN", "Policy output paused: qpos_arm age 0.312 s")
    a = PoseStamped(); a.header.frame_id = "world"; a.pose.position.x, a.pose.position.y, a.pose.position.z = 0.42, 0.01, 0.31; a.pose.orientation.w = 1.0
    h = Float32MultiArray(data=[0.2] * 17)
    out_arm.publish(a); out_hand.publish(h)
    ran = st["steps"] % 5 == 1   # as with update_every: 5. MOCK_PREDICT_MS = time of a prediction (default 60)
    ms = float(os.environ.get("MOCK_PREDICT_MS", "60")) if ran else 1.0
    out_timing.publish(Float32MultiArray(data=[ms, ms + 40.0, 200.0, float(ran), float(ms + 40.0 <= 200.0)]))
    if ctl_arm: ctl_arm.publish(a); ctl_hand.publish(h)

n.create_subscription(String, "/model_inference_command", command, 10)
n.create_timer(0.1, step)
try: rclpy.spin(n)
except KeyboardInterrupt: pass
