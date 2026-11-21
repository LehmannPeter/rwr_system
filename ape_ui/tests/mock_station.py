"""Fake station for testing APE without the robot. Run ONLY on a test ROS domain
(ROS_DOMAIN_ID other than 7). Provides arm pose, glove pose, robot state and the
retargeter, hand and logger services. Env: MOCK_MODE = robot_mode to report (2 = ready)."""
import json, os, sys, time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import array
import math
import numpy as np
from std_srvs.srv import SetBool, Trigger

if os.environ.get("ROS_DOMAIN_ID", "0") == "7":
    sys.exit("Refusing to run the mock station on the real robot's ROS domain (7).")
OUT = os.environ.get("MOCK_DIR", "/tmp")
rclpy.init(); n = rclpy.create_node("mock_station"); g = ReentrantCallbackGroup()
st = {"follow": False, "log": [], "cmd": None}
arm = n.create_publisher(PoseStamped, "/right/arm/end_effector_pose", 10)
glove = n.create_publisher(PoseStamped, "/ingress/right/wrist", 10)
hand = n.create_publisher(Float32MultiArray, "/right/hand/policy_output", 10)
fs = n.create_publisher(FrankaState, "/paco/franka_robot_state_broadcaster/robot_state", 10)
hand_meas = n.create_publisher(Float32MultiArray, "/right/hand/joint_position_from_motor", 10)
hand_cur = n.create_publisher(Float32MultiArray, "/right/hand/motor_current", 10)
hand_temp = n.create_publisher(Float32MultiArray, "/right/hand/motor_temperature", 10)
from rclpy.qos import QoSProfile, ReliabilityPolicy
cams = {c: n.create_publisher(Image, f"/{c}/color", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)) for c in ("oakd_front_view", "oakd_wrist_view")}
base_img = np.zeros((540, 960, 3), np.uint8); base_img[..., 0] = np.linspace(0, 255, 960, dtype=np.uint8)[None, :]

def camera_tick():
    if os.environ.get("MOCK_CAMERAS", "1") != "1": return
    t = time.time()
    for i, (name, pub) in enumerate(cams.items()):
        img = base_img.copy(); x = int((t * 200 + i * 300) % 900); img[200:340, x:x + 60] = (255, 220, 0) if i == 0 else (0, 200, 255)
        m = Image(); m.height, m.width, m.encoding, m.step = 540, 960, "rgb8", 2880; m._data = array.array('B', img.tobytes()); pub.publish(m)

def hand_tick():
    t = time.time(); close = 0.5 + 0.5 * math.sin(t)
    meas = [0.0] * 17
    for i in (3, 4, 6, 7, 9, 10, 12, 13, 15, 16): meas[i] = math.radians(-20 + 110 * close)
    hand_meas.publish(Float32MultiArray(data=meas))
    hand_cur.publish(Float32MultiArray(data=[100.0 + 10 * i for i in range(17)]))
    hand_temp.publish(Float32MultiArray(data=[38.0 + 2 * i for i in range(17)]))

n.create_timer(1 / float(os.environ.get('MOCK_CAMERA_HZ', '10')), camera_tick, callback_group=ReentrantCallbackGroup()); n.create_timer(0.1, hand_tick)

from scipy.spatial.transform import Rotation as _R
arm_rot = {"R": _R.identity(), "cmd": None, "p": None}

def follow_cmd(m):
    """MOCK_ARM_FOLLOW=1: the fake arm follows commands but stops MOCK_DEADBAND degrees short in
    orientation, like the real arm's soft orientation control against friction."""
    o, p = m.pose.orientation, m.pose.position
    arm_rot["cmd"] = _R.from_quat([o.x, o.y, o.z, o.w]); arm_rot["p"] = (p.x, p.y, p.z)

def tick():
    if os.environ.get("MOCK_ARM_FOLLOW") == "1" and arm_rot["cmd"] is not None:
        err = (arm_rot["cmd"] * arm_rot["R"].inv()).as_rotvec(); ang = np.linalg.norm(err); band = math.radians(float(os.environ.get("MOCK_DEADBAND", "7")))
        if ang > band: arm_rot["R"] = _R.from_rotvec(err * min(1.0, 0.3 * (1 - band / ang))) * arm_rot["R"]
        a = PoseStamped(); a.pose.position.x, a.pose.position.y, a.pose.position.z = arm_rot["p"]
        q = arm_rot["R"].as_quat(); a.pose.orientation.x, a.pose.orientation.y, a.pose.orientation.z, a.pose.orientation.w = map(float, q); arm.publish(a)
        p = PoseStamped(); p.pose.orientation.w = 1.0; glove.publish(p)
        if st["follow"]: hand.publish(Float32MultiArray(data=[0.0] * 17))
        return
    a = PoseStamped(); a.pose.position.x, a.pose.position.y, a.pose.position.z = (0.4, 0.0, 0.3) if os.environ.get("MOCK_ARM_CIRCLE") != "1" else (0.45 + 0.12 * math.cos(time.time()), 0.15 * math.sin(time.time()), 0.3); a.pose.orientation.w = 1.0; arm.publish(a)
    p = PoseStamped(); p.pose.position.x = 0.10 if os.path.exists(OUT + "/moved") else 0.0; p.pose.position.z = (time.time() % 1) * 1e-4; p.pose.orientation.w = 1.0; glove.publish(p)
    if st["follow"]: hand.publish(Float32MultiArray(data=[0.0] * 17))

def slow():
    m = FrankaState(); m.robot_mode = int(os.environ.get("MOCK_MODE", "2")); m.control_command_success_rate = 1.0; fs.publish(m)
    json.dump({"cmd": st["cmd"], "log": st["log"]}, open(OUT + "/mock_state.json", "w"))

n.create_timer(0.02, tick); n.create_timer(0.1, slow)
def on_cmd(m):
    st["cmd"] = [round(m.pose.position.x, 4), round(m.pose.position.y, 4), round(m.pose.position.z, 4)]
    follow_cmd(m)
n.create_subscription(PoseStamped, "/right/arm/end_effector_pose_cmd", on_cmd, 10)
paco = None
if os.environ.get("MOCK_CONTROLLER_LISTENS") == "1":   # APE counts listeners in the /paco namespace as "the controller is listening"
    paco = rclpy.create_node("mock_controller", namespace="/paco"); paco.create_subscription(PoseStamped, "/right/arm/end_effector_pose_cmd", lambda m: None, 10)

def setb(name):
    def cb(req, res):
        st["log"].append(f"{name}={req.data}")
        if name == "follow": st["follow"] = req.data
        res.success = True; res.message = "ok"; return res
    return cb

def trig(name):
    def cb(req, res):
        st["log"].append(name)
        if name == "save":
            time.sleep(2.0)
            d = os.environ.get("MOCK_DATASET_DIR")
            if d:
                os.makedirs(d, exist_ok=True); open(f"{d}/{time.strftime('%Y%m%d_%H%M%S')}.h5", "wb").write(b"x" * 1000)
        res.success = True; return res
    return cb

n.create_service(SetBool, "/right/hand/set_torque_enabled", setb("torque"), callback_group=g)
if os.environ.get("MOCK_POLICY_STATION") != "1":   # a policy launch has the hand and the cameras, but no retargeter and no logger
    n.create_service(SetBool, "/right/retargeter/set_enabled", setb("follow"), callback_group=g)
    for k in ("start", "stop", "save"):
        n.create_service(Trigger, f"/demo_logger/{k}_recording", trig(k), callback_group=g)
ex = MultiThreadedExecutor(4); ex.add_node(n)
if paco: ex.add_node(paco)
ex.spin()
