#!/bin/env python3

import json
import os
import numpy as np
import time
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Point, TransformStamped, Vector3, Quaternion, PoseStamped
from tf2_ros import TransformBroadcaster

from scipy.spatial.transform import Rotation, Slerp
from std_srvs.srv import SetBool, Trigger
from faive_system.src.common.transformation_utils import rigid_transform, complete_rotation_matrix
from faive_system.src.common.enums import TeleopState, HandChirality

class RokokoCoilDemo(Node):
    def __init__(self, chirality=HandChirality.RIGHT):
        super().__init__(f"{chirality}_rokoko_coil_demo")
        self.declare_parameter("enable_left", False)      # global parameter for cross embodiment
        self.declare_parameter("enable_right", True)      # global parameter for cross embodiment
        self.declare_parameter("rokoko_source_frame_name", "wrist")      # global parameter for cross embodiment
        # interactive=False keeps the node purely service driven: no stdin prompts, so another
        # node (e.g. the HIL policy node) can own the terminal and call pause/resume itself.
        self.declare_parameter("interactive", True)
        # Arm position scale used by /franka_teleop/<chirality>/set_accurate_mode (SetBool):
        # true -> accurate scale, false -> 1.0. Both are limited to 0.2 .. 1.0.
        self.declare_parameter("accurate_pos_mapping_scale", 0.6)
        # Station specific start pose for move2init, saved by
        # /franka_teleop/<chirality>/set_start_pose_here. Without the file, the built-in pose is used.
        self.declare_parameter("start_pose_file", "~/.ros/teleop_start_pose_{chirality}.json")
        # Fixed "center" pose for /franka_teleop/<chirality>/move2center (world frame, metres; palm down).
        self.declare_parameter("center_position", [0.45, 0.0, 0.40])
        # Optional station correction for the center pose: {"yaw_deg": <rotation about the
        # vertical axis, positive = anticlockwise seen from above>}. Read at every move.
        self.declare_parameter("center_adjust_file", "~/.ros/teleop_center_adjust_{chirality}.json")
        # Optional station correction for how the Rokoko coil frame is turned against the robot
        # base: {"yaw_deg": <rotation about the vertical axis, positive = anticlockwise seen from
        # above>}. 180 reverses sideways and forward/back together. Read at start and at every
        # resume (never while teleop runs); without the file the built-in coil orientation is used.
        self.declare_parameter("coil_adjust_file", "~/.ros/teleop_coil_adjust_{chirality}.json")
        # move2init and move2center travel smoothly at these speeds instead of jumping.
        self.declare_parameter("move_speed", 0.10)       # m/s
        self.declare_parameter("move_rot_speed", 0.5)    # rad/s
        # After a smooth move the arm tends to stop several degrees short in orientation
        # (soft orientation control against joint friction). The settle phase nudges the
        # commanded orientation by the measured error until the hand is within tolerance.
        self.declare_parameter("move_settle_time", 4.0)      # s, longest settle phase; 0 switches it off
        self.declare_parameter("move_settle_gain", 1.5)      # 1/s
        self.declare_parameter("move_settle_max_deg", 15.0)  # largest total correction
        self.declare_parameter("move_settle_tol_deg", 1.0)

        self.enable_left = self.get_parameter("enable_left").value
        self.enable_right = self.get_parameter("enable_right").value
        self.rokoko_source_frame_name = self.get_parameter("rokoko_source_frame_name").value
        self.interactive = self.get_parameter("interactive").value

        enabled = False
        if chirality=="left" and self.enable_left:
            enabled = True
        elif chirality=="right" and self.enable_right:
            enabled = True
        self.enabled = enabled
        if not enabled:
            return

        self.X_bCP_G = None  # rigid_transform from coil pro to glove wrist
        self.X_bCP_G_init = (
            None  # Initial rigid_transform from coil pro to glove wrist
        )

        self.X_W_bCP = rigid_transform( # from Coil pro in world frame. Translation doesn't matter
            [0.5, 0.0, 0.0],
            [0.7071, 0.0, 0.0, 0.7071]
        )

        self.chirality = chirality
        self.R_W_bCP_builtin = self.X_W_bCP[:3,:3].copy()
        self.coil_yaw_deg = 0.0
        self.apply_coil_yaw()

        self.X_W_fEE_d = None
        self.X_W_fEE_init = None
        self.X_W_fEE = None

        self.rokoko_pose_sub = self.create_subscription(
            PoseStamped, f"/ingress/{self.chirality}/{self.rokoko_source_frame_name}", self.rokoko_pose_callback, 10
        )

        # publishes to arm/end_effector_pose_cmd
        self.arm_publisher = self.create_publisher(
            PoseStamped, f"/{self.chirality}/arm/end_effector_pose_cmd", 10
        )

        self.tf_publisher = TransformBroadcaster(self)

        self.arm_subscriber = self.create_subscription(
            PoseStamped, f"/{self.chirality}/arm/end_effector_pose", self.arm_pose_callback, 10
        )

        # the teleop is modeled as a state machine
        # 0: unready
        # 1: paused
        # 2: running
        # Transition from unready to paused is to move the robot to default position
        # Transition from paused to running requires the calibration of the hand
        self._state = TeleopState.UNREADY
        self._move = None  # smooth move in progress (only while paused)
        self.DT = 0.02
        self.timer = self.create_timer(self.DT, self.main_loop_cb)

        # Create Pause and Resume services
        self.move2init_service = self.create_service(
            Trigger, 
            f"/franka_teleop/{self.chirality}/move2init",   # service name
            self.move2init_callback
        )
        self.pause_service = self.create_service(
            Trigger, 
            f"/franka_teleop/{self.chirality}/pause",   # service name
            self.pause_callback
        )
        self.resume_service = self.create_service(
            Trigger,
            f"/franka_teleop/{self.chirality}/resume",  # service name
            self.resume_callback
        )

        self.start_pose_file = os.path.expanduser(
            str(self.get_parameter("start_pose_file").value).replace("{chirality}", str(self.chirality))
        )
        self.move2center_service = self.create_service(
            Trigger,
            f"/franka_teleop/{self.chirality}/move2center",
            self.move2center_callback
        )
        self.set_start_pose_service = self.create_service(
            Trigger,
            f"/franka_teleop/{self.chirality}/set_start_pose_here",
            self.set_start_pose_here_callback
        )
        self.accurate_mode_service = self.create_service(
            SetBool,
            f"/franka_teleop/{self.chirality}/set_accurate_mode",
            self.set_accurate_mode_callback
        )

        self._calc_translation_func = self.calc_translation_ScaledPosMapping
        # self._calc_translation_func = self.calc_translation_ClampV
        # self._calc_translation_func = self.calc_translation_VJoyStick

    def move2init_callback(self, request, response):
        if self._state == TeleopState.UNREADY: 
            response.success = False
            print("Waiting for robot pose")
            return response
        if self._state == TeleopState.RUNNING: 
            response.success = False
            print("Cannot move to init pose while teleop is running")
            return response
        
        if self.chirality == HandChirality.LEFT:
            start_target_pos = np.array([0.40, 0.36, 0.2])
            start_target_rot = complete_rotation_matrix(x=[1, 0, 0], z=[0, 0, -1])
        else:
            start_target_pos = np.array([0.45, -0.36, 0.2])
            start_target_rot = complete_rotation_matrix(x=[1, 0, 0], z=[0, 0, -1])

        start_target = rigid_transform(start_target_pos, Rotation.from_matrix(start_target_rot).as_quat())
        saved = self.load_start_pose()
        if saved is not None:
            start_target = saved
        duration = self.start_smooth_move(start_target)
        self.X_W_fEE_init = deepcopy(start_target)
        self.Xr_W_fEE_init = np.eye(4)
        self.Xr_W_fEE_init[:3,:3] = self.X_W_fEE_init[:3,:3]
        self.Xt_W_fEE_init = rigid_transform(self.X_W_fEE_init[:3,3], [0,0,0,1])

        response.success = True
        response.message = f"{duration:.1f}"
        return response

    def move2center_callback(self, request, response):
        """Move smoothly to the fixed center pose (parameter center_position, palm down)."""
        if self._state != TeleopState.PAUSED or self.X_W_fEE is None:
            response.success = False
            response.message = "Only possible while teleop is paused and the robot pose is known"
            return response
        pos = [float(v) for v in self.get_parameter("center_position").value]
        rot = complete_rotation_matrix(x=[1, 0, 0], z=[0, 0, -1])
        yaw_deg = 0.0
        adjust_file = os.path.expanduser(
            str(self.get_parameter("center_adjust_file").value).replace("{chirality}", str(self.chirality))
        )
        try:
            with open(adjust_file) as f:
                yaw_deg = float(json.load(f).get("yaw_deg", 0.0))
            if not np.isfinite(yaw_deg) or abs(yaw_deg) > 45.0:
                raise ValueError("yaw_deg must be within +-45 degrees")
        except FileNotFoundError:
            pass
        except Exception as exc:
            self.get_logger().warn(f"Ignoring {adjust_file}: {exc}")
            yaw_deg = 0.0
        rot = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix() @ rot
        duration = self.start_smooth_move(rigid_transform(pos, Rotation.from_matrix(rot).as_quat()))
        response.success = True
        response.message = f"{duration:.1f}"
        return response

    def start_smooth_move(self, goal):
        """Begin a smooth move from the measured pose to goal (4x4). Returns its duration in s.
        The targets are published by the main loop while paused; pause or resume cancels it."""
        start = deepcopy(self.X_W_fEE)
        dist = float(np.linalg.norm(goal[:3, 3] - start[:3, 3]))
        rots = Rotation.from_matrix(np.stack([start[:3, :3], goal[:3, :3]]))
        angle = float((rots[0].inv() * rots[1]).magnitude())
        speed = max(float(self.get_parameter("move_speed").value), 0.01)
        rot_speed = max(float(self.get_parameter("move_rot_speed").value), 0.05)
        duration = max(dist / speed, angle / rot_speed, 1.0)
        settle = max(float(self.get_parameter("move_settle_time").value), 0.0)
        self._move = {
            "start": start[:3, 3].copy(), "goal": goal[:3, 3].copy(), "slerp": Slerp([0.0, 1.0], rots),
            "t0": time.monotonic(), "T": duration,
            "goal_rot": rots[1], "settle": settle, "corr": np.zeros(3), "ok_time": 0.0,
        }
        self.get_logger().info(f"Smooth move: {dist*100:.0f} cm, {np.degrees(angle):.0f} deg, {duration:.1f} s (+ up to {settle:.0f} s settling)")
        return duration + settle

    def step_smooth_move(self):
        m = self._move
        if m is None:
            return
        elapsed = time.monotonic() - m["t0"]
        X = np.eye(4)
        if elapsed < m["T"]:
            a = elapsed / m["T"]
            s = a * a * (3.0 - 2.0 * a)  # ease in and out
            X[:3, 3] = (1.0 - s) * m["start"] + s * m["goal"]
            X[:3, :3] = m["slerp"](s).as_matrix()
            self.publish_target_pose(X)
            return
        # Settle phase: hold the goal position and correct the commanded orientation by the
        # measured orientation error, so the hand ends where it should instead of stopping short.
        X[:3, 3] = m["goal"]
        if m["settle"] <= 0.0 or self.X_W_fEE is None:
            X[:3, :3] = m["goal_rot"].as_matrix()
            self.publish_target_pose(X)
            self._move = None
            return
        err = (m["goal_rot"] * Rotation.from_matrix(self.X_W_fEE[:3, :3]).inv()).as_rotvec()  # measured -> goal, world axes
        err_deg = float(np.degrees(np.linalg.norm(err)))
        m["ok_time"] = m["ok_time"] + self.DT if err_deg < float(self.get_parameter("move_settle_tol_deg").value) else 0.0
        corr = m["corr"] + float(self.get_parameter("move_settle_gain").value) * err * self.DT
        limit = np.radians(float(self.get_parameter("move_settle_max_deg").value))
        norm = float(np.linalg.norm(corr))
        if norm > limit:
            corr = corr * (limit / norm)
        m["corr"] = corr
        X[:3, :3] = (Rotation.from_rotvec(corr) * m["goal_rot"]).as_matrix()
        self.publish_target_pose(X)
        if m["ok_time"] >= 0.4 or elapsed > m["T"] + m["settle"]:
            self.get_logger().info(
                f"Move finished: orientation {err_deg:.1f} deg from the goal, correction used {np.degrees(np.linalg.norm(corr)):.1f} deg"
            )
            self._move = None
    
    def pause_callback(self, request, response):
        if self._state == TeleopState.UNREADY:
            response.success = False
            return response
        self._move = None  # pausing also stops a smooth move: the arm holds its last target
        self._state = TeleopState.PAUSED
        response.success = True
        return response

    def resume_callback(self, request, response):
        if self._state == TeleopState.UNREADY:
            response.success = False
            return response

        if self.get_hand_pose() is None:
            # Resuming without a glove pose cannot be calibrated. Refuse instead of raising,
            # so a caller driving this service (HIL takeover) just stays paused.
            self.get_logger().warn("Cannot resume: no glove pose received yet")
            response.success = False
            return response

        self._move = None
        self.init_glove_pose()
        self._state = TeleopState.RUNNING
        response.success = True
        return response

    def load_coil_yaw(self):
        """Yaw correction of the coil frame in degrees from the station file, 0.0 without a valid file."""
        adjust_file = os.path.expanduser(
            str(self.get_parameter("coil_adjust_file").value).replace("{chirality}", str(self.chirality))
        )
        try:
            with open(adjust_file) as f:
                yaw_deg = float(json.load(f).get("yaw_deg", 0.0))
            if not np.isfinite(yaw_deg) or abs(yaw_deg) > 180.0:
                raise ValueError("yaw_deg must be within +-180 degrees")
            return yaw_deg
        except FileNotFoundError:
            return 0.0
        except Exception as exc:
            self.get_logger().warn(f"Ignoring {adjust_file}: {exc}")
            return 0.0

    def apply_coil_yaw(self):
        """Turn the coil frame by the station file's yaw. Called at start and at every resume,
        so a change takes effect at the next resume and never while teleop runs."""
        yaw_deg = self.load_coil_yaw()
        X_W_bCP = self.X_W_bCP.copy()
        X_W_bCP[:3,:3] = Rotation.from_euler("z", yaw_deg, degrees=True).as_matrix() @ self.R_W_bCP_builtin
        self.X_W_bCP = X_W_bCP
        if yaw_deg != self.coil_yaw_deg:
            self.get_logger().info(f"Coil frame turned by {yaw_deg:.1f} deg about the vertical axis")
        self.coil_yaw_deg = yaw_deg

    def load_start_pose(self):
        """Saved start pose as a 4x4 transform, or None if no valid file exists."""
        try:
            with open(self.start_pose_file) as f:
                data = json.load(f)
            pos = [float(v) for v in data["position"]]
            quat = [float(v) for v in data["quat_xyzw"]]
            if len(pos) != 3 or len(quat) != 4 or not np.all(np.isfinite(pos + quat)):
                raise ValueError("bad start pose")
            return rigid_transform(pos, quat)
        except FileNotFoundError:
            return None
        except Exception as exc:
            self.get_logger().warn(f"Ignoring start pose file {self.start_pose_file}: {exc}")
            return None

    def set_start_pose_here_callback(self, request, response):
        """Save the arm's current pose as the start pose used by move2init."""
        if self.X_W_fEE is None:
            response.success = False
            response.message = "No robot pose received yet"
            return response
        X = deepcopy(self.X_W_fEE)
        data = {
            "position": [float(v) for v in X[:3, 3]],
            "quat_xyzw": [float(v) for v in Rotation.from_matrix(X[:3, :3]).as_quat()],
        }
        try:
            os.makedirs(os.path.dirname(self.start_pose_file), exist_ok=True)
            with open(self.start_pose_file, "w") as f:
                json.dump(data, f, indent=2)
        except OSError as exc:
            response.success = False
            response.message = f"Could not write {self.start_pose_file}: {exc}"
            return response
        self.get_logger().info(f"Start pose saved to {self.start_pose_file}: {data['position']}")
        response.success = True
        response.message = self.start_pose_file
        return response

    def set_accurate_mode_callback(self, request, response):
        """Switch the arm position scale. Only while teleop is not running, so the target
        never jumps: the next resume re-captures the glove and arm poses."""
        if self._state == TeleopState.RUNNING:
            response.success = False
            response.message = "Pause teleop before changing the motion scale"
            return response
        scale = float(self.get_parameter("accurate_pos_mapping_scale").value) if request.data else 1.0
        self._pos_mapping_scale = min(max(scale, 0.2), 1.0)
        self.get_logger().info(f"Arm position scale set to {self._pos_mapping_scale:.2f}")
        response.success = True
        response.message = f"{self._pos_mapping_scale:.2f}"
        return response

    def arm_pose_callback(self, msg: PoseStamped):
        orientation = msg.pose.orientation
        position = msg.pose.position
        # Get the pose of the wrist
        self.X_W_fEE = rigid_transform(
            [position.x, position.y, position.z],
            [orientation.x, orientation.y, orientation.z, orientation.w],
        )

    def rokoko_pose_callback(self, msg: PoseStamped):
        orientation = msg.pose.orientation
        position = msg.pose.position
        # Get the pose of the wrist
        self.X_bCP_G = rigid_transform(
            [position.x, position.y, position.z],
            [orientation.x, orientation.y, orientation.z, orientation.w],
        )

        if self.X_W_fEE_init is not None:
            self.publish_tf(f"{self.chirality}_robot_init", self.X_W_fEE_init, f"{self.chirality}_panda_link0")

        self.publish_tf(f"{self.chirality}_coil_pro", self.X_W_bCP, f"{self.chirality}_panda_link0")
        if self.X_bCP_G_init is not None:
            self.publish_tf(f"{self.chirality}_glove_init", self.X_bCP_G_init, "coil_pro")
        self.publish_tf(f"{self.chirality}_glove_wrist", self.X_bCP_G, "coil_pro")

    def get_hand_pose(self):
        return deepcopy(self.X_bCP_G)

    def main_loop_cb(self):
        """
        Main loop for the rokoko_coil_demo node. Modeled as a state machine
        """
        if self._state == TeleopState.UNREADY: # waiting robot pose
            self.X_W_fEE_init = deepcopy(self.X_W_fEE)
            if self.X_W_fEE_init is not None:
                self.Xr_W_fEE_init = np.eye(4)
                self.Xr_W_fEE_init[:3,:3] = self.X_W_fEE_init[:3,:3]
                self.Xt_W_fEE_init = rigid_transform(self.X_W_fEE_init[:3,3], [0,0,0,1])
                self._state = TeleopState.PAUSED
        elif self._state == TeleopState.PAUSED: # paused
            self.step_smooth_move()
        elif self._state == TeleopState.RUNNING:
            X_W_fEE_d = self.compute_robot_target()
            self.publish_target_pose(X_W_fEE_d)


    def compute_robot_target(self):
        if self.X_bCP_G_init is None or self.X_W_fEE_init is None:
            self.get_logger().warn("Initial poses not set yet")
            return self.X_W_fEE

        # Compute the pose of the glove in world frame
        X_W_G = self.X_W_bCP @ self.X_bCP_G
        Xr_W_G = np.eye(4)
        Xr_W_G[:3,:3] = X_W_G[:3,:3]

        Xt_W_fEE_d = self._calc_translation_func(X_W_G)
        # Compute the target pose for the robot
        X_W_fEE_d = (
            Xt_W_fEE_d
            @ Xr_W_G
            @ self.Xr_W_G_init_fEE_init
        )

        return X_W_fEE_d

    def publish_target_pose(self, X_W_fEE_d):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        x, y, z = X_W_fEE_d[:3, 3]
        msg.pose.position = Point(x=x, y=y, z=z)
        quat = Rotation.from_matrix(X_W_fEE_d[:3,:3]).as_quat()# x, y, z, w
        msg.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )
        self.arm_publisher.publish(msg)

    def init_glove_pose(self):

        self.apply_coil_yaw()
        self.X_W_fEE_init = deepcopy(self.X_W_fEE)
        if self.X_W_fEE_init is not None:
            self.Xr_W_fEE_init = np.eye(4)
            self.Xr_W_fEE_init[:3,:3] = self.X_W_fEE_init[:3,:3]
            self.Xt_W_fEE_init = rigid_transform(self.X_W_fEE_init[:3,3], [0,0,0,1])

        self.X_bCP_G_init = self.get_hand_pose()
        self.Xr_bCP_G_init = np.eye(4)
        self.Xr_bCP_G_init[:3,:3] = self.X_bCP_G_init[:3,:3]
        
        self.X_W_G_init = self.X_W_bCP @ self.X_bCP_G_init

        self.Xr_W_G_init = np.eye(4)
        self.Xr_W_G_init[:3,:3] = self.X_W_G_init[:3,:3]
        
        self.Xt_W_G_init = rigid_transform(self.X_W_G_init[:3,3], [0,0,0,1])
        self.Xr_W_G_init_inv = np.linalg.inv(self.Xr_W_G_init)

        self.Xr_W_G_init_fEE_init = np.linalg.inv(self.Xr_W_G_init) @ self.Xr_W_fEE_init

        self.get_logger().info("Calibration complete")

        
    def publish_tf(self, frame_id, X, parent_frame_id):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent_frame_id
        msg.child_frame_id = frame_id
        msg.transform.translation = Vector3(
            x=X[0,3], y=X[1,3], z=X[2,3]
        )
        quat = Rotation.from_matrix(X[:3,:3]).as_quat() # x, y, z, w
        msg.transform.rotation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )
        self.tf_publisher.sendTransform(msg)


    ### Implementations of different ways to calculate the translation cmd
    def calc_translation_VJoyStick(self, X_W_G):
        """
        assume the glove is a virtual joystick. Translate the position difference into velocity
        """
        if not hasattr(self, "_vjoy_gain"):
            self._vjoy_gain = 5.0
        if not hasattr(self, "_vjoy_deadzone"):
            self._vjoy_deadzone = 0.05
        
        t_W_G_delta = X_W_G[:3,3] - self.Xt_W_G_init[:3,3]
        if np.linalg.norm(t_W_G_delta) < self._vjoy_deadzone:
            t_W_G_delta *= 0
        t_W_G_delta *= self._vjoy_gain
        Xt_W_fEE_d = rigid_transform(t_W_G_delta* self.DT + self.X_W_fEE[:3,3],[0,0,0,1])
        return Xt_W_fEE_d

    ### Implementations of different ways to calculate the translation cmd
    def calc_translation_ClampV(self, X_W_G):
        """
        Calculate the difference between the previous step, clamp the velocity
        """
        if not hasattr(self, "_pdiff_max"):
            self._pdiff_max = 0.01
        if not hasattr(self, "X_W_G_prev"):
            self.X_W_G_prev = deepcopy(X_W_G)

        t_W_G_delta = X_W_G[:3,3] - self.X_W_G_prev[:3,3]
        self.X_W_G_prev = deepcopy(X_W_G)
        # print(np.linalg.norm(t_W_G_delta))
        # clamp the diff
        if np.linalg.norm(t_W_G_delta) > self._pdiff_max:
            t_W_G_delta = self._pdiff_max * t_W_G_delta / np.linalg.norm(t_W_G_delta)

        Xt_W_fEE_d = rigid_transform(t_W_G_delta + self.X_W_fEE[:3,3],[0,0,0,1])
        return Xt_W_fEE_d

    def calc_translation_ScaledPosMapping(self, X_W_G):
        """
        The robot follows the difference in the position of the glove, but the velocity is clamped
        """
        if not hasattr(self, "_pos_mapping_scale"):
            self._pos_mapping_scale = 1.0

        Xt_W_G_delta = np.linalg.inv(self.Xt_W_G_init) @ rigid_transform(
            X_W_G[:3,3],
            [0,0,0,1]
        )
        Xt_W_G_delta[:3,3] *= self._pos_mapping_scale
        Xt_W_fEE_d =  Xt_W_G_delta @ self.Xt_W_fEE_init
        return Xt_W_fEE_d


def main(args=None):
    rclpy.init(args=args)
    nodes = [RokokoCoilDemo(chirality=side) for side in HandChirality]
    active_nodes = []
    for node in nodes:
        if node.enabled:
            active_nodes.append(node)
        else:
            node.destroy_node()
    if not active_nodes:
        rclpy.shutdown()
        return

    import threading

    executor = rclpy.executors.MultiThreadedExecutor()
    for node in active_nodes:
        executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except (ExternalShutdownException, KeyboardInterrupt):
            pass

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()

    interactive = any(node.interactive for node in active_nodes)
    if not interactive:
        # Service driven mode (HIL): the robot is initialized by whoever owns it, and
        # pause/resume come from /franka_teleop/<chirality>/{pause,resume}. Just serve.
        print("Teleop running in non-interactive mode, waiting for pause/resume service calls")
        try:
            spin_thread.join()
        except KeyboardInterrupt:
            pass
        executor.shutdown()
        for node in active_nodes:
            node.destroy_node()
        rclpy.try_shutdown()
        return

    for node in active_nodes:
        while rclpy.ok() and node.X_W_fEE is None:
            print(f"Waiting for {node.chirality} robot pose...")
            time.sleep(1)

    input("Press enter to move the robot to init pose")
    for node in active_nodes:
        node.move2init_callback(Trigger.Request(), Trigger.Response())

    input("Press enter to calibrate the gloves")
    for rcd in active_nodes:
        while rcd.get_hand_pose() is None:
            print(f"Waiting for {rcd.chirality} glove pose...")
            time.sleep(0.2)
        result = rcd.resume_callback(Trigger.Request(), Trigger.Response())

    keep_running = True
    try:
        while keep_running:
            input("Press enter to pause")

            for rcd in active_nodes:
                rcd.pause_callback(Trigger.Request(), Trigger.Response())

            input("Press enter to resume to accurate mode")
            for rcd in active_nodes:
                # # use the v joy stick
                # rcd._calc_translation_func = rcd.calc_translation_VJoyStick
        

                # use the pose scale
                rcd._calc_translation_func = rcd.calc_translation_ScaledPosMapping 
                rcd._pos_mapping_scale = 0.6
        
            for rcd in active_nodes:
                rcd.resume_callback(Trigger.Request(), Trigger.Response())

            input("Press enter to pause")
            for rcd in active_nodes:
                rcd.pause_callback(Trigger.Request(), Trigger.Response())

            input("Press enter to resume to normal mode")
            for rcd in active_nodes:
                rcd._calc_translation_func = rcd.calc_translation_ScaledPosMapping
                rcd._pos_mapping_scale = 1.0

            for rcd in active_nodes:
                rcd.resume_callback(Trigger.Request(), Trigger.Response())


    except KeyboardInterrupt:
        keep_running = False

    executor.shutdown()
    spin_thread.join()
    for node in active_nodes:
        node.destroy_node()
    rclpy.try_shutdown()

if __name__ == "__main__":
    main()
