"""Write a small raw recording in the logger's format, for testing processing and playback."""
import sys, h5py, numpy as np
out, seconds = sys.argv[1], 4.0
t0 = 1_800_000_000_000_000_000
with h5py.File(out, "w") as h:
    def stream(topic, hz, fn):
        g = h.create_group(topic)
        for k in range(int(seconds * hz)):
            g.create_dataset(str(t0 + int(k * 1e9 / hz)), data=fn(k / hz))
    pose = lambda t: np.array([0.4 + 0.05 * np.sin(t), 0.1 * np.cos(t), 0.3, 0, 0, 0, 1.0])
    stream("/right/arm/end_effector_pose", 50, pose)
    stream("/right/arm/end_effector_pose_cmd", 50, pose)
    stream("/right/hand/joint_position_from_motor", 50, lambda t: np.full(17, 0.3 * np.sin(t), np.float32))
    stream("/right/hand/policy_output", 50, lambda t: np.full(17, 0.3 * np.sin(t), np.float32))
    for i, cam in enumerate(("oakd_front_view", "oakd_wrist_view")):
        def img(t, i=i):
            a = np.zeros((90, 160, 3), np.uint8); a[..., i] = 120; x = int(t * 30) % 140; a[30:60, x:x + 20] = 255; return a
        stream(f"/{cam}/color", 15, img)
        g = h.create_group(f"/{cam}/intrinsics"); g.create_dataset(str(t0), data=np.eye(3))
print("wrote", out)
