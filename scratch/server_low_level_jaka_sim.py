import re
import time
import mujoco, mujoco_viewer, mujoco.viewer
import numpy as np
import onnxruntime as ort
import argparse
from enum import Enum
from collections import deque
from lcm_types.jaka_mimic_obs import jaka_mimic_obs
import lcm
import os


# ====== 四元数 & 旋转工具函数 ======

def get_projected_gravity(quaternion):
    """Get projected gravity from quaternion (wxyz format)"""
    qw = quaternion[0]
    qx = quaternion[1]
    qy = quaternion[2]
    qz = quaternion[3]

    gravity_orientation = np.zeros(3)
    gravity_orientation[0] = 2 * (-qz * qx + qw * qy)
    gravity_orientation[1] = -2 * (qz * qy + qw * qx)
    gravity_orientation[2] = 1 - 2 * (qw * qw + qz * qz)

    return gravity_orientation


def quat_to_rot6d(q):
    """
    将四元数转换为旋转矩阵的前两列并展平。
    Args:
        q: numpy array, shape (4,), order [w, x, y, z]
    Returns:
        numpy array, shape (6,) -> [R00, R01, R10, R11, R20, R21]
    """
    r, i, j, k = q[0], q[1], q[2], q[3]
    two_s = 2.0 / (r * r + i * i + j * j + k * k)

    ii = i * i
    jj = j * j
    kk = k * k
    ij = i * j
    kr = k * r
    ik = i * k
    jr = j * r
    jk = j * k
    ir = i * r

    return np.array([
        1 - two_s * (jj + kk),  # R00
        two_s * (ij - kr),      # R01
        two_s * (ij + kr),      # R10
        1 - two_s * (ii + kk),  # R11
        two_s * (ik - jr),      # R20
        two_s * (jk + ir)       # R21
    ])


def quaternion_conjugate(q):
    """四元数共轭: [w, x, y, z] -> [w, -x, -y, -z]"""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_inv_np(q):
    conj = quaternion_conjugate(q)
    norm_sq = np.sum(q ** 2)
    inv_norm_sq = 1 / max(norm_sq, 1e-9)
    return conj * inv_norm_sq


def subtract_frame_transforms(t01, q01, t02, q02):
    """Subtract transformations between two reference frames."""
    q10 = quat_inv_np(q01)
    if q02 is not None:
        q12 = quat_mul_np(q10, q02)
    else:
        q12 = q10
    return None, q12


def quat_mul_np(q1, q2):
    """四元数乘法: q1 ⊗ q2"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return np.array([w, x, y, z])


def yaw_quat(q):
    w, x, y, z = q
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
    return np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])


def quatToEuler(quat):
    """ 将四元数转换为欧拉角(roll, pitch, yaw)。 """
    eulerVec = np.zeros(3)
    qw, qx, qy, qz = quat
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    eulerVec[0] = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    if np.abs(sinp) >= 1:
        eulerVec[1] = np.copysign(np.pi / 2, sinp)
    else:
        eulerVec[1] = np.arcsin(sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    eulerVec[2] = np.arctan2(siny_cosp, cosy_cosp)
    return eulerVec


def quat_apply_inverse(quat, vec):
    """
    对向量应用四元数的逆旋转 (Inverse Rotation)
    Args:
        quat: 四元数 [w, x, y, z], shape (4,)
        vec: 向量 [x, y, z], shape (3,)
    Returns:
        旋转后的向量, shape (3,)
    """
    xyz = quat[1:]  # shape (3,)
    w = quat[0]  # scalar
    t = np.cross(xyz, vec) * 2
    return vec - w * t + np.cross(xyz, t)


def motion_anchor_ori_b_future(robot_quat, ref_quat, ref_to_robot_quat_init):
    """Future N-step motion anchor orientation in body frame - matching Isaac Lab order"""
    future_anchor_quat_w = quat_mul_np(ref_to_robot_quat_init, ref_quat)
    robot_anchor_quat_w_exp = robot_quat

    pos_b, ori_b = subtract_frame_transforms(
        None,
        robot_anchor_quat_w_exp,
        None,
        future_anchor_quat_w,
    )

    ori_b_flat = quat_to_rot6d(ori_b)
    return ori_b_flat  # [6]


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd


# ====== Jaka Khan Mini 关节配置 ======

class AnchorBody(Enum):
    WAIST_YAW_LINK = 3  # BFS index: base_link(0) -> Left_hip_pitch(1) -> Right_hip_pitch(2) -> waist_yaw(3)


anchor_body = AnchorBody.WAIST_YAW_LINK

# IsaacLab joint names (Jaka Khan Mini ordering)
isaaclab_joint_names = [
    'Left_hip_pitch_joint', 'Right_hip_pitch_joint', 'waist_yaw_joint',
    'Left_hip_roll_joint', 'Right_hip_roll_joint', 'Left_shoulder_pitch_joint', 'Neck_yaw_joint',
    'Right_shoulder_pitch_joint',
    'Left_hip_yaw_joint', 'Right_hip_yaw_joint', 'Left_shoulder_roll_joint', 'Neck_pitch_joint',
    'Right_shoulder_roll_joint', 'Left_knee_joint', 'Right_knee_joint', 'Left_shoulder_yaw_joint',
    'Right_shoulder_yaw_joint',
    'Left_ankle_pitch_joint', 'Right_ankle_pitch_joint', 'Left_elbow_joint', 'Right_elbow_joint',
    'Left_ankle_roll_joint',
    'Right_ankle_roll_joint', 'Left_wrist_roll_joint', 'Right_wrist_roll_joint',
    'Left_wrist_yaw_joint', 'Right_wrist_yaw_joint'
]

# MuJoCo joint names (from Khan_mini_simplified.xml order)
mujoco_joint_names = [
    "Left_hip_pitch_joint", "Left_hip_roll_joint", "Left_hip_yaw_joint", "Left_knee_joint",
    "Left_ankle_pitch_joint", "Left_ankle_roll_joint",
    "Right_hip_pitch_joint", "Right_hip_roll_joint", "Right_hip_yaw_joint", "Right_knee_joint",
    "Right_ankle_pitch_joint", "Right_ankle_roll_joint",
    "waist_yaw_joint",
    "Left_shoulder_pitch_joint", "Left_shoulder_roll_joint", "Left_shoulder_yaw_joint", "Left_elbow_joint",
    "Left_wrist_roll_joint", "Left_wrist_yaw_joint",
    "Right_shoulder_pitch_joint", "Right_shoulder_roll_joint", "Right_shoulder_yaw_joint", "Right_elbow_joint",
    "Right_wrist_roll_joint", "Right_wrist_yaw_joint",
    "Neck_yaw_joint", "Neck_pitch_joint"
]

# Stiffness and Damping (from deploy_mujoco_history_jakamini.py)
stiffness_dict = {
    ".*_hip_pitch_joint": 187.0,
    ".*_hip_roll_joint": 187.0,
    ".*_hip_yaw_joint": 187.0,
    ".*_knee_joint": 187.0,
    ".*_ankle_pitch_joint": 100,
    ".*_ankle_roll_joint": 50,
    "waist_yaw_joint": 187.0,
    ".*_shoulder_pitch_joint": 102.0,
    ".*_shoulder_roll_joint": 102.0,
    ".*_shoulder_yaw_joint": 40.8,
    ".*_elbow_joint": 40.8,
    ".*_wrist_roll_joint": 6.7,
    ".*_wrist_yaw_joint": 6.7,
    "Neck_yaw_joint": 6.7,
    "Neck_pitch_joint": 6.7,
}

damping_dict = {
    ".*_hip_pitch_joint": 18.7,
    ".*_hip_roll_joint": 18.7,
    ".*_hip_yaw_joint": 18.7,
    ".*_knee_joint": 18.7,
    ".*_ankle_pitch_joint": 2,
    ".*_ankle_roll_joint": 0.5,
    "waist_yaw_joint": 18.7,
    ".*_shoulder_pitch_joint": 10.2,
    ".*_shoulder_roll_joint": 10.2,
    ".*_shoulder_yaw_joint": 4.0,
    ".*_elbow_joint": 4.0,
    ".*_wrist_roll_joint": 0.67,
    ".*_wrist_yaw_joint": 0.67,
    "Neck_yaw_joint": 0.67,
    "Neck_pitch_joint": 0.67,
}

scale_dict = {
    ".*": 0.5,
}

# 默认关节位置
joint_pos_config = {
    "Left_shoulder_roll_joint": -1.57,
    "Left_elbow_joint": 1.57,
    "Left_wrist_yaw_joint": 0.3,
    "Right_shoulder_roll_joint": -1.57,
    "Right_elbow_joint": 1.57,
    "Right_wrist_yaw_joint": 0.3,
}


def get_param(joint_name, param_dict):
    """从正则匹配的字典中获取关节参数"""
    for pattern, value in param_dict.items():
        if pattern == ".*":
            return value
        if pattern.startswith(".*"):
            suffix = pattern[3:]
            if joint_name.lower().endswith(suffix.lower()):
                return value
        else:
            if joint_name == pattern:
                return value
    if ".*" in param_dict:
        return param_dict[".*"]
    raise ValueError(f"No value found for joint: {joint_name}")


def get_joint_default_pos(joint_name):
    return joint_pos_config.get(joint_name, 0.0)


# 构建 reindex 映射和参数数组
isaaclab_to_mujoco_reindex = [isaaclab_joint_names.index(name) for name in mujoco_joint_names]
mujoco_to_isaaclab_reindex = [mujoco_joint_names.index(name) for name in isaaclab_joint_names]

kps = np.array([get_param(name, stiffness_dict) for name in mujoco_joint_names], dtype=np.float32)
kds = np.array([get_param(name, damping_dict) for name in mujoco_joint_names], dtype=np.float32)
action_scale = np.array([get_param(name, scale_dict) for name in mujoco_joint_names], dtype=np.float32)
default_angles = np.array([get_joint_default_pos(name) for name in mujoco_joint_names], dtype=np.float32)
default_isaaclab_angles=default_angles[mujoco_to_isaaclab_reindex].copy()  
default_isaaclab_angles[-4:]=[1.5,1.5,0.,0,]
# print(default_isaaclab_angles)
# Jaka 默认 mimic obs (33 维: 2 + 1 + 2 + 1 + 27)
# 默认站立姿态
default_action_mimic = np.concatenate([
    np.array([0., 0.]),       # xy velocity
    np.array([0.7]),         # z position (Jaka Khan Mini height)
    np.array([0., 0.]),       # roll, pitch
    np.array([0.]),           # yaw angular velocity
    default_isaaclab_angles # 27 dof in isaaclab order
])
# default_isaaclab_angles[-4:]=[0,0,0.,0,]


# ====== Fix bug for mujoco_viewer ======
def __fix__add_marker_to_scene(self, marker):
    if self.scn.ngeom >= self.scn.maxgeom:
        raise RuntimeError("Ran out of geoms. maxgeom: %d" % self.scn.maxgeom)

    g = self.scn.geoms[self.scn.ngeom]

    mujoco.mjv_initGeom(
        g,
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=np.zeros(3),
        pos=np.zeros(3),
        mat=np.eye(3).flatten(),
        rgba=np.ones(4)
    )
    g.objtype = mujoco.mjtObj.mjOBJ_UNKNOWN
    g.objid = -1
    g.category = mujoco.mjtCatBit.mjCAT_DECOR
    g.emission = 0
    g.specular = 0.5
    g.shininess = 0.5
    g.reflectance = 0

    for key, value in marker.items():
        if isinstance(value, (int, float, mujoco._enums.mjtGeom)):
            setattr(g, key, value)
        elif isinstance(value, (tuple, list, np.ndarray)):
            attr = getattr(g, key)
            attr[:] = np.asarray(value).reshape(attr.shape)
        elif isinstance(value, str):
            assert key == "label", "Only label is a string in mjtGeom."
            if value is None:
                g.label[0] = 0
            else:
                g.label = value
        elif hasattr(g, key):
            raise ValueError("mjtGeom has attr {} but type {} is invalid".format(key, type(value)))
        else:
            raise ValueError("mjtGeom doesn't have field %s" % key)

    self.scn.ngeom += 1
    return


mujoco_viewer.MujocoViewer._add_marker_to_scene = __fix__add_marker_to_scene


# ====== realtime_controller 类 ======

import threading


class realtime_controller:
    def __init__(self):
        HERE = os.path.dirname(os.path.abspath(__file__))

        # Paths (hardcoded, same style as textop_copy)
        policy_path = os.path.join(HERE, "jaka_data/latest35knew.onnx")
        xml_path = os.path.join(HERE, "jaka_data/Khan_mini_simplified/Khan_mini_simplified_new_bigfeet.xml")

        print(f"Policy path: {policy_path}")
        print(f"XML path: {xml_path}")

        # Load policy
        self.session = ort.InferenceSession(policy_path)
        self.obs_name = self.session.get_inputs()[0].name

        # Simulation parameters
        self.simulation_duration = 1000
        self.simulation_dt = 0.001
        self.control_decimation = 20
        self.num_actions = 27

        # PD parameters (from inline config, no yaml)
        self.kps = kps
        self.kds = kds
        self.action_scale = action_scale
        self.default_dof_pos = default_angles.copy()

        # Reindex mappings
        self.mujoco_to_isaaclab_reindex = mujoco_to_isaaclab_reindex
        self.isaaclab_to_mujoco_reindex = isaaclab_to_mujoco_reindex

        # Frame stack (5 frames of 126-dim obs)
        self.obs_dim = 126  # 33 + 6 + 3 + 3 + 27*3
        self.frame_stack = deque(maxlen=5)
        obs = np.zeros(self.obs_dim, dtype=np.float32)
        for _ in range(5):
            self.frame_stack.append(obs.copy())

        # LCM setup
        self.lc = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        subscription = self.lc.subscribe("JAKA_MIMIC_OBS", self.msg_handler)
        self.mimic_thread = threading.Thread(target=self.mimic_receive, daemon=True)
        self.mimic_thread.start()

        # Context variables
        self.last_action = np.zeros(self.num_actions, dtype=np.float32)
        self.target_dof_pos = self.default_dof_pos.copy()
        self.counter = 0
        self.inner_counter = 5

        # Initialize mimic command with default standing pose
        self.mimic_command = default_action_mimic.copy()
        self.ref_quat = np.array([1., 0., 0., 0.])

        # Load robot model
        self.m = mujoco.MjModel.from_xml_path(xml_path)
        self.d = mujoco.MjData(self.m)
        self.viewer = mujoco_viewer.MujocoViewer(self.m, self.d)
        self.viewer.cam.lookat[:] = np.array([0, 0, 0.55])
        self.viewer.cam.distance = 5.0
        self.viewer.cam.azimuth = 0
        self.viewer.cam.elevation = -30
        self.m.opt.timestep = self.simulation_dt

        # Initialize robot position
        self.d.qpos[7:7 + self.num_actions] = default_isaaclab_angles[isaaclab_to_mujoco_reindex]#self.default_dof_pos
        self.d.qpos[:3] = np.array([0, 0, 0.8])
        self.d.qpos[3:7] = [1., 0., 0., 0.]

        mujoco.mj_step(self.m, self.d)
        self.start = time.time()

        # Setup initial rotation offset
        # Use anchor body (waist_yaw_Link) orientation
        anchor_body_name = "waist_yaw_Link"
        self.anchor_body_name = anchor_body_name
        self.robot_init_quat = yaw_quat(self.d.body(self.anchor_body_name).xquat.copy())
        self.ref_init_quat_inv = quat_inv_np(yaw_quat(self.ref_quat))
        self.ref_to_robot_quat_init = quat_mul_np(self.robot_init_quat, self.ref_init_quat_inv)
        print(self.ref_to_robot_quat_init, "ref_to_robot_quat_init")

        self.last_receive_time = time.time()

    def mimic_receive(self):
        while True:
            self.lc.handle()

    def msg_handler(self, channel, data):
        """
        LCM 回调函数，每次收到 JAKA_MIMIC_OBS 消息时触发
        """
        msg = jaka_mimic_obs.decode(data)
        print("lcm receive:", time.time() - self.last_receive_time)
        self.last_receive_time = time.time()

        # 转换为 numpy 数组
        mimic_command = np.array(msg.mimic_obs)
        self.mimic_command = mimic_command.copy()
        # Reindex the joint positions (last 27 dims) from mujoco order to isaaclab order
      #  self.mimic_command[-27:] = mimic_command[-27:][self.mujoco_to_isaaclab_reindex]
        # print(self.mimic_command, "mimic_command")
        # print(self.ref_quat, "ref_quat")
        # Update ref_quat from LCM message
        ref_quat_plus_flag = np.array(msg.quat_plus_flag)
        if ref_quat_plus_flag[-1] > -0.5:  # not in idle
            self.ref_quat = ref_quat_plus_flag[:4][[3, 0, 1, 2]]  # xyzw -> wxyz

        if ref_quat_plus_flag[-1] > 0.5:  # idle/pause to teleop transition
            # Get anchor body orientation
            robot_quat = self.d.body(self.anchor_body_name).xquat.copy()
            self.robot_init_quat = yaw_quat(robot_quat)
            ref_init_quat_inv = quat_inv_np(yaw_quat(self.ref_quat))
            self.ref_to_robot_quat_init = quat_mul_np(self.robot_init_quat, ref_init_quat_inv)

        self.high_level_send = True

    def compute_observation(self, gravity, ang_vel, dof_pos_rel, dof_vel,
                            motion_command, last_actions, robot_quat,
                            ref_quat, ref_to_robot_quat_init):
        """Compute observation for Jaka Khan Mini (126 dims per frame)"""
        obs = np.zeros(self.obs_dim, dtype=np.float32)

        command = motion_command.copy()
        motion_anchor_ori = motion_anchor_ori_b_future(robot_quat, ref_quat, ref_to_robot_quat_init)

        obs[:33] = command
        obs[33:39] = motion_anchor_ori
        obs[39:42] = gravity
        obs[42:45] = ang_vel * 0.25
        obs[45:72] = dof_pos_rel[self.mujoco_to_isaaclab_reindex]
        obs[72:99] = dof_vel[self.mujoco_to_isaaclab_reindex] * 0.05
        obs[99:126] = last_actions

        return obs

    def run(self):
        while self.viewer.is_alive and time.time() - self.start < self.simulation_duration:
            step_start = time.time()

            # PD control
            tau = pd_control(self.target_dof_pos, self.d.qpos[7:7 + self.num_actions],
                             self.kps, np.zeros_like(self.kds),
                             self.d.qvel[6:6 + self.num_actions], self.kds)
            self.d.ctrl[:] = tau
            mujoco.mj_step(self.m, self.d)

            if self.counter % self.control_decimation == 0:
                t0 = time.time()

                # Get anchor body orientation (waist_yaw_Link)
                anchor_quat = self.d.body(self.anchor_body_name).xquat.copy()
                self.robot_quat = anchor_quat.copy()

                # Compute observation components
                proj_gravity = get_projected_gravity(self.d.qpos[3:7])
                ang_vel = self.d.qvel[3:6]
                dof_pos_rel = self.d.qpos[7:7 + self.num_actions] - self.default_dof_pos
                dof_vel = self.d.qvel[6:6 + self.num_actions]

                obs = self.compute_observation(
                    proj_gravity, ang_vel, dof_pos_rel, dof_vel,
                    self.mimic_command, self.last_action,
                    anchor_quat, self.ref_quat, self.ref_to_robot_quat_init
                )

                # Initialize frame stack on first step
                if self.counter == 0:
                    for _ in range(5):
                        self.frame_stack.append(obs.copy())

                self.frame_stack.append(obs.copy())

                # Feature-major stacking (same as textop_copy)
                stacked_obs = np.concatenate(list(self.frame_stack), axis=0)
                stacked_obs_reshape = stacked_obs.reshape(5, self.obs_dim)

                obs_command = stacked_obs_reshape[:, :33].reshape(-1)
                obs_anchor_ori = stacked_obs_reshape[:, 33:39].reshape(-1)
                obs_gravity = stacked_obs_reshape[:, 39:42].reshape(-1)
                obs_ang_vel = stacked_obs_reshape[:, 42:45].reshape(-1)
                obs_dof_pos = stacked_obs_reshape[:, 45:72].reshape(-1)
                obs_dof_vel = stacked_obs_reshape[:, 72:99].reshape(-1)
                obs_last_action = stacked_obs_reshape[:, 99:126].reshape(-1)

                obs_concat = np.concatenate([
                    obs_command,
                    obs_anchor_ori,
                    obs_gravity,
                    obs_ang_vel,
                    obs_dof_pos,
                    obs_dof_vel,
                    obs_last_action
                ], axis=0)

                # Run policy
                obs_tensor = np.array(obs_concat, dtype=np.float32).reshape(1, -1)
                output = self.session.run(None, {self.obs_name: obs_tensor})
                action = output[0].squeeze()
                action = np.clip(action, -5, 5)
                self.last_action = action.copy()

                # Transform action to target_dof_pos
                self.target_dof_pos = action[self.isaaclab_to_mujoco_reindex] * self.action_scale + self.default_dof_pos

                self.inner_counter += 1
                print(time.time() - t0)

            self.counter += 1

            # Update camera
            self.viewer.cam.lookat = self.d.xpos[1]
            self.viewer.cam.distance = 5
            self.viewer.cam.elevation = -30

            self.viewer.render()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Deploy Jaka Khan Mini MuJoCo simulation")
    args = parser.parse_args()
    print(args)
    controller = realtime_controller()
    controller.run()
