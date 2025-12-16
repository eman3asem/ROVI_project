import numpy as np
import spatialmath as sm
import roboticstoolbox as rtb
import mujoco as mj

def _make_tf(R, t):
    """
        Combine translation and orientation
    """
    # TODO: add checks for dimensions
    return sm.SE3.Rt(R=R, t=t, check=False)

def get_mjobj_frame(model, data, obj_name):
    """
    Get the frame of a specific object in the MuJoCo simulation.
    """
    obj_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, obj_name)
    if obj_id == -1:
        raise ValueError(f"Object '{obj_name}' not found")
    # Get the object's position and orientation
    obj_pos = data.xpos[obj_id]
    obj_rot = data.xmat[obj_id]
    return _make_tf(R=obj_rot.reshape(3,3), t=obj_pos)

def ur_get_qpos(data, model):
    # Define the joint names (adjust based on your UR model)
    UR_JOINT_NAMES = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]
    # Get joint IDs
    joint_ids = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name) for name in UR_JOINT_NAMES]
    # Get qpos indices for the joints
    # (since qpos is a flat array, we need to know where each joint's value is stored)
    qpos_indices = []
    for jid in joint_ids:
        # Each joint's position is stored at data.qpos[model.jnt_qposadr[jid]]
        qpos_indices.append(model.jnt_qposadr[jid])
    q_values = data.qpos[qpos_indices]
    return q_values

def ur_ctrl_qpos(data, q_desired):
     assert len(q_desired) == 6, "Expected 6 joint positions for UR robot"
     for i in range(len(q_desired)):
            data.ctrl[i] = q_desired[i]  # Assumes actuators are position-controlled
  

TOOL_LENGTH = 0.15
# TODO: Implement DH robot for an UR5e using RTB
robot_ur5 = rtb.DHRobot([
        rtb.RevoluteDH(d=0.1625, alpha=np.pi / 2.0, qlim=(-np.pi, np.pi)),              # J1
        rtb.RevoluteDH(a=-0.425, qlim=(-np.pi, np.pi)),                                 # J2
        rtb.RevoluteDH(a=-0.3922, qlim=(-np.pi, np.pi)),                                # J3
        rtb.RevoluteDH(d=0.1333, alpha=np.pi / 2.0, qlim=(-np.pi, np.pi)),              # J4
        rtb.RevoluteDH(d=0.0997, alpha=-np.pi / 2.0, qlim=(-np.pi, np.pi)),             # J5
        rtb.RevoluteDH(d=0.0996 + TOOL_LENGTH, qlim=(-np.pi, np.pi)),                   # J6
        ], name="UR5", base=sm.SE3.Rz(-np.pi))


def program(d, m):
    # Find the frame of an object:
    target_point_1_frame = get_mjobj_frame(model=m, data=d, obj_name="pickup_point_box") # Get body frame
    target_point_1_frame = target_point_1_frame * sm.SE3.Rx(-np.pi) # Rotate 180 to align with TCP frame


    q_desired = robot_ur5.ik_LM(target_point_1_frame, q0=ur_get_qpos(d, m))[0]
    print("My FK frame: ", robot_ur5.fkine(q_desired), "mj frame: ", target_point_1_frame)

    ur_ctrl_qpos(data=d, q_desired=q_desired)


       
         
