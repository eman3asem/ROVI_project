# pickplace_trapezoid_and_rrt.py
import time
import numpy as np
import spatialmath as sm
import matplotlib.pyplot as plt
import random

from ompl import base as ob
from ompl import geometric as og

from robot import *   # keep your UR5robot, get_mjobj_frame, is_q_valid, etc.

# -------------------------
# Utilities
# -------------------------
class StateValidator:
    def __init__(self, d, m, num_joint):
        self.d = d
        self.m = m
        self.num_joint = num_joint

    def __call__(self, state):
        q_pose = [state[i] for i in range(self.num_joint)]
        return is_q_valid(d=self.d, m=self.m, q=q_pose)


def plan(d, m, start_q, goal_q, timeout=8.0, planner_range=0.6, interpolate_states=200):
    """
    OMPL RRT planner in joint space.
    Returns a list of joint vectors (numpy arrays) if solved, otherwise None.
    """
    num_joint = len(start_q)
    space = ob.RealVectorStateSpace(num_joint)
    bounds = ob.RealVectorBounds(num_joint)
    # set bounds to typical joint limits; adjust to your robot limits if known
    bounds.setLow(-3.2)
    bounds.setHigh(3.2)
    space.setBounds(bounds)

    ss = og.SimpleSetup(space)
    validator = StateValidator(d, m, num_joint)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(validator))

    start = ob.State(space)
    goal = ob.State(space)
    for i in range(num_joint):
        start[i] = float(start_q[i])
        goal[i] = float(goal_q[i])

    ss.setStartAndGoalStates(start, goal)

    planner = og.RRT(ss.getSpaceInformation())
    # set planner range to control expansion step
    planner.setRange(planner_range)
    ss.setPlanner(planner)

    solved = ss.solve(timeout)
    if not solved:
        print("RRT: no solution found in timeout.")
        return None

    path = ss.getSolutionPath()
    # densify path by interpolation
    path.interpolate(interpolate_states)

    solution_trajectory = []
    for i in range(path.getStateCount()):
        st = path.getState(i)
        q_pose = np.array([st[j] for j in range(space.getDimension())], dtype=float)
        solution_trajectory.append(q_pose)
    print(f"RRT: found solution with {len(solution_trajectory)} states.")
    return solution_trajectory


# -------------------------
# Trapezoidal profile
# -------------------------
def trapezoidal_segment_q(q0, qf, segment_time=1.2, steps=200, tb_frac=0.25):
    """
    Single segment joint-space trapezoidal velocity profile.
    q0, qf: numpy arrays of shape (n_joints,)
    segment_time: total duration (sec)
    steps: number of time samples
    tb_frac: fraction of total time used for acceleration (tb = tb_frac * td)
    Returns trajectory of shape (steps, n_joints)
    """
    q0 = np.array(q0, dtype=float)
    qf = np.array(qf, dtype=float)
    td = float(segment_time)
    tb = min(tb_frac * td, td / 2.0)
    t_samples = np.linspace(0.0, td, steps)
    dq = qf - q0
    s = np.sign(dq)
    abs_dq = np.abs(dq)
    # avoid division by zero
    vmax = np.zeros_like(dq)
    with np.errstate(divide='ignore', invalid='ignore'):
        vmax = np.where(abs_dq > 1e-8, abs_dq / (td - tb), 0.0)
    a = vmax / tb  # acceleration magnitude

    traj = np.zeros((steps, q0.size))
    for idx, t in enumerate(t_samples):
        if t < tb:
            # accelerating phase
            q_t = q0 + s * 0.5 * a * (t ** 2)
        elif t <= td - tb:
            # constant velocity phase
            q_t = q0 + s * (vmax * (t - tb / 2.0))
        else:
            # decelerating phase
            tau = td - t
            q_t = qf - s * 0.5 * a * (tau ** 2)
        # handle tiny moves: if abs_dq very small, use linear interpolation fallback
        small_mask = abs_dq <= 1e-6
        if np.any(small_mask):
            # linear interpolation for tiny moves
            frac = t / td if td > 0 else 0.0
            q_t[small_mask] = q0[small_mask] + dq[small_mask] * frac
        traj[idx, :] = q_t
    return traj


def multi_via_trapezoidal(via_q_list, segment_time=1.2, steps_per_segment=200, tb_frac=0.25):
    """
    via_q_list: iterable of joint vectors [q0, q1, ...]
    returns concatenated trajectory (N_samples, n_joints)
    """
    all_segs = []
    for i in range(len(via_q_list) - 1):
        seg = trapezoidal_segment_q(via_q_list[i], via_q_list[i + 1],
                                    segment_time=segment_time, steps=steps_per_segment, tb_frac=tb_frac)
        # avoid duplicate endpoints except last
        if i < len(via_q_list) - 2:
            seg = seg[:-1]
        all_segs.append(seg)
    if len(all_segs) == 0:
        return np.zeros((0, len(via_q_list[0])))
    return np.vstack(all_segs)


# -------------------------
# Helpers to build via points (task -> joint)
# -------------------------
def build_via_q_for_object(robot, model, data, pickup_body_name, drop_body_name,
                           pick_height_rel=0.00, pre_pick_z=0.12, lift_z=0.20, pre_place_z=0.20):
    """
    Creates a list of joint configs [home, pre_pick, pick, lift, pre_place, place]
    Uses ik_LM and refines using ik_NR if available.
    Returns list of numpy arrays or raises Exception if IK fails.
    """
    q_home = np.array(robot.get_current_q())

    # get frames from mujoco bodies; multiply by rotation if needed
    obj_frame = get_mjobj_frame(model=model, data=data, obj_name=pickup_body_name) * sm.SE3.Rx(-np.pi)
    drop_frame = get_mjobj_frame(model=model, data=data, obj_name=drop_body_name) * sm.SE3.Rx(-np.pi)

    via = [q_home]

    # pre-pick above object
    T_prepick = obj_frame * sm.SE3.Tz(+pre_pick_z)
    q_pre_pick = robot.robot_ur5.ik_LM(Tep=T_prepick, q0=q_home)[0]
    via.append(np.array(q_pre_pick))

    # pick pose (slightly lower)
    T_pick = obj_frame * sm.SE3.Tz(pick_height_rel)
    q_pick = robot.robot_ur5.ik_LM(Tep=T_pick, q0=q_pre_pick)[0]
    via.append(np.array(q_pick))

    # lift
    T_lift = obj_frame * sm.SE3.Tz(+lift_z)
    q_lift = robot.robot_ur5.ik_LM(Tep=T_lift, q0=q_pick)[0]
    via.append(np.array(q_lift))

    # pre-place
    T_preplace = drop_frame * sm.SE3.Tz(+pre_place_z)
    q_pre_place = robot.robot_ur5.ik_LM(Tep=T_preplace, q0=q_lift)[0]
    via.append(np.array(q_pre_place))

    # place
    T_place = drop_frame * sm.SE3.Tz(+0.05)
    q_place = robot.robot_ur5.ik_LM(Tep=T_place, q0=q_pre_place)[0]
    via.append(np.array(q_place))

    return via


# -------------------------
# Execution helpers
# -------------------------
def execute_traj_open_loop(robot, traj, dt=0.02, gripper_actions=None):
    """
    Execute joint trajectory open loop.
    traj: (N,6) array
    gripper_actions: dict mapping sample indices to gripper values {idx: 0 or 255}
    This uses robot.move_j to step between samples. Adjust to your robot API if needed.
    """
    if gripper_actions is None:
        gripper_actions = {}

    for i, q in enumerate(traj):
        # call gripper if at this sample index
        if i in gripper_actions:
            robot.set_gripper(gripper_actions[i])
        # we use move_j single step: small t so robot controller interpolates smoothly
        # convert dt to ms for your API if it expects ms; here we set t small.
        try:
            robot.move_j(start_q=robot.queue[-1][0], end_q=list(q), t=max(1, int(dt * 1000)))
        except Exception as e:
            # fallback: directly set queue item if available
            robot.move_j(start_q=robot.get_current_q(), end_q=list(q), t=max(1, int(dt * 1000)))


def flatten_rrt_segments(rrt_segments):
    """
    rrt_segments: list of arrays (each array is (N_i, n_joints))
    returns a single stacked array without duplicate endpoints
    """
    out = []
    for i, seg in enumerate(rrt_segments):
        if i < len(rrt_segments) - 1:
            out.append(seg[:-1])
        else:
            out.append(seg)
    return np.vstack(out)


# -------------------------
# Main program
# -------------------------
def program(d, m):
    robot = UR5robot(data=d, model=m)

    # names from your scene xml
    obj_triplets = [
        ("pickup_point_cylinder", "drop_point_cylinder", "cylinder"),
        ("pickup_point_tblock", "drop_point_tblock", "t_block"),
        ("pickup_point_box", "drop_point_box", "box"),
    ]

    # initial safe poses you used
    pickzone_init_guess = [-0.224, -1.347, 1.417, -1.64, -1.57, -1.79]
    dropzone_init_guess = [0.224, -2.89, 0.55, 0.769, -1.57, -1.35]
    # move to an initial safe pose
    robot.move_j(start_q=robot.get_current_q(), end_q=pickzone_init_guess, t=1000)

    results = []

    for pickup_name, drop_name, obj_name in obj_triplets:
        print(f"\n=== Processing object: {obj_name} ===")

        # Build via joint points
        try:
            via_q = build_via_q_for_object(robot, m, d, pickup_name, drop_name,
                                           pick_height_rel=0.00, pre_pick_z=0.12, lift_z=0.20, pre_place_z=0.20)
        except Exception as e:
            print("IK failed building via points:", e)
            continue

        # locate indices for gripper actions in trapezoid execution
        # estimate where pick and place occur: pick is at segment 2 start, place at segment 5 start
        # For robust mapping to sample indices, build trapezoid then map.
        seg_time = 1.2
        steps_per_seg = 220

        # ---- Method A: Point-to-point trapezoidal ----
        t0 = time.time()
        traj_trap = multi_via_trapezoidal(via_q, segment_time=seg_time, steps_per_segment=steps_per_seg, tb_frac=0.25)
        t_trap = time.time() - t0
        # compute sample indices for gripper ops:
        # pick happens at end of segment 1 -> segment indices: 0..N-1 segments
        # our via list: [home, pre_pick, pick, lift, pre_place, place] -> pick index corresponds to the last sample of segment 1 start?
        # simpler: find the sample nearest to the exact via points
        def find_closest_sample(traj, q_target):
            diffs = np.linalg.norm(traj - q_target.reshape(1, -1), axis=1)
            return int(np.argmin(diffs))

        pick_sample = find_closest_sample(traj_trap, np.array(via_q[2]))
        place_sample = find_closest_sample(traj_trap, np.array(via_q[5]))

        gripper_actions = {pick_sample: 255, place_sample: 0}

        print(f"Executing trapezoidal trajectory, planning/compute time {t_trap:.3f}s, samples {traj_trap.shape[0]}")
        execute_traj_open_loop(robot, traj_trap, dt=0.02, gripper_actions=gripper_actions)

        # After execution, evaluate if placed successfully by inspecting Mujoco state? For now, log success
        # Move up to pre-place to be safe
        robot.move_j(start_q=robot.queue[-1][0], end_q=via_q[-1], t=400)

        # ---- Method B: RRT per segment ----
        print("Running RRT per segment for same via points")
        rrt_all_segments = []
        rrt_failed = False
        planning_times = []
        for i in range(len(via_q) - 1):
            s_q = via_q[i]
            g_q = via_q[i + 1]
            t0 = time.time()
            sol = plan(d=d, m=m, start_q=s_q, goal_q=g_q, timeout=6.0, planner_range=0.6, interpolate_states=120)
            t_pl = time.time() - t0
            planning_times.append(t_pl)
            if sol is None:
                print(f"RRT failed between segment {i} and {i+1}.")
                rrt_failed = True
                break
            # sol is list of q arrays; convert to numpy
            rrt_all_segments.append(np.vstack(sol))
            print(f"Segment {i}->{i+1} planned, {len(sol)} states, plan time {t_pl:.3f}s")

        if not rrt_failed:
            # flatten segments
            traj_rrt = flatten_rrt_segments(rrt_all_segments)
            # select sample indices for gripper actions similar to trapezoid
            pick_sample_rrt = find_closest_sample(traj_rrt, np.array(via_q[2]))
            place_sample_rrt = find_closest_sample(traj_rrt, np.array(via_q[5]))
            gripper_actions_rrt = {pick_sample_rrt: 255, place_sample_rrt: 0}
            print(f"Executing RRT trajectory with {traj_rrt.shape[0]} samples. planning times: {planning_times}")
            execute_traj_open_loop(robot, traj_rrt, dt=0.02, gripper_actions=gripper_actions_rrt)
        else:
            print("RRT planning failed for at least one segment. Skipping RRT exec for this object.")

        results.append({
            "object": obj_name,
            "trapezoid_samples": traj_trap.shape[0],
            "trapezoid_compute_time": t_trap,
            "rrt_failed": rrt_failed,
            "rrt_planning_times": planning_times if not rrt_failed else None
        })

        # Reset robot to a known safe pose before next object
        robot.move_j(start_q=robot.queue[-1][0], end_q=pickzone_init_guess, t=1000)

    print("\n=== Summary ===")
    for r in results:
        print(r)

    return robot.queue


# If you directly run this file, you might want to wrap program(d,m) invocation in your mujoco runner.
# Example usage pattern (in your mujoco runner):
# from mujoco_runner import load_model_and_data
# m, d = load_model_and_data("scene_obstacles.xml")
# program(d, m)
