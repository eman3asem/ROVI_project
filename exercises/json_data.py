import json
import numpy as np

def export_trajectory_data(trajectory, robot, filename="graphs/trajectory_data.json"):
    """
    Processes the trajectory list and saves Q, DK, Velocity, and Accel to JSON.
    trajectory: list of (q_array, gripper_state) or just q_array
    """
    data_log = []
    dt = 3 / (800 - 1) # tf=3 t0=0 steps= 800
    
    # Pre-calculate DK and format trajectory
    qs = []
    for entry in trajectory:
        # Handle different return formats from your RRT vs P2P
        q = entry[0] if isinstance(entry, tuple) else entry
        qs.append(q.tolist() if isinstance(q, np.ndarray) else q)

    qs = np.array(qs)
    
    # 1. Calculate Velocities (dq/dt)
    vbox = np.diff(qs, axis=0) / dt
    # 2. Calculate Accelerations (dv/dt)
    abox = np.diff(vbox, axis=0) / dt

    for i in range(len(qs)):
        # Calculate Direct Kinematics (End Effector Position)
        # Assuming robot.robot_ur5.fkine returns an SE3 object
        T = robot.robot_ur5.fkine(qs[i])
        ee_pos = T.t.tolist() # [x, y, z]

        node = {
            "step": i,
            "q": qs[i].tolist(),
            "ee_pos": ee_pos,
            "v": vbox[i-1].tolist() if i > 0 else [0]*6,
            "a": abox[i-2].tolist() if i > 1 else [0]*6
        }
        data_log.append(node)

    with open(filename, 'w') as f:
        json.dump(data_log, f, indent=4)
    print(f"Data successfully exported to {filename}")
