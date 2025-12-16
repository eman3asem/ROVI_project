
import numpy as np
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt

from robot import *


# Global queue container for trajectories
QUEUE = []

def quintic_poly(start_q, end_q, steps):
    # Create trajectory
    trajectories = # Create a quintic trajectory
    # Add each step in that trajectory into the QUEUE
    for step in trajectories:
        QUEUE.append((step, None))

def via_points(via_q, dt=0.002, tacc=1, qdmax=1):
    trajectories = # Create a trajectory with via points
    for step in trajectories:
        QUEUE.append((step, None))
    

def program(d, m):
    # Define our robot object
    robot = UR5robot(data=d, model=m)

    current_q = robot.get_current_q()

    object_list = ["pickup_point_box", "pickup_point_cylinder", "pickup_point_tblock", "pickup_point_cylinder", "pickup_point_box"]

    # Collect a list of target q-poses as your via points. 
    via_q = [current_q]
    q0 = current_q
    for obj in object_list:
        obj_frame = get_mjobj_frame(model=m, data=d, obj_name=obj) # Get body frame
        obj_frame = obj_frame * sm.SE3.Rx(-np.pi)
        obj_desired_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0=q0)[0]
        via_q.append(obj_desired_q)
        q0 = obj_desired_q

    via_q = np.array(via_q)
    
    # Exercise 5.1: VIA POINTS
    via_points(via_q, dt=0.002, tacc=0.1, qdmax=10)

    # Exercise 5.2: QUINTIC POLY
    for i in range(1, len(via_q)):
        quintic_poly(start_q=via_q[i-1], end_q=via_q[i], steps=500)

    # PLOT trajectory profiles
    data = [q_pose[0] for q_pose, _ in QUEUE] # one joint value to plot

    fig, axs = plt.subplots(4)
    fig.suptitle('Trajectory profiles')

    pos = np.array(data)
    vel = np.diff(pos)
    acc = np.diff(vel)
    jerk = np.diff(acc)

    axs[0].plot(pos)
    axs[1].plot(vel)
    axs[2].plot(acc)
    axs[3].plot(jerk)

    # axs[2].set_xlabel("")
    axs[0].set_ylabel("Position")
    axs[1].set_ylabel("Velocity")
    axs[2].set_ylabel("Accelaration")
    axs[3].set_ylabel("Jerk")

    plt.savefig("trajectory_profile.png")

    # The calculated trajectory from the robot is sent to the controller
    return QUEUE
    
    

       
         
