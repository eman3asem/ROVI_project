
import numpy as np
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt

from robot import *


# Global queue container for trajectories
QUEUE = []

def linear_q_interpolation(start_q, end_q, steps):
    q0 = start_q
    qf = end_q
    t0 = 0
    tf = 1

    for t in np.linspace(t0, tf, steps):
        ....
        QUEUE.append((q_t, None))

def parabolic_q_interpolation(start_q, end_q, steps):
    q0 = start_q
    qf = end_q
    t0 = 0
    tf = 1
    td = # duration of travel
    tb = # blend time

    # Calculate required acceleration for the blend
    ddqb =  # constant acceleration during blend

    for t in np.linspace(t0, tf, steps):
        ...
        QUEUE.append((q_t, None))


def program(d, m):
    # Define our robot object
    robot = UR5robot(data=d, model=m)

    current_frame = robot.get_current_tcp()
    current_q = robot.get_current_q()

    object_list = ["pickup_point_box", "pickup_point_cylinder", "pickup_point_tblock", "pickup_point_cylinder", "pickup_point_box"]
    q0 = current_q
    t0 = current_frame
    for obj in object_list:

        # # Define grasping frames for object: box
        obj_frame = get_mjobj_frame(model=m, data=d, obj_name=obj) # Get body frame
        obj_frame = obj_frame * sm.SE3.Rx(-np.pi)

        obj_desired_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0=q0)[0]

        # STEP 1: Implement your own linear-q interpolation
        linear_q_interpolation(start_q=q0, end_q=obj_desired_q, steps=300)

        # STEP 2: Implement your own parabolic-q interpolation
        parabolic_q_interpolation(start_q=q0, end_q=obj_desired_q, steps=300)
        
        q0 = obj_desired_q
        t0 = obj_frame

    data = []
    for q_pose, gripper_value in QUEUE:
        # extract q_pose to data for plotting
        data.append(q_pose[0]) # y-value

    # Plot trajectory profile, position, velocity, accelaration
    ...

    plt.savefig("trajectory_profile.png")

    # The calculated trajectory from the robot is sent to the controller
    return QUEUE
    

       
         
