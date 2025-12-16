import numpy as np
import spatialmath as sm

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

from robot import *


# OPTIONAL: Function to check if any joint position is below floor
def is_valid_configuration(robot, q):
    # Calculate forward kinematics for all joints
    Ts =robot.fkine_all(q)    # hint: fkine_all(q) 
    threshold = 0.1 # Padding to account for joint sizes
    # Check each joint position's z-coordinate
    for T in Ts[1:]: # Omit base
        if T.t[2] < threshold:  # z-coordinate is below floor
            return False
    return True

def program(d, m):
    # Define our robot object
    robot = UR5robot(data=d, model=m)
    results_data = []
    vis_traj = []

    # STEP 1: 
    # Make a new scene file, which only contain your target object e.g. the cylinder. Space out the object equally in a grid. 
    # Look at scene.xml for reference. 


    # STEP 2: 
    # Iterate through all the objects in your scene. For each object you can change the angle of the grasp. 
    # Find the frame of your target object:
    # e.g. get_mjobj_frame(model=m, data=d, obj_name=f"cylinder{cylinder_num}")
    for cylinder_num in range(1, 11): # Go through each object

        cylinder_grasp_pose = get_mjobj_frame(model=m, data=d, obj_name=f"cylinder{cylinder_num}")# Get body frame
        success_count = 0 

        for theta in np.linspace(0, 2*np.pi, 100): # Go through each angle
            print(f"Trying: cylinder{cylinder_num} with theta {theta}")

            for _ in range(0, 5): # Tries to get a valid q-pose
                # Calculate the inverse kinematics to check if a valid solution exist
            
                    # Filter solutions based on if joints are in the floor or other criterias
                    if is_valid_configuration(robot.robot_ur5, q_desired):
                        vis_traj.append((q_desired, None)) # qpose and gripper value
                        success_count += 1
                        break

        results_data.append(( f"cylinder{cylinder_num}", success_count))
    

    # STEP 3: 
    # Finally print a heatmap of the results, e.g., rows and columns of the object grid and colored based on valid grasp
    # Ideally you should see that the objects in the middle has a higher reachability than those further away.


    # The calculated trajectory from the robot is sent to the controller
    trajectory = vis_traj
    return trajectory