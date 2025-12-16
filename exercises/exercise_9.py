
import numpy as np
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt
import random

from ompl import base as ob
from ompl import geometric as og

from robot import *

import numpy as np
import roboticstoolbox as rtb
from scipy.optimize import minimize
from spatialmath import SE3, SO3
import mujoco

def random_box_pos(m, d):
    selected_pos = []
    for i in range(1, 7):
        STOP = False
        while not STOP:
            rand_x = random.uniform(0.45+0.25,0.75-0.25)
            rand_y = random.uniform(-0.55+0.25,0.55-0.25)
            if len(selected_pos) == 0:
                STOP = True
                break

            SHOULD_STOP = True   
            for pos in selected_pos:
                dist = np.linalg.norm(np.array([rand_x, rand_y]) - np.array(pos))
                if dist <= 0.15:
                    SHOULD_STOP = False
                    break
            
            if SHOULD_STOP:
                STOP = True

        z = 0.025
        d.joint(f'box{i}').qpos [0:3]= [rand_x, rand_y, z+0.01]
        print(f'box{i}: ', rand_x, rand_y)
        selected_pos.append([rand_x, rand_y])

    mujoco.mj_step(m, d)

    



def program(d, m):

    # Randomize box positions
    random_box_pos(m, d)

    # Define our robot object
    robot = UR5robot(data=d, model=m)

    # EXERCISE: Create/implement a program that stacks the blocks on the dropzone
    # - Localize the blocks and define the grasping frames
    # - Path plan the pick and place trajectory for each block for stacking

    # Example: In the new scene you can get the box frames with "get_mjobj_frame" with each box named box1, box2, ....
    box_frame = get_mjobj_frame(model=m, data=d, obj_name="box1") * sm.SE3.Rx(-np.pi)  # Get body frame


    return robot.queue

    
    

       
         
