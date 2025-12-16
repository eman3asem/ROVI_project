
import numpy as np
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt
import random

from ompl import base as ob
from ompl import geometric as og

from robot import *


class StateValidator:
    # This is mujoco specific, so I have implemented this for you
    def __init__(self, d, m, num_joint):
        self.d = d
        self.m = m
        self.num_joint = num_joint
    
    def __call__(self, state):
        print("isStateValid - state: ", state)
        q_pose = [state[i] for i in range(self.num_joint)]
        return is_q_valid(d=self.d, m=self.m, q=q_pose) 

def plan(d, m, start_q, goal_q):
    num_joint = 6
    
    space = ob.RealVectorStateSpace(num_joint) # Create a joint-space vector instead of a 2D space as in the example
    #TODO: Create joint bounds
   
    #TODO: Create SimpleSetup

    # Create a state validity checker object
    validator = StateValidator(d, m, num_joint)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(validator))
    
    
    # Set start and goal states
    start = ob.State(space)
    goal = ob.State(space)

    # TODO: Set start and goal poses
    
    ss.setStartAndGoalStates(start, goal)

    # ===== EXPLICITLY SET PLANNER =====
    # TODO: Create/set your planner type

    ss.setPlanner(planner)
    
    # Solve the problem
    solved = ss.solve(10.0)
    
    if solved:
        # TODO: Extract solution path
    
        # TODO: Return the q-poses to solution_trajectory
        
        return solution_trajectory


def program(d, m):
    # Define our robot object
    robot = UR5robot(data=d, model=m)
    
    start_q = robot.get_current_q()
    # Define grasping frames for object: box
    obj_frame = get_mjobj_frame(model=m, data=d, obj_name="drop_point_box") # Get body frame
    # TODO: Plan a collision free trajectory from start_q to a q-pose representating the pose in obj_frame
    

    # Define grasping frames for object: box
    obj_frame = get_mjobj_frame(model=m, data=d, obj_name="pickup_point_box") # Get body frame
    # TODO: Plan a collision free trajectory from the new current q-pose to a q-pose representating the pose in obj_frame

    return robot.queue

    
    

       
         
