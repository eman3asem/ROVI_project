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
        # print("isStateValid - state: ", state)
        q_pose = [state[i] for i in range(self.num_joint)]
        return is_q_valid(d=self.d, m=self.m, q=q_pose) 
    
def plan(d, m, start_q, goal_q):
    num_joint = 6
    
    space = ob.RealVectorStateSpace(num_joint) # Create a joint-space vector instead of a 2D space as in the example
    # Create joint bounds
    bounds = ob.RealVectorBounds(num_joint)
    bounds.setLow(-3.14)
    bounds.setHigh(3.14)
    space.setBounds(bounds)
    
    # Create SimpleSetup
    ss = og.SimpleSetup(space)
    validator = StateValidator(d, m, num_joint)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(validator))
    
    # Set start and goal states
    start = ob.State(space)
    goal = ob.State(space)
    
    # Set specific joint values for start and goal
    for i in range(num_joint):
        start[i] = start_q[i] # initial joint angles
        goal[i] = goal_q[i]  # goal joint angles (~90 degrees)
    
    ss.setStartAndGoalStates(start, goal)

    planner = og.RRT(ss.getSpaceInformation())

    ss.setPlanner(planner)
    
    # Solve the problem
    solved = ss.solve(10.0)
    
    if solved:
        # Get the solution path
        path = ss.getSolutionPath()
        print("Found a Solution!")
        # Print basic information about the path
        print(f"Path length: {path.length()}")
        print(f"Number of states: {path.getStateCount()}")

        solution_trajectory = []
        for i in range(path.getStateCount()):
            state = path.getState(i)
            q_pose = [state[i] for i in range(space.getDimension())]
            print(f"State {i}: {q_pose}")
            solution_trajectory.append(np.array(q_pose))

        return solution_trajectory

def program(d, m):
    # Define our robot object
    robot = UR5robot(data=d, model=m)

    ## pick and place objects: box, t_block, cylinder
    box_frame = get_mjobj_frame(model=m, data=d, obj_name="box") * sm.SE3.Rx(-np.pi)  # Get body frame
    drop_point_box_frame = get_mjobj_frame(model=m, data=d, obj_name="drop_point_box") * sm.SE3.Rx(-np.pi)  # Get body frame

    t_block_frame = get_mjobj_frame(model=m, data=d, obj_name="t_block") * sm.SE3.Rx(-np.pi)  # Get body frame
    drop_point_t_block_frame = get_mjobj_frame(model=m, data=d, obj_name="drop_point_tblock") * sm.SE3.Rx(-np.pi)  # Get body frame

    cylinder_frame = get_mjobj_frame(model=m, data=d, obj_name="cylinder") * sm.SE3.Rx(-np.pi)  # Get body frame
    drop_point_cylinder_frame = get_mjobj_frame(model=m, data=d, obj_name="drop_point_cylinder") * sm.SE3.Rx(-np.pi)  # Get body frame
    
    
    pickzone_init_guess = [0.264, -1.17, 1.18, -1.58, -1.57, -1.31]
    robot.move_j(start_q=robot.get_current_q(), end_q=pickzone_init_guess, t=100)

    EXE_TIME = 500 #ms  

    # start_q = robot.get_current_q()
    # Define grasping frames for object: box
    obj_frame = get_mjobj_frame(model=m, data=d, obj_name="box") # Get body frame
    obj_frame = obj_frame * sm.SE3.Rx(-np.pi)
    # move to the object
    goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame* sm.SE3.Tz(-0.15), q0=robot.queue[-1][0])[0]
    robot.move_j(start_q=robot.queue[-1][0], end_q=goal_q, t=EXE_TIME)
    #back to pickup position
    goal_q = robot.robot_ur5.ik_LM(Tep=obj_frame, q0=robot.queue[-1][0])[0]
    robot.move_j(start_q=robot.queue[-1][0], end_q=goal_q, t=EXE_TIME)
    
    robot.set_gripper(255) # Close gripper

    goal_q = robot.robot_ur5.ik_LM(Tep=box_frame * sm.SE3.Tz(-0.15), q0=robot.queue[-1][0])[0]
    robot.move_j(start_q=robot.queue[-1][0], end_q=goal_q, t=EXE_TIME)
    
    dropzone_init_guess = [2.92, -1.35, 1.42, -1.64, -1.57, 1.35]
    sol_traj = plan(d=d, m=m, start_q=robot.queue[-1][0], goal_q=dropzone_init_guess)

    robot.move_j_via(points=sol_traj, t=500)

    goal_q = robot.robot_ur5.ik_LM(Tep=drop_point_box_frame * sm.SE3.Tz(0.25), q0=robot.queue[-1][0])[0]
    robot.move_j(start_q=robot.queue[-1][0], end_q=goal_q, t=EXE_TIME)

    robot.set_gripper(0) # open gripper

    # Go up again
    robot.move_j(start_q=robot.queue[-1][0], end_q=dropzone_init_guess, t=EXE_TIME)

    # Reset
    sol_traj = plan(d=d, m=m, start_q=robot.queue[-1][0], goal_q=pickzone_init_guess)

    robot.move_j_via(points=sol_traj, t=500)

    return robot.queue        