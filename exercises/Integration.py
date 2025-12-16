import numpy as np
import spatialmath as sm
import numpy as np
import matplotlib.pyplot as plt
import random
import mujoco

from ompl import base as ob
from ompl import geometric as og

import os

import open3d as o3d
import copy

from robot import *
from exercises.do_pe import do_pose_estimation
from exercises.cv_demo import program as cv_demo
from cam import get_pointcloud, get_camera_pose_cv
from exercises.Project import via_points

def program(d, m):
    # Define our robot object
    robot = UR5robot(data=d, model=m)
    # I0: Move duck to random position (done in cv_demo)
    cv_demo(d, m)
    
    # I1: Pose estimate the duck
    # Create renderer and capture scene point cloud
    renderer = mujoco.Renderer(m, height=480, width=640)
    get_pointcloud(m, d, renderer, pcd_File="point_cloud_0000.pcd", camera_name="cam1")
    scene_pointcloud = o3d.io.read_point_cloud("point_cloud_0000.pcd")
    
    # Load duck model as point cloud
    duck_mesh = o3d.io.read_triangle_mesh('./exercises/duck.stl')
    duck_pointcloud = duck_mesh.sample_points_poisson_disk(10000)
    
    # Estimate duck pose in camera coordinates
    estimated_pose = do_pose_estimation(scene_pointcloud, duck_pointcloud)
    
    # Get camera pose and convert estimated pose to world coordinates
    cam_se3 = get_camera_pose_cv(m, d, camera_name="cam1")
    duck_camera_pose = sm.SE3(estimated_pose, check=False)  # Convert 4x4 numpy array to SE3
    duck_world_pose = cam_se3 * duck_camera_pose  # Transform to world frame
    
    # I2 & I3: Define frames for grasping and placing
    obj_frame = duck_world_pose * sm.SE3.Rx(-np.pi)  # Duck grasp frame (from pose estimation)
    obj_drop_frame = get_mjobj_frame(model=m, data=d, obj_name="drop_point_box") * sm.SE3.Rx(np.pi)  # Box drop location
    pick_zone_frame = get_mjobj_frame(model=m, data=d, obj_name="pickup_point_cylinder") * sm.SE3.Rx(-np.pi)  # Pick zone
    drop_zone_frame = get_mjobj_frame(model=m, data=d, obj_name="drop_point_cylinder") * sm.SE3.Rx(np.pi)  # Drop zone
    
    # Generate trajectory using Point-to-Point interpolator with trapezoidal velocity profile
    trajectory = via_points(robot, obj_frame, obj_drop_frame, pick_zone_frame, drop_zone_frame, steps=800)

    return trajectory