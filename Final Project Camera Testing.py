# -*- coding: utf-8 -*-
"""
Created on Wed Dec  3 15:25:41 2025

@author: peyte
"""

import coppeliasim_zmqremoteapi_client as zmq
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R

def get_range_data(sim,scripthandle,scriptfuncname,eps=0.06,min_samples=2,transform_pose=None):
    '''
    Parameters:
    -----------
    sim : object
        The simulation object, which provides access to the simulation API.
    
    scriptfuncname : str
        The name of the script function to call in the simulation environment. This function is assumed to be 
        defined as a child script in the simulation and is expected to return range data.
    
    transform_pose : list or numpy array, optional
        A 7-element pose vector representing the robot's position and orientation in the world frame.
        The first 3 elements are the position (x, y, z), and the remaining 4 elements are the orientation 
        as a quaternion (qx, qy, qz, qw). If provided, the retrieved range data is transformed from the robot's 
        local frame to the world frame using this pose.
    
    Returns:
    --------
    output_robotframe : numpy array
        A numpy array of shape (n, 3) containing the range data points, either in the robot's local frame 
        or transformed to the world frame if a transformation pose is provided.
        Each row represents a point in 3D space (x, y, z).
    '''
    
    output = sim.callScriptFunction(scriptfuncname, scripthandle)
    try:
        points = np.array(output).reshape((-1,3))
    except:
        points = np.zeros((0,3))
        
    print(f"[DEBUG] Number of LiDAR points read: {len(points)}")
    if len(points) ==0:
        return[]
    
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(points[:, :2])
    labels = db.labels_
    unique_labels = set(labels)
    print(f"[DEBUG] Unique DBSCAN labels found: {unique_labels}")
    
    unique_labels.discard(-1)
    
    print(f"[DEBUG] Number of clusters detected: {len(unique_labels)}")
    
    objects = []
    for label in unique_labels:
        cluster_points = points[labels == label]
        print(f"\n[DEBUG] Cluster {label} has {len(cluster_points)} points")
        print("        Sample points:\n", cluster_points[:3])
        
        centroid = cluster_points.mean(axis=0)
        distance = np.linalg.norm(centroid[:2])
        angle = np.arctan2(centroid[1], centroid[0])
        
        print(f"        Centroid: {centroid}")
        print(f"        Distance: {distance:.3f} m")
        print(f"        Angle: {np.degrees(angle):.1f}°")
       
        objects.append({
            'points': cluster_points,
            'centroid': centroid,
            'distance': distance,
            'angle': angle
        })
    
    print("\n[DEBUG] Final number of objects returned:", len(objects))
    
    return objects

def transform_to_camera_frame(points_robot, R_cam, t_cam):
    points_cam = (R_cam @ points_robot.T).T + t_cam.T
    return points_cam

def project_to_image(points_cam, K):
    points_2d_hom = (K @ points_cam.T).T
    u = points_2d_hom[:,0] / points_2d_hom[:,2]
    v = points_2d_hom[:,1] / points_2d_hom[:,2]
    return np.stack([u,v], axis=1)

def get_colors(image, pixels):
    colors = []
    H, W, _ = image.shape
    for u,v in pixels:
        u,v = int(round(u)), int(round(v))
        if 0 <= u < W and 0 <= v < H:
            colors.append(image[v, u])
        else:
            colors.append([0,0,0])
    return np.array(colors)

if __name__ == '__main__':
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
    robot = sim.getObjectHandle("/Pure_Robot/Dummy")
    hokuyo_scripthandle = sim.getObject('/hoku_script')
    camera_handle = sim.getObject("/Pure_Robot/visionSensor")
    
    # get the robot pose data
    robot_pose = sim.getObjectPose(robot,sim.handle_world)
    robot_pos_world = robot_pose[:3]
    
    robot_pose = sim.getObjectPose(robot,sim.handle_world)
    robot_pos_world = robot_pose[:3]
        
    rangedata_worldframe = get_range_data(sim,hokuyo_scripthandle,'getMeasuredData',0.06)
    