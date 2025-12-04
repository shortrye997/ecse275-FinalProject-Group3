# -*- coding: utf-8 -*-
"""
Created on Wed Dec  3 15:25:41 2025

@author: peyte
"""

import coppeliasim_zmqremoteapi_client as zmq
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.spatial.transform import Rotation as R
import cv2

def get_range_data(sim,scripthandle,scriptfuncname,eps=0.06,min_samples=2,transform_pose=None):

    
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

def get_camera_image(sim, camera_handle):
    img_buffer, width, height = sim.getVisionSensorCharImage(camera_handle)
    
    res = [width, height]
    
    img = np.frombuffer(img_buffer, dtype=np.uint8).reshape(res[1], res[0], 3)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img,res

def detect_color_blobs(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    
    color_ranges = {
        "red": [((0, 100, 100), (10, 255, 255)), ((160, 100, 100), (179, 255, 255))],
        "green": [((40, 50, 50), (80, 255, 255))],
        "blue": [((100, 50, 50), (140, 255, 255))]
    }
    
    masks = {}
    
    for color, ranges in color_ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))
        masks[color] = mask
                   
    return masks

def lidar_to_image_robot_frame(centroid_robot, camera_pos_rel, camera_quat_rel, intrinsics, image_shape):
    
    pc = centroid_robot - camera_pos_rel
    
    R_cam = R.from_quat(camera_quat_rel)
    pc_cam = R_cam.inv().apply(pc)
    
    Xc, Yc, Zc = pc_cam
    fx, fy, cx, cy = intrinsics
    
    if Zc <= 0:
        return None
    
    u = int(fx * (Xc / Zc) + cx)
    v = int(fy * (Yc / Zc) + cy)
    
    height, width = image_shape
    if not (0 <= u < width and 0 <= v < height):
        return None
    
    return (u,v)

def match_lidar_to_camera_with_masks(lidar_objects, projected_points, color_masks, window=3):
    matches = []
    
    for i, proj in enumerate(projected_points):
        if proj is None:
            matches.append((i, None))
            continue
        
        u, v = proj
        assigned_color = None
        
        for color, mask in color_masks.items():
            found = False
            
            for du in range(-window, window+1):
                for dv in range(-window, window+1):
                    uu, vv = u + du, v + dv
                    if 0 <= uu < mask.shape[1] and 0 <= vv < mask.shape[0]:
                        if mask[vv, uu] > 0:
                            assigned_color = color
                            found = True
                            break
                if found: 
                    break
        matches.append((i, assigned_color))
    
    return matches


def get_obstacle_readings(sim, robot, hokuyo_scripthandle, camera_handle, intrinsics):
    """
    High-level helper:
    Runs LiDAR + camera, matches clusters to colors, and returns a list of
    (color_str, angle_rad, distance_m) tuples for each detected object.

    This is perfect to feed into obstacle_is_dangerous(color, angle, distance).
    """
    objects = get_range_data(sim, hokuyo_scripthandle, 'getMeasuredData')
    if len(objects) == 0:
        return []  # no obstacles

    img, res = get_camera_image(sim, camera_handle)
    image_shape = img.shape[:2]
    color_masks = detect_color_blobs(img)

    camera_pose_rel = sim.getObjectPose(camera_handle, robot)
    camera_pos_rel = np.array(camera_pose_rel[:3])
    camera_quat_rel = np.array(camera_pose_rel[3:])

    projected = []
    for obj in objects:
        proj = lidar_to_image_robot_frame(
            obj['centroid'], camera_pos_rel, camera_quat_rel, intrinsics, image_shape
        )
        projected.append(proj)
        print(f"[DEBUG] Object at {obj['centroid']} projected to image at {proj}")

    matches = match_lidar_to_camera_with_masks(objects, projected, color_masks)

    # Build a clean list of (color, angle, distance)
    obstacle_list = []
    print("\nFINAL MATCHES:")
    for i, color in matches:
        obj = objects[i]
        print(f"Object {i+1} → Dist={obj['distance']:.2f}m Angle={np.degrees(obj['angle']):.1f}° → Color={color}")
        obstacle_list.append(
            (color, obj['angle'], obj['distance'])  # (color_str, angle_rad, distance_m)
        )

    print("\n[DEBUG] obstacle_list:", obstacle_list)
    return obstacle_list


if __name__ == '__main__':
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
    robot = sim.getObjectHandle("/Pure_Robot/Dummy")
    hokuyo_scripthandle = sim.getObject('/hoku_script')
    camera_handle = sim.getObject("/Pure_Robot/visionSensor")
    
    fx, fy = 250, 250
    cx, cy = 128, 128
    intrinsics = (fx, fy, cx, cy)
    
    obstacle_list = get_obstacle_readings(
        sim, robot, hokuyo_scripthandle, camera_handle, intrinsics
    )

    print("\nObstacle readings (color, angle_rad, distance_m):")
    for obs in obstacle_list:
        print(obs)
