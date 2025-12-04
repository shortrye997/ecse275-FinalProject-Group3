# -*- coding: utf-8 -*-
"""
Self Driving Smart Car
ECSE 275 Final Project

@author: Peyten Hargraves, Yechan Kim, Joshua Kim, Ryan Shorter
"""

import coppeliasim_zmqremoteapi_client as zmq
import matplotlib.pyplot as plt
import numpy as np
import finalprojectutils as util
from scipy.spatial.transform import Rotation as R
from sklearn.cluster import DBSCAN
import cv2


num_iterations = 5000 # iteration threshold
inflation_amt = 2

FOV_DEG = 90                               
HALF_FOV_RAD = np.deg2rad(FOV_DEG / 2)    
MAX_STOP_DIST = 0.5                        
DANGER_COLOR = "blue"  # color of obstacle

# Establish the ZMQ connection
client = zmq.RemoteAPIClient()
sim = client.getObject('sim')

#Get data about robot start and goal coords

goal = None
#goal = sim.getObjectHandle("/goal_point")
goal_world = None
#goal_world = sim.getObjectPosition(goal,sim.handle_world) # query the goal xyz coords in the world frame

robot = None
#robot = sim.getObjectHandle("/Pure_Robot/Dummy") 
start_world = None
#start_world = sim.getObjectPosition(robot,sim.handle_world) # output is x, y ,z

#Initialize the Grid Map
worldmap = None
#worldmap = util.gridmap(sim,5.0,goal_world,start_world,inflate_iter=inflation_amt)

current_path_list = None

state_dict = None

blocked_edges = set()

# One inplementation is only recognizing obstacles one road ahead of the robot
# 

found_goal = False

worldmap = None

travel_history = []


def astar(grid):
    """
    
    """
    print(f"Replanning from new start: {worldmap.start_grid_coord}")
    
    open_set = []
    explored_set = []
    parent_table_coord = np.zeros((worldmap.norm_map.shape[0],worldmap.norm_map.shape[1],2),dtype=int) # world H x W x 2 
    ctg_table = np.zeros_like(worldmap.norm_map)
    ctc_table = np.zeros_like(worldmap.norm_map)
    #

    
    ''' 
    One easy way to access and modify these variables in-place in Python is to use a dictionary that we pass to other functions.
    '''
    
    state_dict = {'open_set':open_set,
                  'explored_set':explored_set,
                  'parent_table': parent_table_coord,
                  'world_map': worldmap, # stores the states of the node (whether start (3), goal (2), obstacle (1) or open space (0) )
                  'ctg_table': ctg_table,
                  'ctc_table': ctc_table,
                  # NEW LINE
                  'blocked_edges': blocked_edges
                  }
    
    plt.close('all') # close all previously open plotting windows
    
    worldmap.plot(normalized=True) # visualize the world map
    
    # add the start point to the open set
    state_dict['open_set'].append(worldmap.start_grid_coord)
    
    # add the cost to come and cost to go of the start point
    state_dict['ctg_table'][util.coord_to_index(worldmap.start_grid_coord)] = cost_to_go(worldmap.start_grid_coord,worldmap.goal_grid_coord)
    state_dict['ctc_table'][util.coord_to_index(worldmap.start_grid_coord)] = 0
    
    path_list = None # set this to None and when the process_next_node function returns, it will update this to a non-None value
    counter = 0 # a counter to count iterations
    
    while path_list is None and counter<num_iterations: # set the iteration threshold, and check our termination condition
        print("iteration: " + str(counter))
        path_list = process_next_node(state_dict) # keep processing nodes until we terminate
        counter = counter + 1
    print(f"plot_current_state: explored_set: {explored_set}")
    state_dict['world_map'].plot_current_state(explored_set,open_set,path_list) # this plotting code can help you check the state of your exploration algorithm.
    
    # this should take your path list and generate the path in coppelia sim
    #path_in_world_coords_xy  = np.array(state_dict['world_map'].get_world_coords(np.array(path_list)))
    
    # path_in_world_coords_xy  = np.array(state_dict['world_map'].grid_to_world(np.array(path_list)))
    # path_in_world_coords_xyz = np.hstack((path_in_world_coords_xy,np.zeros((path_in_world_coords_xy.shape[0],1))))
    # coppelia_path = util.generate_path_from_trace(sim, path_in_world_coords_xyz)
    trackpoint = sim.getObjectHandle("/track_point")
    index_path = len(path_list)
    index_path_dupe = index_path
    index = 0
    current_node = path_list[index_path - 1]
    ending_node = path_list[index_path - 2]
    print(f"current_node: {current_node}")
    print(f"ending_node: {ending_node}")
    while (index + 1 < index_path_dupe):
        execute_one_step(current_node, ending_node, index_path, sim, trackpoint, robot)
        
        #Peyten's Code
        obstacle_list = get_obstacle_readings(
        sim, robot, hokuyo_scripthandle, camera_handle, intrinsics)
        
        print("\nObstacle readings (color, angle_rad, distance_m):")
        for obs in obstacle_list:
            print(obs)
            if (update_stop_flag(sim, obs)):
                # color, angle, distance = obs
                # robot_pos = np.array(sim.getObjectPosition(robot, sim.handle_world))
                # obs_x = robot_pos[0] + distance * np.cos(angle)
                # obs_y = robot_pos[1] + distance * np.sin(angle)
                # g_o = world_to_grid((obs_x, obs_y, 0))
                # print(f"g_o: {g_o}")
                
                add_blocked_edge(ending_node, path_list[index_path - 3])
                worldmap.start_grid_coord = ending_node
                return False

        
        print(f"path_list: {path_list}")
        print(f"index_path: {index_path}")
        index_path = index_path - 1
        current_node = path_list[index_path - 1]
        ending_node = path_list[index_path - 2]
        print(f"current_node: {current_node}")
        print(f"ending_node: {ending_node}")
        index = index + 1
        print(f"index: {index}")
    
    return True

def detect_obstacles(obs):
    FOV_DEG = 300
    color, angle, distance = obs
    return_value = True
    robot_pos = np.array(sim.getObjectPosition(robot, sim.handle_world))
    obs_x = robot_pos[0] + distance * np.cos(angle)
    obs_y = robot_pos[1] + distance * np.sin(angle)
    g_o = world_to_grid((obs_x, obs_y, 0))
    print(f"g_o: {g_o}")
    if (0.4 <= obs_x % 1 <= 0.6) and not (0.3 <= obs_y % 1 <= 0.7):
        add_blocked_edge((int(obs_x), int(obs_y)), (int(obs_x) + 1, int(obs_y)))
        return_value = False
    if (0.4 <= obs_y % 1 <= 0.6) and not (0.4 <= obs_x % 1 <= 0.6):
        add_blocked_edge((int(obs_x), int(obs_y)), (int(obs_x), int(obs_y)+1))
        return_value = False
    
    return return_value
            
    #execute_path(coppelia_path,sim,trackpoint,robot,thresh=0.1)
    
def add_blocked_edge(node_a, node_b):
    """
    Mark the road (edge) between two grid intersections as blocked.
    node_a, node_b: (x, y) grid coordinates of intersections.
    """
    global blocked_edges
    a = tuple(node_a)
    b = tuple(node_b)
    blocked_edges.add((a, b))
    blocked_edges.add((b, a))  # block both directions
    print(f"[blocked_edges] Blocking road between {a} and {b}")
    
# I WILL BE USING THE GRID COORDINATES FROM (0,0) TO (8,8)
def update(obstacle):
    """
    Needs to get done
    """

def process_next_node(state_dict):

    
    path_list = None
    goal_bool = None
    goal_node_coord = None
    
    open_set = state_dict['open_set']
    explored_set = state_dict['explored_set']
    total_cost = state_dict['ctg_table'] + state_dict['ctc_table']

    #open_set.sort(key=lambda coord: total_cost[util.coord_to_index(coord)])
    open_set.sort(key=lambda x: total_cost[x[1],x[0]],reverse=False)
    
    node_to_process_coord = open_set.pop(0)
    print('processing node: ' + str(node_to_process_coord[0]) + ',' +str(node_to_process_coord[1]))

    node_to_process_coord = node_to_process_coord[:2]
    
    explored_set.append(tuple(node_to_process_coord))
    #print(f"pnn, node_to_process_coord: {node_to_process_coord}")
    goal_bool, goal_node_coord = find_neighbors(node_to_process_coord, state_dict)
    
    if goal_bool:
        path_list = trace_path(goal_node_coord, state_dict)
    
    #
    # *** DO STUFF HERE ***
    #
    
    return path_list

def find_neighbors(current_node_coord,state_dict):

    # map the state dictionary keys to their own variables in the function.
    ctg_table = state_dict['ctg_table']
    ctc_table = state_dict['ctc_table']
    open_set = state_dict['open_set']
    explored_set = state_dict['explored_set']
    parent_table_coord = state_dict['parent_table']
    worldmap = state_dict['world_map']
    
    
    #FORBIDDEN LISTS: WHEN AN OBSTACLE BLOCKS A ROAD
    blocked_edges = state_dict['blocked_edges']
    
    # populate a list of neigbors based on a 4-connected grid
    #print(f"current node coord: {current_node_coord}")
    #node_x, node_y, z = current_node_coord
    node_x = current_node_coord[0]
    node_y = current_node_coord[1]
    
    node_list = [(node_x+1,node_y),(node_x-1,node_y),(node_x,node_y+1),(node_x,node_y-1)]
    
    node_list = [
        (node_x + 1, node_y),
        (node_x - 1, node_y),
        (node_x, node_y + 1),
        (node_x, node_y - 1)
    ]
    current_node_coord = (node_x, node_y)
    
    # filter out invalid neighbors
    node_list = [
        n for n in node_list
        if (0 <= n[0] <= 8 and 0 <= n[1] <= 8)
        and (current_node_coord, n) not in blocked_edges
    ]
    
    
    
    # initialize some default return values
    goal_bool = False
    goal_node_coord = None
    
    """
    for neighbor_coord in node_list:
        print('exploring neighbor ' + str(neighbor_coord[0]) + ',' + str(neighbor_coord[1]))
        if neighbor_coord in explored_set or neighbor_coord in open_set: # check if we are in the explored set or the open set
            print('already traverse. skipping...')
        else:
            try:
                
    
    """
    # make a for loop that loops through node_list to find and explore the neighbor nodes.
    for neighbor_coord in node_list:
        #print('exploring neighbor ' + str(neighbor_coord))
        #print(f"neighbor_coord: {neighbor_coord}")
        #print(f"explored_set: {explored_set}")
        if neighbor_coord in explored_set or neighbor_coord in open_set: # check if we are in the explored set or the open set
            #print('already traverse. skipping...')
            pass
        else:
            try: # a try statement can be used to attempt the operations in his block and skip over if we hit an exception or error in the code. This can be useful if to check if we index a neighbor that is out of bounds.
                node_type = worldmap.get_node_type_coord(neighbor_coord) # first we query the type of node (goal, obstacle or open space)
                node_idx = util.coord_to_index(neighbor_coord) # to properly index the node we have to convert from a coordinate to an index
                parent_table_coord[node_idx] = tuple(current_node_coord) # we set the parent of this neighbor node to the current_node_coord
                # if node_type == 1: # obstacle check
                #     print('found obstacle at ' + str(neighbor_coord[0]) + ',' + str(neighbor_coord[1]))
                #     explored_set.append(neighbor_coord)
                if node_type == 2: # goal check
                    goal_bool = True
                    goal_node_coord = neighbor_coord
                    print('found goal at ' + str(neighbor_coord[0]) + ',' + str(neighbor_coord[1]))
                else: # if not we are in free space
                    # compute the costs
                    ctg = cost_to_go(neighbor_coord,worldmap.goal_grid_coord)
                    ctc = cost_to_come(neighbor_coord,current_node_coord,ctc_table)
                    print('total cost: ' + str(ctc+ctg))
                    # add the neighbor to the open set
                    open_set.append(neighbor_coord) 
                    # update the state_dictionary values using the mapped variables
                    ctg_table[neighbor_coord] = ctg
                    ctc_table[neighbor_coord] = ctc
            except:
                pass

    return goal_bool,goal_node_coord # this return value is True and not None if we hit the goal.

def cost_to_go(current_node_coord,goal_coord):

    # ctg = abs(goal_coord[0] - current_node_coord[0]) + abs(goal_coord[1] - current_node_coord[1])
    # ctg = sum(abs(a - b) for a, b in zip(goal_coord, current_node_coord)) # ChatGPT taught me this
    goal_coord = (goal_coord[0], goal_coord[1])
    current_node_coord = (current_node_coord[0], current_node_coord[1])
    dist = np.array(goal_coord) - np.array(current_node_coord)
    ctg = np.linalg.norm(dist, 1)
    #
    # *** DO STUFF HERE ***
    #
    
    return ctg

def cost_to_come(neighbor_coord,current_node_coord,ctc_table):
 
    ctc = ctc_table[util.coord_to_index(current_node_coord)] + 1
    #ctc_table[neighbor_coord] = ctc_table[current_node_coord] + 1
    # ctc_table[neighbor_coord] = ctc
    #
    # *** DO STUFF HERE ***
    #
    
    return ctc

def trace_path(goal_node_coord,state_dict):

    parent_table_coord = state_dict['parent_table'] # map the parent_table key to the parent_table_coord variable
    
    current_node_coord = goal_node_coord # set the current_node_coord to the goal and work backwards from there
    path_list = [] # create an empty list
    
    start = tuple(worldmap.start_grid_coord)  # <-- new dynamic start
    
    while (worldmap.get_node_type_coord(current_node_coord) != 3) and (tuple(current_node_coord) != start): # while we are not at the start
        #
        # *** DO STUFF HERE ***
        path_list.append(np.array(current_node_coord))
        print(f"trace-path, current_node_coord: {current_node_coord}")
        index = util.coord_to_index(current_node_coord)
        print(f"trace-path, index: {index}")
        current_node_coord = parent_table_coord[index]
        print(f"trace-path, current_node_coord: {current_node_coord}")
        # Tuple conversion
        #
    path_list.append(current_node_coord)
    
    print(f"path_list: {path_list}")
    return path_list

def execute_one_step(start, end, index, sim,trackpoint_handle,robot_handle,thresh=0.05):
    global travel_history
    global state_dict
    
    if not travel_history:
        travel_history.append(tuple(start))
        
    travel_history.append(tuple(end))
    
    path_in_world_coords_xy  = np.array(state_dict['world_map'].grid_to_world(np.array([start, end])))
    path_in_world_coords_xyz = np.hstack((path_in_world_coords_xy,np.zeros((path_in_world_coords_xy.shape[0],1))))
    coppelia_path = util.generate_path_from_trace(sim, path_in_world_coords_xyz)
    path_index = 1
    timer = 0
    
    target_point = coppelia_path[-path_index,:]
    if any(np.isnan(target_point)):
        target_point[3:] = [0.0,0.0,0.0,1.0]
    sim.setObjectPose(trackpoint_handle,sim.handle_world, target_point.tolist())
    print(f"target_point.tolist(): {target_point.tolist()}")
    rob_trackpt_dist = 100
    while (rob_trackpt_dist > thresh):
        robot_pos = sim.getObjectPosition(robot_handle,sim.handle_world)
        trackpt_pos = sim.getObjectPosition(trackpoint_handle,sim.handle_world)
        #print(f"robot_pos: {robot_pos}")
        #print(f"trackpt_pos: {trackpt_pos}")
        rob_trackpt_dist = np.linalg.norm(np.array(robot_pos)-np.array(trackpt_pos))
        timer = timer + 1
    path_index = path_index + 1
    


def execute_path(pathData_array,sim,trackpoint_handle,robot_handle,thresh=0.15):

    path_index = 1
    timer = 0
    while path_index<=pathData_array.shape[0]:
        # set the track point pos
        target_point = pathData_array[-path_index,:]
        #print(f"target_point: {target_point}")
        if any(np.isnan(target_point)):
            target_point[3:] = [0.0,0.0,0.0,1.0]
        sim.setObjectPose(trackpoint_handle,sim.handle_world,list(pathData_array[-path_index,:]))
        #obstacle_list = get_range_data()
        #if ():
        #    update()
        print(list(pathData_array[-path_index,:]))
        # get the current robot position
        robot_pos = sim.getObjectPosition(robot_handle,sim.handle_world)
        trackpt_pos = sim.getObjectPosition(trackpoint_handle,sim.handle_world)
        print(f"robot_pos: {robot_pos}")
        print(f"trackpt_pos: {trackpt_pos}")
        # compute the distance between the trackpt position and the robot
        rob_trackpt_dist = np.linalg.norm(np.array(robot_pos)-np.array(trackpt_pos))
        #print(f"execute_path rob_trackpt_dist: {rob_trackpt_dist}")
        
        if (rob_trackpt_dist < thresh) or (timer > 40):
            path_index = path_index + 1
            print("next_point")
            timer = 0
            
        else:
            timer = timer + 1

def world_to_grid(point_world):
    """
    Parameters
    ----------
    point_world : World coordinates (x,y)
        DESCRIPTION.

    Returns
    -------
    tuple: Grid coordinates
    
    Wx=2-0.5Mx
    Wy=-2+0.5My

    """
    """
    pos = (-point_world[0]*2 + 4, point_world[1]*2 + 4)
    
    return tuple(pos)
"""
    x, y, z = point_world  # unpack world coords

    # Apply the same grid mapping to x,y
    grid_x = -x * 2 + 4
    grid_y = y * 2 + 4
    
    # Decide what to do with Z:
    # Option A: keep as-is (continuous world → continuous grid)
    grid_z = z

    return (grid_x, grid_y, grid_z)

def obstacle_is_dangerous(color, angle_rad, distance_m):
    """
    Decide whether this obstacle should cause the robot to stop.
    """
    in_fov = abs(angle_rad) <= HALF_FOV_RAD
    in_range = distance_m <= MAX_STOP_DIST
    #is_color = (color == DANGER_COLOR)
    return in_fov and in_range
    #return in_fov and in_range and is_color


def update_stop_flag(sim, obstacle_info):
    """
    Sets 'stopFlag' in CoppeliaSim:
      1 -> STOP
      0 -> GO
    Returns:
      danger (bool)
    """

    if obstacle_info is None:
        sim.setInt32Signal('stopFlag', 0)
        return False

    color, angle_rad, distance_m = obstacle_info
    danger = obstacle_is_dangerous(color, angle_rad, distance_m)
    sim.setInt32Signal('stopFlag', 1 if danger else 0)
    return danger

        

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

def plot_travel_history(worldmap, travel_history):

    if not travel_history:
        print("[plot_travel_history] No travel history to plot.")
        return

    grid_xy = np.array([ (n[0], n[1]) for n in travel_history ])


    world_xy = np.array(worldmap.grid_to_world(grid_xy))
    xs = world_xy[:, 0]
    ys = world_xy[:, 1]

    plt.figure()

    worldmap.plot(normalized=True)

    plt.plot(xs, ys, '-o')   # robot trajectory
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Robot Travel Path")
    plt.axis('equal')
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    
    
    num_iterations = 5000 # iteration threshold
    inflation_amt = 2
    
    # Establish the ZMQ connection
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')

    #Get data about robot start and goal coords
    
    goal = sim.getObjectHandle("/goal_point")
    goal_world = sim.getObjectPosition(goal,sim.handle_world) # query the goal xyz coords in the world frame\
    print(f"goal_world: {goal_world}")
    # goal_world_test = world_to_grid(goal_world)
    # print(f"goal_world_test: {goal_world_test}")

    robot = sim.getObjectHandle("/Pure_Robot/Dummy") 
    start_world = sim.getObjectPosition(robot,sim.handle_world) # output is x, y ,z
    print(f"start_world: {start_world}")
    
    
    trackpoint = sim.getObjectHandle("/track_point")
    
    hokuyo_scripthandle = sim.getObject('/hoku_script')
    camera_handle = sim.getObject("/Pure_Robot/visionSensor")
    
    fx, fy = 250, 250
    cx, cy = 128, 128
    intrinsics = (fx, fy, cx, cy)
    
    #objects = get_range_data(sim, hokuyo_scripthandle, 'getMeasuredData')
    
    #Initialize the Grid Map
    worldmap = util.gridmap(sim,5.0,goal_world,start_world,inflate_iter=inflation_amt)

    #Declare your state variables

    open_set = []
    explored_set = []
    parent_table_coord = np.zeros((worldmap.norm_map.shape[0],worldmap.norm_map.shape[1],2),dtype=int) # world H x W x 2 
    ctg_table = np.zeros_like(worldmap.norm_map)
    ctc_table = np.zeros_like(worldmap.norm_map)
    #
    blocked_edges = set()

    
    state_dict = {'open_set':open_set,
                  'explored_set':explored_set,
                  'parent_table': parent_table_coord,
                  'world_map': worldmap, # stores the states of the node (whether start (3), goal (2), obstacle (1) or open space (0) )
                  'ctg_table': ctg_table,
                  'ctc_table': ctc_table,
                  # NEW LINE
                  }
    
    robot_pos = sim.getObjectPosition(robot,sim.handle_world)
    print(f"robot_pos: {robot_pos}")
    trackpt_pos = sim.getObjectPosition(trackpoint,sim.handle_world)
    print(f"trackpt_pos: {trackpt_pos}")

    
    plt.close('all') # close all previously open plotting windows
    
    worldmap.plot(normalized=True) # visualize the world map
    
    while not found_goal:
        found_goal = astar(worldmap)
        
    plot_travel_history(worldmap, travel_history)
    

        
