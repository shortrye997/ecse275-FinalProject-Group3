# -*- coding: utf-8 -*-
"""
ECSE 275 Final Project
12/4/2025

@author: Peyten Hargraves, Joshua Kim, Yechan Kim, Ryan Shorter
"""

import coppeliasim_zmqremoteapi_client as zmq
import matplotlib.pyplot as plt
import numpy as np
import finalprojectutils as util

# I WILL BE USING THE GRID COORDINATES FROM (0,0) TO (8,8)

FOV_DEG = 90                               
HALF_FOV_RAD = np.deg2rad(FOV_DEG / 2)     # ±45 degrees
MAX_STOP_DIST = 0.6                        
DANGER_COLOR = "red"  # color of obstacle


def update(obstacle):
    """
    Needs to get done
    """

def process_next_node(state_dict):
    """
    Processes the next node in the open set as part of a path-planning algorithm.
    
    Parameters:
    state_dict : dict
        A dictionary containing a least the following keys:
        - 'open_set' (list): List of nodes (grid coordinates) that are open for exploration.
        - 'explored_set' (list): List of nodes that have already been explored.
        - 'ctg_table' (numpy array): A 2D array representing the cost to go (heuristic) for each node.
        - 'ctc_table' (numpy array): A 2D array representing the cost to come for each node (path cost from the start).
    
    Returns:
    path_list : list or None
        The traced path as a list of coordinates if the goal node is found. Returns None if the goal is not found during this step.
    """
    
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
    #print(f"PATIENT ZERO?: {node_to_process_coord}")
    
    explored_set.append(node_to_process_coord)
    #print(f"pnn, node_to_process_coord: {node_to_process_coord}")
    goal_bool, goal_node_coord = find_neighbors(node_to_process_coord, state_dict)
    
    if goal_bool:
        path_list = trace_path(goal_node_coord, state_dict)
    
    return path_list

def find_neighbors(current_node_coord,state_dict):
    """
    Explores the neighboring nodes of the current node and updates the state dictionary accordingly.
    
    This function checks the 4-connected neighbors (up, down, left, right) of the current node to determine their type (goal, obstacle, or free space) and updates the state dictionary with the costs and parents for valid neighbors. If a goal node is found, it returns `True` and the coordinates of the goal.
    
    Parameters:
    current_node_coord : tuple
        A tuple representing the grid coordinates (x, y) of the current node.
    
    state_dict : dict
        A dictionary containing the current state of the algorithm, with the following keys:
        - 'ctg_table' (numpy array): A 2D array storing the cost to go (heuristic cost) for each node.
        - 'ctc_table' (numpy array): A 2D array storing the cost to come (actual cost from start) for each node.
        - 'open_set' (list): A list of nodes (grid coordinates) that are open for exploration.
        - 'explored_set' (list): A list of nodes (grid coordinates) that have already been explored.
        - 'parent_table' (2D array): A table storing the parent node for each node.
        - 'world_map' (object): The map of the world that includes obstacles and goals.
    
    Returns:
    goal_bool : bool
        True if the goal node is found; otherwise, False.
    
    goal_node_coord : tuple or None
        The coordinates of the goal node if found; otherwise, None.
    """
    
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
        if neighbor_coord in explored_set or neighbor_coord in open_set: # check if we are in the explored set or the open set
            #print('already traverse. skipping...')
            pass
        else:
            try: # a try statement can be used to attempt the operations in his block and skip over if we hit an exception or error in the code. This can be useful if to check if we index a neighbor that is out of bounds.
                node_type = worldmap.get_node_type_coord(neighbor_coord) # first we query the type of node (goal, obstacle or open space)
                node_idx = util.coord_to_index(neighbor_coord) # to properly index the node we have to convert from a coordinate to an index
                parent_table_coord[node_idx] = tuple(current_node_coord) # we set the parent of this neighbor node to the current_node_coord
                if node_type == 1: # obstacle check
                    print('found obstacle at ' + str(neighbor_coord[0]) + ',' + str(neighbor_coord[1]))
                    explored_set.append(neighbor_coord)
                elif node_type == 2: # goal check
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
            #     #print("code is running try")    
            #     node_type = worldmap.get_node_type_coord(neighbor_coord) # first we query the type of node (goal, obstacle or open space)
            #     #print(node_type)
            #     node_idx = util.coord_to_index(neighbor_coord) # to properly index the node we have to convert from a coordinate to an index
            #     #print(node_idx)
            #     parent_table_coord[node_idx] = current_node_coord # we set the parent of this neighbor node to the current_node_coord
            #     #print("code is running try try 2") 
                
            #     #if node_type == 1: # obstacle check
            #         # print('found obstacle at ' + str(neighbor_coord[0]) + ',' + str(neighbor_coord[1]))
            #         #explored_set.append(neighbor_coord)
            #         #print(f"obstacle check, node_to_process_coord: {node_to_process_coord}")

                    
            #         # ***DO STUFF HERE***
            #     #elif node_type == 2: # goal check
            #     if node_type == 2: # goal check
            #         #print('found goal at ' + str(neighbor_coord[0]) + ',' + str(neighbor_coord[1]))
            #         #
            #         # ***DO STUFF HERE***
            #         goal_bool = True
            #         goal_node_coord = neighbor_coord
            #         break
            #         #
            #     else: # if not we are in free space
            #         #
            #         # ***DO STUFF HERE***
            #         #print("code is running")
            #         #print(f"Exploring neighbor {neighbor_coord}")
            #         #print(f"worldmap.goal_grid_coord: {worldmap.goal_grid_coord}")
            #         ctg = cost_to_go(neighbor_coord, worldmap.goal_grid_coord)
            #         #print("did ctg work")
            #         ctc = cost_to_come(neighbor_coord, current_node_coord, ctc_table)
            #         #print("did ctc work")
            #         open_set.append(neighbor_coord)
            #         #print(f"Added to open_set: {neighbor_coord}, ctg={ctg}, ctc={ctc}")
            #         ctg_table[util.coord_to_index(neighbor_coord)] = ctg
            #         print("does this print ctg")
            #         ctc_table[util.coord_to_index(neighbor_coord)] = ctc
            #         print("does this print ctc")
            #         parent_table_coord[node_x][node_y] = neighbor_coord
            #         print(f"what even is this {parent_table_coord[node_x][node_y]}")
            #         print("does this print")
            #         parent_table_coord[util.coord_to_index(neighbor_coord)] = current_node_coord
            #         #
            # except:
            #     pass
        # print("Open set now:", open_set)
        
    return goal_bool,goal_node_coord # this return value is True and not None if we hit the goal.

def cost_to_go(current_node_coord,goal_coord):
    """
    Calculates the heuristic cost (cost-to-go) from the current node to the goal node.
    
    This function computes the Manhattan distance between the current node and the goal node, 
    which is used as a heuristic in grid-based path planning. The Manhattan distance is 
    the sum of the absolute differences between the x and y coordinates of the two points.
    
    Parameters:
    -----------
    current_node_coord : tuple
        The (x, y) coordinates of the current node.
    
    goal_coord : tuple
        The (x, y) coordinates of the goal node.
    
    Returns:
    --------
    ctg : float
        The cost-to-go, represented by the Manhattan distance between the current node 
        and the goal node.
    """ 
    
    # ctg = abs(goal_coord[0] - current_node_coord[0]) + abs(goal_coord[1] - current_node_coord[1])
    # ctg = sum(abs(a - b) for a, b in zip(goal_coord, current_node_coord)) # ChatGPT taught me this
    goal_coord = (goal_coord[0], goal_coord[1])
    current_node_coord = (current_node_coord[0], current_node_coord[1])
    dist = np.array(goal_coord) - np.array(current_node_coord)
    ctg = np.linalg.norm(dist, 1)
    return ctg

def cost_to_come(neighbor_coord,current_node_coord,ctc_table):
    """
    Calculates the actual cost (cost-to-come) from the start node to the current neighbor node.

    The cost-to-come (ctc) is incremented by 1 from the current node to its neighbor, assuming
    uniform movement cost between adjacent nodes. This function adds 1 to the ctc value of the
    current node and returns it for the neighbor node.

    Parameters:
    -----------
    neighbor_coord : tuple
        The (x, y) coordinates of the neighboring node being evaluated.
    
    current_node_coord : tuple
        The (x, y) coordinates of the current node.

    ctc_table : numpy array
        A 2D array where each entry contains the cost-to-come value for the corresponding node.

    Returns:
    --------
    ctc : float
        The cost-to-come from the start node to the neighbor node, incremented from the 
        current node's ctc value by 1.
    """
    
    ctc = ctc_table[util.coord_to_index(current_node_coord)] + 1
    #ctc_table[neighbor_coord] = ctc_table[current_node_coord] + 1
    # ctc_table[neighbor_coord] = ctc
    return ctc

def trace_path(goal_node_coord,state_dict):
    """
    Traces the path from the goal node back to the start node using the parent table.
    
    This function works by starting at the goal node and following the parent chain in reverse 
    until it reaches the start node. Each step from the goal to the start is recorded in the 
    `path_list`, which is returned as the final output.
    
    Parameters:
    -----------
    goal_node_coord : tuple
        The (x, y) coordinates of the goal node from which the backtracking starts.
    
    state_dict : dict
        A dictionary containing the current state of the path-planning algorithm, with the following key:
        - 'parent_table' (2D array): A table storing the parent node for each node. The parent table allows backtracking from the goal node to the start node.
    
    Returns:
    --------
    path_list : list of tuples
        A list of grid coordinates representing the path from the goal node to the start node.
        The list is ordered from the goal node to the start node, with each tuple representing 
        a node's (x, y) coordinate along the path.
    
    Functionality:
    --------------
    1. Retrieves the parent table from `state_dict` and initializes `current_node_coord` to the goal node.
    2. Starts a loop that continues until the start node is reached:
       - At each step, the current node's coordinates are added to the `path_list`.
       - The parent of the current node is retrieved using the parent table.
       - The current node is updated to its parent, effectively moving one step back toward the start.
    3. The start node is identified by its node type, which is checked using `worldmap.get_node_type_coord()`.
    4. Returns the `path_list`, which contains the sequence of nodes from the goal to the start.
    """
    parent_table_coord = state_dict['parent_table'] # map the parent_table key to the parent_table_coord variable
    
    current_node_coord = goal_node_coord # set the current_node_coord to the goal and work backwards from there
    path_list = [] # create an empty list
    while worldmap.get_node_type_coord(current_node_coord) != 3: # while we are not at the start

        path_list.append(np.array(current_node_coord))
        print(f"trace-path, current_node_coord: {current_node_coord}")
        index = util.coord_to_index(current_node_coord)
        print(f"trace-path, index: {index}")
        current_node_coord = parent_table_coord[index]
        print(f"trace-path, current_node_coord: {current_node_coord}")
        # Tuple conversion

    path_list.append(current_node_coord)
    
    print(f"path_list: {path_list}")
    return path_list


def obstacle_is_dangerous(color, angle_rad, distance_m):
    """
    Decide whether this obstacle should cause the robot to stop.
    """
    in_fov = abs(angle_rad) <= HALF_FOV_RAD
    in_range = distance_m <= MAX_STOP_DIST
    is_color = (color == DANGER_COLOR)
    return in_fov and in_range and is_color

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


def get_path_points(sim, num_iterations=5000, inflation_amt=2):
    goal = sim.getObjectHandle("/goal_point")
    goal_world = sim.getObjectPosition(goal, sim.handle_world)  # query the goal xyz coords in the world frame
    print(f"goal_world: {goal_world}")

    robot = sim.getObjectHandle("/Pure_Robot/Dummy")
    start_world = sim.getObjectPosition(robot, sim.handle_world)

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
                  'blocked_edges': blocked_edges
                  }
    plt.close('all') # close all previously open plotting windows
    
    worldmap.plot(normalized=True) # visualize the world map
    
    # add the start point to the open set
    state_dict['open_set'].append(worldmap.start_grid_coord)
    #print("open_set")
    #print(state_dict['open_set'])
    
    # add the cost to come and cost to go of the start point
    state_dict['ctg_table'][util.coord_to_index(worldmap.start_grid_coord)] = cost_to_go(worldmap.start_grid_coord,worldmap.goal_grid_coord)
    state_dict['ctc_table'][util.coord_to_index(worldmap.start_grid_coord)] = 0
    
    path_list = None # set this to None and when the process_next_node function returns, it will update this to a non-None value
    counter = 0 # a counter to count iterations
    while path_list is None and counter<num_iterations: # set the iteration threshold, and check our termination condition
        print("iteration: " + str(counter))
        path_list = process_next_node(state_dict) # keep processing nodes until we terminate
        counter = counter + 1
    # I COMMENTED THIS OUT
    print(f"plot_current_state: explored_set: {explored_set}")
    state_dict['world_map'].plot_current_state(explored_set,open_set,path_list)

    # this should take your path list and generate the path in coppelia sim
    #path_in_world_coords_xy  = np.array(state_dict['world_map'].get_world_coords(np.array(path_list)))
    path_in_world_coords_xy  = np.array(state_dict['world_map'].grid_to_world(np.array(path_list)))
    path_in_world_coords_xyz = np.hstack((path_in_world_coords_xy,np.zeros((path_in_world_coords_xy.shape[0],1))))

    return path_in_world_coords_xyz
    
if __name__ == '__main__':
    
    num_iterations = 5000 # iteration threshold
    inflation_amt = 2
    
    # Establish the ZMQ connection
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
    
    path_in_world_coords_xyz = get_path_points(sim, num_iterations=num_iterations, inflation_amt=inflation_amt)
    
    coppelia_path = util.generate_path_from_trace(sim, path_in_world_coords_xyz)
    trackpoint = sim.getObjectHandle("/track_point")
    robot = sim.getObjectHandle("/Pure_Robot/Dummy")
    util.execute_path(coppelia_path,sim,trackpoint,robot,thresh=0.1)



