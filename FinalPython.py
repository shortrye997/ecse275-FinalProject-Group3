# -*- coding: utf-8 -*-
"""
Created on Tue Nov 25 00:04:54 2025

@author: ched0
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
    
    This function performs the following tasks:
    1. Sorts the open set based on the total cost (cost to go + cost to come).
    2. Selects the next node with the lowest cost from the open set and processes it.
    3. Moves the selected node to the explored set.
    4. Checks if the goal node is found by exploring the neighbors of the selected node.
    5. If the goal is found, traces the path from the goal node to the start node.
    
    Parameters:
    -----------
    state_dict : dict
        A dictionary containing a least the following keys:
        - 'open_set' (list): List of nodes (grid coordinates) that are open for exploration.
        - 'explored_set' (list): List of nodes that have already been explored.
        - 'ctg_table' (numpy array): A 2D array representing the cost to go (heuristic) for each node.
        - 'ctc_table' (numpy array): A 2D array representing the cost to come for each node (path cost from the start).
    
    Returns:
    --------
    path_list : list or None
        The traced path as a list of coordinates if the goal node is found. Returns None if the goal is not found during this step.
    """
    
    '''
    Homework Instructions:
    ----------------------
    In this function, you will implement a node processing step for a path-planning algorithm. The task is to process the next node 
    from the open set, check if it leads to the goal, and update the explored set accordingly. Follow the steps and the comments to 
    complete the function.

    1. The function receives `state_dict`, which contains:
        - `open_set`: A list of nodes (grid coordinates) that are still open for exploration.
        - `explored_set`: A list of nodes (grid coordinates) that have already been explored.
        - `ctg_table`: A 2D array that stores the cost to go (heuristic) for each node.
        - `ctc_table`: A 2D array that stores the cost to come for each node (actual path cost from the start).
        - You will combine the two cost tables to form the total cost table: `total_cost = ctc_table + ctg_table`.

    2. **Sorting the Open Set**: 
       - The `open_set` needs to be sorted based on the combined values in the total cost table (`total_cost`), in ascending order. 
       This is done so that the node with the lowest cost is processed first.
       - You can use Python's provided `sort()` function to sort the `open_set` in-place using a "key" function where total_cost is 
       indexed by the elements in the open set.

    3. **Processing the Next Node**:
       - After sorting, remove the node with the lowest cost from the `open_set` (i.e., pop the first element using a pop method on 
       the list) and store it in `node_to_process_coord`.
       - Add this node to the `explored_set` to mark it as processed.

    4. **Exploring Neighbors**:
       - Use a helper function called `find_neighbors` (implemented below) to explore the neighboring nodes of 
       `node_to_process_coord`.
       - `find_neighbors` will return two values:
         - `goal_bool`: A boolean indicating whether a neighbor is the goal.
         - `goal_node_coord`: The grid coordinates of the goal if found.

    5. **Tracing the Path**:
       - If the goal is found (`goal_bool is True`), call another function `trace_path` (implemented below) to backtrack from the 
       goal to the start and retrieve the full path.

    Your task:
    - Complete the function using the provided steps and comments.
    
    '''
    
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
    
    #
    # *** DO STUFF HERE ***
    #
    
    return path_list

def find_neighbors(current_node_coord,state_dict):
    """
    Explores the neighboring nodes of the current node and updates the state dictionary accordingly.
    
    This function checks the 4-connected neighbors (up, down, left, right) of the current node to determine their type (goal, obstacle, or free space) and updates the state dictionary with the costs and parents for valid neighbors. If a goal node is found, it returns `True` and the coordinates of the goal.
    
    Parameters:
    -----------
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
    --------
    goal_bool : bool
        True if the goal node is found; otherwise, False.
    
    goal_node_coord : tuple or None
        The coordinates of the goal node if found; otherwise, None.
    
    Homework Instructions:
    ----------------------
    In this assignment, you will implement the logic to explore the neighbors of a given node
    in a path-planning algorithm. You will identify whether a neighbor is an obstacle, the goal,
    or free space, and update the state of the search accordingly. Follow the steps below to 
    complete the function.

    1. **Input Structure**:
        - `current_node_coord`: This is the coordinate (x, y) of the node you are currently processing.
        - `state_dict`: A dictionary that holds the state of the algorithm with the following components:
            - `ctg_table`: A 2D array that stores the "cost-to-go" (heuristic) for each node.
            - `ctc_table`: A 2D array that stores the "cost-to-come" (actual cost from the start) for each node.
            - `open_set`: A list of nodes (grid coordinates) that are open for exploration.
            - `explored_set`: A list of nodes (grid coordinates) that have already been explored.
            - `parent_table`: A table storing the parent node for each explored node.
            - `world_map`: An object that includes information about obstacles, free space, and the goal.

    2. **Exploring Neighbors**:
        - For each node, identify its 4-connected neighbors (up, down, left, right).
        - Loop through the list of neighbors (`node_list`), and for each neighbor:
            - **Check if it's already explored or in the open set**: 
                - If it is, skip the node and continue to the next neighbor.
            - **Identify the type of node** (goal, obstacle, or free space):
                - Use the `world_map.get_node_type_coord()` function to get the type of the neighbor.
                - Use a `try-except` block to handle any indexing errors for neighbors outside the grid.

    3. **Node Type Actions**:
        - **Obstacle (node_type == 1)**: 
            - If the neighbor is an obstacle, add it to the `explored_set` and skip it.
        - **Goal (node_type == 2)**: 
            - If the neighbor is the goal, set `goal_bool` to `True`, store the coordinates in `goal_node_coord`,
              and exit the loop (you've found the goal, no need to continue).
        - **Free Space (node_type == 0)**: 
            - If the neighbor is free space, compute its:
                - **Cost-to-go (ctg)**: The heuristic cost from the neighbor to the goal.
                - **Cost-to-come (ctc)**: The actual cost from the start node to this neighbor.
            - Add the neighbor to the `open_set` for further exploration.
            - Update its `ctg` and `ctc` in the `ctg_table` and `ctc_table`.
            - Set the parent of the neighbor to the current node in the `parent_table`.

    4. **Function Output**:
        - Return two values:
            - `goal_bool`: A boolean indicating whether the goal has been found (`True` if found, `False` otherwise).
            - `goal_node_coord`: The coordinates of the goal node if found, or `None` if the goal has not been found.

    Your task:
    ----------
    - Complete the function using the steps provided.
    - Ensure that each neighbor is correctly evaluated based on its type (goal, obstacle, free space).
    - Make sure the state dictionary is updated with the new costs and parent information for valid neighbors.

    Testing:
    --------
    Test your function with a variety of scenarios, including:
    - Neighbors that include obstacles.
    - Neighbors that include the goal node.
    - Neighbors in free space where costs need to be calculated.

    Hint:
    -----
    - Use `util.coord_to_index()` to convert a coordinate (x, y) to an index for updating tables.
    - Handle edge cases where the neighbor might be out of bounds using the `try-except` block.
    
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
        
    Homework Instructions:
    ----------------------
    - This function calculates the cost-to-go from a current node to the goal node using the 
      Manhattan distance.
    - The Manhattan distance is the sum of the absolute differences between the x and y coordinates 
      of the two points.
     
    Your Task:
    ----------
    - Use NumPy's `np.linalg.norm()` function to calculate the Manhattan distance between 
      `current_node_coord` and `goal_coord`.
    - Return the computed distance as the cost-to-go.
     
    Parameters:
    - `current_node_coord`: The (x, y) coordinates of the current node.
    - `goal_coord`: The (x, y) coordinates of the goal node.
     
    Return:
    - The Manhattan distance between the current node and the goal.
    
    Testing:
    --------
    - Test the function by calculating the function between several points on a grid.
    - Verify that the Manhattan distance is correctly calculated.
    """ 
    
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
 
    Homework Instructions:
    ----------------------           
    - This function calculates the cost-to-come from the start node to a neighboring node.
    - The cost-to-come is incremented by 1 for each movement from one node to an adjacent node, 
      assuming uniform movement cost.
    
    Your Task:
    ----------
    - Use the `ctc_table` to get the cost-to-come value of `current_node_coord`.
    - Add 1 to that value to represent the cost of moving to `neighbor_coord`.
    - Return the new cost as the cost-to-come for the neighbor node.
    
    Parameters:
    - `neighbor_coord`: The (x, y) coordinates of the neighboring node.
    - `current_node_coord`: The (x, y) coordinates of the current node.
    - `ctc_table`: A 2D array that contains the cost-to-come values for all nodes in the grid.
    
    Return:
    - The new cost-to-come value for `neighbor_coord`, incremented from the current node.

    Testing:
    --------
    - Test the functions by calculating the and cost-to-come between several points on a grid.
    - Verify that cost-to-come increments appropriately.
    """
    
    ctc = ctc_table[util.coord_to_index(current_node_coord)] + 1
    #ctc_table[neighbor_coord] = ctc_table[current_node_coord] + 1
    # ctc_table[neighbor_coord] = ctc
    #
    # *** DO STUFF HERE ***
    #
    
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
    
    Homework Instructions:
    ----------------------
    In this task, you will implement the logic for tracing the path from the goal node back to 
    the start node using a parent table. The goal node is passed to the function, and the task 
    is to trace backward through each node's parent until the start node is reached.
    
    1. **Retrieve the Parent Table**:
       - Map the key `'parent_table'` from the `state_dict` to a local variable `parent_table_coord`.
       - This table contains, for each node, the coordinates of its parent.
    
    2. **Initialize the Path Trace**:
       - Start by setting `current_node_coord` to the goal node's coordinates (`goal_node_coord`).
       - Create an empty list `path_list` to store the path as you trace backward.
    
    3. **Trace Back to the Start**:
       - Use a `while` loop to continue until you reach the start node.
       - At each iteration:
         - Add `current_node_coord` to `path_list`.
         - Convert the current node's coordinates to an index using the `util.coord_to_index()` function.
         - Use this index to find the parent node in the parent table and update `current_node_coord` 
           to this parent.
    
    4. **Exit Condition**:
       - The loop should stop when `worldmap.get_node_type_coord(current_node_coord)` returns 3, 
         indicating that the start node has been reached.
    
    5. **Return the Path**:
       - The path list should contain the coordinates from the goal node to the start node. Return `path_list`.
    
    Testing:
    --------
    - Make sure that the parent relationships are accurately followed and that the path correctly terminates at the start node.
    
    """
    parent_table_coord = state_dict['parent_table'] # map the parent_table key to the parent_table_coord variable
    
    current_node_coord = goal_node_coord # set the current_node_coord to the goal and work backwards from there
    path_list = [] # create an empty list
    while worldmap.get_node_type_coord(current_node_coord) != 3: # while we are not at the start
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
    
    
    '''
        This main routine sets up a grid-based path-planning simulation in a virtual environment using tools 
        like ZeroMQ (ZMQ) to communicate with the simulation software (CoppeliaSim in this case). It initializes 
        the environment, including the start and goal positions, creates a grid map with obstacles, and runs 
        a path-planning algorithm to find a path from the start to the goal. The path is then visualized and 
        executed in the simulation.
        
        Steps:
        1. Initialize Variables:
            - num_iterations: The maximum number of iterations for the path-planning algorithm.
            - inflation_amt: Defines how much to inflate obstacles in the grid (to account for robot size).
            - ZMQ connection: Establishes a remote API connection to the simulation environment (sim object).
            
            You can set num_iterations and inflation_amt
            
        2. Get the Start and Goal Positions:
            - Goal position: Queries the goal's position in the world frame (goal_world).
            - Start position: Queries the robot's start position in the world frame (start_world).
        
        3. Initialize the Grid Map:
            - Creates a gridmap object (worldmap) representing the environment's grid-based map, including 
              inflated obstacles, using goal_world and start_world.
        
        4. Declare State Variables:
            - open_set: A list of nodes that are open for exploration.
            - explored_set: A list of nodes that have already been explored.
            - parent_table_coord: A table that tracks the parent of each node for path backtracking.
            - ctg_table: Stores the cost-to-go (heuristic) values for each node.
            - ctc_table: Stores the cost-to-come (actual path cost) values for each node.
        
        5. Create the State Dictionary:
            - A dictionary (state_dict) groups key state variables like open_set, explored_set, parent_table, 
              world_map, ctg_table, and ctc_table, which is passed between functions for easier manipulation.
        
        6. Initial Visualization:
            - Closes previous plot windows and visualizes the world map in its normalized form, showing 
              obstacles and free space.
        
        7. Add the Start Point:
            - Adds the start point to the open_set and initializes the cost-to-go and cost-to-come values for 
              the start node in their respective tables.
        
        8. Main Path-Planning Loop:
            - Termination Condition: The loop runs until either:
                - A valid path is found (path_list is no longer None), or
                - The iteration count exceeds the set threshold (num_iterations).
            - Processing Nodes: At each iteration, process_next_node() is called to process the next node 
              from the open_set based on cost values and explore neighboring nodes. This continues until 
              the goal is found or the iteration limit is reached.
        
        9. Plot the Current State:
            - After path-planning is complete, it plots the explored nodes, open nodes, and the final path 
              for visualization.
        
        10. Generate Path in Simulation:
            - Path Transformation: Converts the path from grid coordinates to world coordinates.
            - Execute Path: The path is passed to the simulation (CoppeliaSim) where the robot is instructed 
              to follow the path by adjusting its track point.
        
        Summary of the Process:
        1. Setup: Initializes the environment and creates a grid-based map with obstacles.
        2. Path Planning: Uses an iterative algorithm to explore nodes and find a path from the start to the goal.
        3. Visualization: Visualizes the path-finding process and displays the explored grid.
        4. Simulation Execution: Once the path is found, it is executed in the simulation by having the robot 
           follow the path.
        
        Key Functions:
        - process_next_node(state_dict): Processes nodes in the open set, exploring neighbors and updating the state dictionary.
        - cost_to_go(start, goal): Calculates heuristic cost (Manhattan distance).
        - generate_path_from_trace(sim, path): Generates a smooth path for the robot in the simulation environment.
        - execute_path(path, sim, trackpoint, robot): Executes the path by moving the robot along the generated 
          path in the simulation.
    '''
    
    num_iterations = 5000 # iteration threshold
    inflation_amt = 2
    
    # Establish the ZMQ connection
    client = zmq.RemoteAPIClient()
    sim = client.getObject('sim')
    
    path_in_world_coords_xyz = get_path_points(sim, num_iterations=num_iterations,
                                               inflation_amt=inflation_amt)
    
    coppelia_path = util.generate_path_from_trace(sim, path_in_world_coords_xyz)
    trackpoint = sim.getObjectHandle("/track_point")
    robot = sim.getObjectHandle("/Pure_Robot/Dummy")
    util.execute_path(coppelia_path,sim,trackpoint,robot,thresh=0.1)

