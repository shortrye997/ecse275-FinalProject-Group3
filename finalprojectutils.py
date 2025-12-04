# -*- coding: utf-8 -*-
"""
Created on Sat Nov 29 20:36:31 2025

@author: ched0
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul  1 20:31:06 2023

@author: zxc703
"""


import coppeliasim_zmqremoteapi_client as zmq
import matplotlib.pyplot as plt
import numpy as np

class gridmap:
    
    '''
    # gridmap class generates a 2D grid map from vision sensor data in a simulation,
    # inflates obstacles, normalizes the map, and provides methods for plotting 
    # and manipulating grid and world coordinates.
    
    A class to represent a grid map for navigation and obstacle inflation.
    
    Accessible Variables:
    ---------------------
    handle : int
        The handle for the vision sensor in the simulation.
    gridmap : numpy array
        The 2D grid representing the map with obstacles and free space.
    resolution : tuple
        Resolution of the grid map in pixels.
    world_size : float
        The size of the world in world coordinates.
    scaling : float
        Scaling factor to convert between grid coordinates and world coordinates.
    offset : numpy array
        Offset applied during coordinate conversion.
    norm_map : numpy array
        The normalized grid map, where obstacles are marked as 1, free space as 0. It can be further augmented with goal and start positions, marked as 2 and 3 respectively
    goal_grid_coord : tuple
        The grid coordinates of the goal point.
    start_grid_coord : tuple
        The grid coordinates of the start point.
    obs_world : numpy array
        The world coordinates of the detected obstacles.
    
    Methods:
    --------
    __init__(sim, world_size, goal_world_coord, start_world_coord, inflate_iter=1, name="/world_camera", robot_name="/Pure_Robot")
        Initializes the gridmap object.
    
    inflate_obstacles(num_iter=1, obs_thresh=100, infl_val=99)
        Inflates obstacles on the grid map.
    
    get_grid_coords(point_xyz)
        Converts world coordinates to grid coordinates.
    
    get_world_coords(point_xyz)
        Converts grid coordinates back to world coordinates.
    
    get_obs_in_world_coords(plot=False)
        Retrieves obstacle points in world coordinates and optionally plots them.
    
    plot(normalized=False)
        Plots the grid map, either in normalized or original form.
    
    plot_coord(point_xy, color='blue')
        Plots a specific coordinate point on the grid map.
    
    normalize_map()
        Normalizes the grid map by converting obstacles to 1 and free space to 0.
    
    add_goal_coord(goal_coord)
        Marks the goal location on the normalized map.
    
    add_start_coord(start_coord)
        Marks the start location on the normalized map.
    
    get_node_type_coord(node_coord)
        Returns the type of node (e.g., obstacle, free space, goal, start) at the given coordinate.
    
    plot_current_state(explored_set, open_set, path_list)
        Plots the current state of the grid, including explored nodes, open nodes, and the path.
    
    '''
    
    def __init__(self,sim,world_size,goal_world_coord,start_world_coord,inflate_iter = 1,name="/world_camera",robot_name="/Pure_Robot"):
        
        """
        Initializes the gridmap object.
        
        Parameters:
        sim (object): Simulation environment instance (e.g., from an API).
        world_size (float): Size of the world in world coordinates.
        goal_world_coord (tuple): Coordinates of the goal (x, y, z) in world space.
        start_world_coord (tuple): Coordinates of the start (x, y, z) in world space.
        inflate_iter (int, optional): Number of iterations for obstacle inflation (default is 1).
        name (str, optional): Name of the vision sensor (default is "/world_camera").
        robot_name (str, optional): Name of the robot in the simulation (default is "/Pure_Robot").
        """
        
        self.handle = sim.getObjectHandle(name)
        robot_handle = sim.getObjectHandle(robot_name)
        #robot_pos = sim.getObjectPosition(robot_handle,-1)
        robot_pos = sim.getObjectPosition(robot_handle, sim.handle_world)
        #sim.setObjectPosition(robot_handle,-1,(100,100,100))
        self.image,self.resolution = sim.getVisionSensorImg(self.handle,1)
        #sim.setObjectPosition(robot_handle,-1,robot_pos)
        self.gridmap = np.array(bytearray(self.image),dtype='uint8').reshape(self.resolution[0],self.resolution[1])    
        
        self.world_size = world_size
        self.scaling = world_size/self.resolution[0]
        self.offset = np.array([self.resolution[0]/2,self.resolution[1]/2,0])
        self.norm_map = None
        
        self.inflate_obstacles(num_iter=inflate_iter)
        self.normalize_map()
        
        #self.goal_grid_coord = self.get_grid_coords(goal_world_coord)
        print(f"goal_world_coord: {goal_world_coord}")
        self.goal_grid_coord = self.world_to_grid(goal_world_coord)
        self.goal_grid_coord_old = self.get_grid_coords(goal_world_coord)
        print(f"goal_grid_coord: {self.goal_grid_coord}")
        print(f"goal_grid_coord_old: {self.goal_grid_coord_old}")
        bruh = int(self.goal_grid_coord[0])
        bruh2 = int(self.goal_grid_coord[1])
        self.add_goal_coord(self.goal_grid_coord)
        #self.add_goal_coord((bruh, bruh2, 0))
        #self.add_goal_coord(self.goal_grid_coord_old)
        
        #self.start_grid_coord = self.get_grid_coords(start_world_coord)
        self.start_grid_coord = self.world_to_grid(start_world_coord)
        print(f"start_grid_coord: {self.start_grid_coord}")
        cri = int(self.start_grid_coord[0])
        cri2 = int(self.start_grid_coord[1])
        cri3 = int(self.start_grid_coord[2])
        self.add_start_coord(self.start_grid_coord)
        #self.add_start_coord((cri, cri2, cri3))
        
        #goal_grid_coord and start_grid_coord, add_start_coord
    
    def world_to_grid(self,point_world):
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
        grid_x = int(-x * 2 + 4)
        grid_y = int(y * 2 + 4)
        
        # Decide what to do with Z:
        # Option A: keep as-is (continuous world → continuous grid)
        grid_z = z
    
        return (grid_x, grid_y, grid_z)
    
    def grid_to_world(self,point_grid):
        """
        Parameters
        ----------
        point_grid : TYPE
            DESCRIPTION.

        Returns
        -------
        
        Wx=2-0.5Mx
        Wy=-2+0.5My
        """
        """
        pos = (2 - 0.5*point_grid[0], -2 + 0.5*point_grid[1])
        
        return tuple(pos)
    """
        # 1. Use NumPy indexing to unpack the columns for vectorized operation
        # M_x is the first column (index 0) of all rows
        M_x = point_grid[:, 0]
        # M_y is the second column (index 1) of all rows
        M_y = point_grid[:, 1]
        
        # 2. Apply the documented transformations:
        # Wx = 2 - 0.5 * Mx
        W_x = 2.0 - 0.5 * M_x
        
        # Wy = -2 + 0.5 * My
        W_y = -2.0 + 0.5 * M_y
        
        # 3. Combine Wx and Wy into an N x 2 array and return it
        # We use np.stack or np.column_stack to combine the two columns back together.
        return np.column_stack((W_x, W_y))
    

        
    
    def get_grid_coords(self,point_xyz):
        """
        Converts world coordinates to grid coordinates.

        Parameters:
        point_xyz (tuple or array): World coordinates (x, y, z).

        Returns:
        tuple: Grid coordinates.
        """
        # takes the x,y coordinates in from the world frame and scales it to the (x,y) coordinates of the grid world
        
        try:
            pos = np.round(np.array(point_xyz)/self.scaling+self.offset).astype(int)[:,0:2]
        except:
            pos = np.round(np.array(point_xyz)/self.scaling+self.offset).astype(int)[0:2]
        
        return tuple(pos)
        
    def get_world_coords(self,point_xyz):
        """
        Converts grid coordinates back to world coordinates.

        Parameters:
        point_xyz (tuple or array): Grid coordinates (x, y, z).

        Returns:
        tuple: World coordinates.
        """
        # takes the x,y coordinates from the grid world and scales it back to x,y coordinates of the world frame.
        try:
            pos = (np.array(point_xyz)-self.offset)*self.scaling
        except:
            pos = (np.array(point_xyz)-self.offset[:2])*self.scaling
        
        return tuple(pos)
    
    
    def get_obs_in_world_coords(self,plot=False):
        """
        Retrieves obstacle points in world coordinates and optionally plots them.

        Parameters:
        plot (bool, optional): Whether to plot the obstacle points (default is False).
        """
        if self.norm_map is not None:
            obs_points = np.argwhere(self.norm_map==1)
        else:
            print("norm map doesn't exist!")
        #self.obs_world = self.get_world_coords(np.hstack((obs_points,np.zeros((obs_points.shape[0],1)))))
        #self.obs_world = np.array(self.get_world_coords(obs_points))
        self.obs_world = self.grid_to_world(np.hstack((obs_points,np.zeros((obs_points.shape[0],1)))))
        self.obs_world_old = self.get_world_coords(np.hstack((obs_points,np.zeros((obs_points.shape[0],1)))))
        print(f"self.obs_world: {self.obs_world}")
        print(f"self.obs_world_old: {self.obs_world_old}")
        self.obs_world = np.array(self.grid_to_world(obs_points))
        
        if plot:
            plt.plot(self.obs_world[:,1],self.obs_world[:,0],'.')    
            #plt.plot(obs_points[:,1],obs_points[:,0],'.') 
            plt.gca().set_aspect('equal')
        
    def add_goal_coord(self,goal_coord):
        """
        Marks the goal location on the normalized map.

        Parameters:
        goal_coord (tuple): Goal coordinates in grid space.
        """
        print(f"goal_coord: {goal_coord}")
        goal_idx = coord_to_index(goal_coord)
        print(f"goal_idx: {goal_idx}")
        self.norm_map[goal_idx[0],goal_idx[1]] = 2
        print('goal point added at index: ' + str(goal_idx[0]) + ',' + str(goal_idx[1]))
        
    def add_start_coord(self,start_coord):
        """
        Marks the start location on the normalized map.

        Parameters:
        start_coord (tuple): Start coordinates in grid space.
        """
        #start_idx = coord_to_index(start_coord)
        start_idx = coord_to_index(start_coord)
        self.norm_map[start_idx[0],start_idx[1]] = 3
        print('start point added at index: ' + str(start_idx[0]) + ',' + str(start_idx[1]))
        
    def get_node_type_coord(self,node_coord):
        """
        Returns the type of node at the given coordinate (e.g., obstacle, free space, goal, or start).

        Parameters:
        node_coord (tuple): Coordinate of the node.

        Returns:
        int: Type of node (1 = obstacle, 0 = free space, etc.).
        """
        if node_coord[0] < 0 or node_coord[1] < 0:
            raise ValueError("Indexing a negative coordinate")
        
        node_idx = coord_to_index(node_coord)
        node_type = self.norm_map[node_idx]
        
        return node_type

    def inflate_obstacles(self,num_iter=1,obs_thresh = 100,infl_val = 99):
        
        """
        Inflates obstacles by increasing their size on the grid map.

        Parameters:
        num_iter (int, optional): Number of inflation iterations (default is 1).
        obs_thresh (int, optional): Threshold to detect obstacles (default is 100).
        infl_val (int, optional): Value assigned to inflated obstacles (default is 99).

        Returns:
        numpy array: The inflated gridmap.
        """
        
        rows = self.gridmap.shape[0]
        cols = self.gridmap.shape[1]
        inflated_grid = np.copy(self.gridmap)
        
        # Define the possible movements (up, down, left, right, and diagonals)
        movements = [
            (0, -1),  # Up
            (0, 1),   # Down
            (-1, 0),  # Left
            (1, 0),   # Right
            (-1, -1), # Diagonal: Up-Left
            (-1, 1),  # Diagonal: Up-Right
            (1, -1),  # Diagonal: Down-Left
            (1, 1)    # Diagonal: Down-Right
        ]
        
        # Iterate through the grid
        for i in range(num_iter):
            inflated_temp = np.copy(inflated_grid)
            for row in range(rows):
                for col in range(cols):
                    if inflated_grid[row][col] < obs_thresh:  # Found an obstacle
                        for move in movements:  # Inflate the obstacle
                            new_row = row + move[0]
                            new_col = col + move[1]
                            if 0 <= new_row < rows and 0 <= new_col < cols and inflated_grid[new_row][new_col] > 200:
                                inflated_temp[new_row][new_col] = infl_val
            inflated_grid = np.copy(inflated_temp)
            self.gridmap = np.copy(inflated_grid)
            
        return self.gridmap
    
    def normalize_map(self):
        """
        Normalizes the grid map. Converts obstacles to 1 and free space to 0.
        """
        
        gridmap_temp = np.copy(self.gridmap)
        gridmap_temp[gridmap_temp>99] = 255
        self.norm_map =(gridmap_temp/255).astype(int)
        self.norm_map[self.norm_map==0] = 3 # temporarily assign obstacles as 3
        self.norm_map[self.norm_map==1] = 0 # set free space to 0s
        self.norm_map[self.norm_map==3] = 1 # convert obstacle back into 1s

    def plot(self,normalized=False):
        """
        Plots the grid map.

        Parameters:
        normalized (bool, optional): If True, plots the normalized map (default is False).
        """
        if normalized:
            plt.imshow(self.norm_map,cmap='binary',origin="lower")
        else:
            plt.imshow(self.gridmap,cmap='gray', vmin=0, vmax=255,origin="lower")
            
        self.plot_coord(self.goal_grid_coord)
        self.plot_coord(self.start_grid_coord,color='red')
            
    def plot_coord(self,point_xy,color='blue'):
        """
        Plots a specific coordinate point on the grid map.

        Parameters:
        point_xy (tuple): The coordinate to plot.
        color (str, optional): Color of the point (default is 'blue').
        """    
        plt.plot(point_xy[0],point_xy[1],'.',markersize=5,color=color)

    def plot_current_state(self,explored_set,open_set,path_list):
        """
        Plots the current state of the grid, including explored nodes, open nodes, and the path.
        
        Parameters:
        explored_set (list): List of explored grid coordinates.
        open_set (list): List of open grid coordinates.
        path_list (list): List of grid coordinates representing the path.
        """
        print(f"plot_current_state: explored_set: {explored_set}")
        explored_set = np.array(explored_set)
        open_set = np.array(open_set)
        try:
            plt.plot(explored_set[:,0],explored_set[:,1],'.',markersize=5,color='black')
        except:
            pass    
        try:
            plt.plot(open_set[:,0],open_set[:,1],'.',markersize=5,color='green')
        except:
            pass
        try:
            path_list = np.array(path_list)
            plt.plot(path_list[:,0],path_list[:,1],'.',markersize=5,color='yellow')
        except:
            pass    
        
def coord_to_index(coord):
    """
    Converts a 2D coordinate to a grid index.
    
    Parameters:
    coord (tuple): A tuple representing a coordinate (x, y).
    
    Returns:
    tuple: A tuple representing the index (row, column), where y corresponds to row and x corresponds to column.
    """
    idx = [coord[1],coord[0]] # the index is specified as row by height, so the y by x.
    #idx = [coord[0],coord[1]]
    
    return tuple(idx)   

def index_to_coord(idx):
    """
    Converts a grid index back to a 2D coordinate.

    Parameters:
    idx (tuple): A tuple representing an index (row, column).

    Returns:
    tuple: A tuple representing the coordinate (x, y), where row corresponds to y and column corresponds to x.
    """
    coord = [idx[1],idx[0]] # the index is specified as row by height, so the y by x.
    
    return tuple(coord)  
        
def generate_path_from_trace(sim,trace_path, num_smoothing_points=100):
    """
    Generates a path from a given trace of points and returns the smoothed path data from the simulator.
    
    Parameters:
    sim (object): The simulation environment instance.
    trace_path (numpy array): A Nx3 array representing the trace of points, already scaled to world coordinates.
    num_smoothing_points (int, optional): Number of smoothing points for the path (default is 100).
    
    Returns:
    numpy array: Smoothed path data as a Nx7 array, retrieved from the simulator.
    """
    # path must be Nx3 and already scaled back to the world coords
    
    n,m = trace_path.shape
    
    trace_path = np.hstack((trace_path,np.zeros((n,3)),np.ones((n,1))))
    
    path = np.array(trace_path).astype(float)
    #path_handle = sim.createPath(list(path.reshape(-1)),28,num_smoothing_points,1.0,0,[1.0,0.0,0.0])
    path_handle = sim.createPath(list(path.reshape(-1)),16,num_smoothing_points,1.0,0,[1.0,0.0,0.0])
    #pathShape = sim.getObjectHandle("/Path/shape")
    #sim.setShapeColor(pathShape,"",sim.colorcomponent_ambient_diffuse,[1.0,0.0,0.0])
    
    # obtain the smoothed path data from coppeliasim
    pathData=sim.unpackDoubleTable(sim.readCustomDataBlock(path_handle,'PATH'))
    pathData_array = np.array(pathData).reshape((-1,7))
    
    return pathData_array

def execute_path(pathData_array,sim,trackpoint_handle,robot_handle,thresh=0.15):
    """
    Executes the path by moving the robot through the points specified in the path data array.
    
    Parameters:
    pathData_array (numpy array): Array of Nx7 path points to follow.
    sim (object): The simulation environment instance.
    trackpoint_handle (int): Handle to the tracking point object.
    robot_handle (int): Handle to the robot object.
    thresh (float, optional): Threshold distance to the next point before switching (default is 0.1).
    
    While the robot is moving:
    - Sets the track point position to the next point in the path.
    - Gets the current robot position and compares it to the track point.
    - If the robot is within the threshold distance to the track point, moves to the next path point.
    """
    path_index = 1
    timer = 0
    while path_index<=pathData_array.shape[0]:
        # set the track point pos
        target_point = pathData_array[-path_index,:]
        #print(f"target_point: {target_point}")
        if any(np.isnan(target_point)):
            target_point[3:] = [0.0,0.0,0.0,1.0]
        sim.setObjectPose(trackpoint_handle,sim.handle_world,list(pathData_array[-path_index,:]))
        print(list(pathData_array[-path_index,:]))
        # get the current robot position
        robot_pos = sim.getObjectPosition(robot_handle,sim.handle_world)
        parent = sim.getObjectParent(robot_handle)
        robot_pos_parent = sim.getObjectPosition(robot_handle,parent)
        trackpt_pos = sim.getObjectPosition(trackpoint_handle,sim.handle_world)
        robot_pos_grid = world_to_grid_a(robot_pos)
        trackpt_pos_grid = world_to_grid_a(trackpt_pos)
        print(f"robot_handle: {robot_handle}")
        print(f"robot_pos: {robot_pos}")
        print(f"robot_pos_parent: {robot_pos_parent}")
        print(f"robot_pos_grid: {robot_pos_grid}")
        print(f"trackpt_pos: {trackpt_pos}")
        print(f"trackpt_pos_grid: {trackpt_pos_grid}")
        print(f"np.array(robot_pos): {np.array(robot_pos)}")
        print(f"np.array(trackpt_pos): {np.array(trackpt_pos)}")
        parent = sim.getObjectParent(trackpoint_handle)
        print("Parent:", parent)
        # compute the distance between the trackpt position and the robot
        rob_trackpt_dist = np.linalg.norm(np.array(robot_pos)-np.array(trackpt_pos))
        print(f"rob_trackpt_dist: {rob_trackpt_dist}")
        print(f"rob_trackpt_dist < thresh: {rob_trackpt_dist < thresh}")
        #print(f"execute_path rob_trackpt_dist: {rob_trackpt_dist}")
        
        if (rob_trackpt_dist < thresh) or (timer > 40):
            path_index = path_index + 1
            print("next_point")
            timer = 0
            
        else:
            timer = timer + 1

def move_to_next_point(start, end):
    
    
    """
def execute_path_fp(path_list, sim, trackpoint_handle, robot_handle, thresh=0.1):
    path_index = 1
    while path_index <= len(path_list):
        # set the track point pos
        target_point = pathData_array[-path_index,:]
        if any(np.isnan(target_point)):
            target_point[3:] = [0.0,0.0,0.0,1.0]
        sim.setObjectPose(trackpoint_handle,sim.handle_world,list(pathData_array[-path_index,:]))
        # get the current robot position
        robot_pos = sim.getObjectPosition(robot_handle,sim.handle_world)
        trackpt_pos = sim.getObjectPosition(trackpoint_handle,sim.handle_world)
        # compute the distance between the trackpt position and the robot
        rob_trackpt_dist = np.linalg.norm(np.array(robot_pos)-np.array(trackpt_pos))
        print(rob_trackpt_dist)
        if rob_trackpt_dist < thresh:
            path_index = path_index + 1
            print("next_point")
            """
        
        
    
    
def visualize_potential_field_2D(world_u):
    '''
    

    Parameters
    ----------
    world_u : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    '''
    # Create the heatmap plot using imshow()
    plt.imshow(world_u, cmap='hot', origin='lower')
    # Add colorbar
    plt.colorbar()
    # Add labels and title
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('2D Heatmap Plot')
    # Show the plot
    plt.show()
    
def visualize_potential_field_3D(world_u):
    '''
    

    Parameters
    ----------
    world_u : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    '''
    
    grid_height,grid_width = world_u.shape
    
    # Generate data for the surface plot
    x = np.linspace(0, grid_width-1, grid_width)
    y = np.linspace(0, grid_height-1, grid_height)
    X, Y = np.meshgrid(x, y)
    Z = world_u

    # Create a 3D figure
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the surface
    surf = ax.plot_surface(X, Y, Z, cmap='viridis')

    # Add labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Surface Plot')

    # Add a colorbar
    fig.colorbar(surf)

    # Show the plot
    plt.show()
    
def discrete_grad_descent(start,end,grad,step=1,max_iter=2000):
    '''
    

    Parameters
    ----------
    start : TYPE
        DESCRIPTION.
    end : TYPE
        DESCRIPTION.
    grad : TYPE
        DESCRIPTION.
    step : TYPE, optional
        DESCRIPTION. The default is 1.
    max_iter : TYPE, optional
        DESCRIPTION. The default is 500.

    Returns
    -------
    TYPE
        DESCRIPTION.

    '''
    current_point = np.flip(start)
    end = np.flip(end)
    point_list = []
    n = 0
    point_list.append(current_point)
    
    height, width = grad[0,:,:].shape
    
    print("start_point (y,x): " + str(current_point))
    while ((current_point[0] != end[0]) or (current_point[1]!=end[1])) and (n<=max_iter):
        print("iteration: " + str(n))
        #get gradient at the point:
        point_grad = -grad[:,int(current_point[0]),int(current_point[1])]
        print(point_grad)
        #point_grad = np.round(point_grad)
        mask = np.abs(point_grad)>0
        point_grad[mask] = point_grad[mask]/np.abs(point_grad[mask])
        print(point_grad)
        
        next_point = current_point+point_grad*step
        print(next_point)
        if (next_point[0] > height-1 or next_point[0]<0) or (next_point[1] > width-1 or next_point[1]<0) :
            pass
        else:
            current_point = next_point
            
        print("current_point (y,x): " + str(current_point))
        point_list.append(current_point)
        n=n+1

    return np.array(point_list)

def compute_discrete_gradient(world_u):
    '''
    

    Parameters
    ----------
    world_u : TYPE
        DESCRIPTION.

    Returns
    -------
    world_ugrad : TYPE
        DESCRIPTION.

    '''
    
    world_ugrad = np.gradient(world_u)
    
    world_ugrad = np.array(world_ugrad)
    
    return world_ugrad

def visualize_gradient(world,world_ugrad,goal,step=5):
    '''
    

    Parameters
    ----------
    world : TYPE
        DESCRIPTION.
    world_ugrad : TYPE
        DESCRIPTION.
    start : TYPE
        DESCRIPTION.
    goal : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    '''
    
    plt.figure(figsize=(6, 6))
    plt.imshow(world, cmap='binary', origin='lower',aspect="equal")
    # plt.xticks(range(grid_width))
    # plt.yticks(range(grid_height))
    # plt.grid(color='black', lw=1)
    plt.plot(goal[0],goal[1],'.',markersize=10,color="red")
    #plt.plot(start[0],start[1],'.',markersize=10,color="blue")
    grid_height,grid_width = world.shape
    x = np.linspace(0, grid_width-1, grid_width)
    y = np.linspace(0, grid_height-1, grid_height)
    X, Y = np.meshgrid(x, y)
    plt.quiver(X[::step,::step],Y[::step,::step], -world_ugrad[::step,::step,1], -world_ugrad[::step,::step,0])
    
    
def plot_gradient_descent(fig,path):
    '''    

    Parameters
    ----------
    fig : TYPE
        DESCRIPTION.
    path : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    '''
    
    fig.plot(path[:,1],path[:,0],'-')     
    fig.plot(path[:,1],path[:,0],'.',markersize=5)    
    plt.gca().set_aspect('equal')
    
def world_to_grid_a(point_world):
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
    grid_x = int(-x * 2 + 4)
    grid_y = int(y * 2 + 4)
    
    # Decide what to do with Z:
    # Option A: keep as-is (continuous world → continuous grid)
    grid_z = z

    return (grid_x, grid_y, 0)