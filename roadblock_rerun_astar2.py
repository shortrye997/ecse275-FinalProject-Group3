import coppeliasim_zmqremoteapi_client as zmq
import numpy as np
import finalprojectutils as util

# If encounter roadblock,
# Clear current path, add roadblock to map
# rerun a* algorithm with current position
def encounter_roadblock_and_replan(obstacle, state_dict, sim, robot, worldmap, num_iterations=5000):
    """
    Detects static obstacle (i.e roadblock) from a sensor (color, heading, distance),
    converts reading to a world coord, map it to the nearest grid cell,
    mark that cell as an obstacle in `worldmap`, and rerun the a* algorithm
    loop using the provided `state_dict`.

    Parameters:
    - obstacle: `'color'`, `'heading'` (radians or degrees), `'distance'`.
    - state_dict: the planning state dictionary used by the algorithm.
    - sim, robot, worldmap: objects used to query robot pose and convert coords.
    - num_iterations: iteration cap for replanning.

    Returns: path_list if found, otherwise None.
    """
    # Process only roadblocks (red obstacles)
    if obstacle.get('color', '') != 'red':
        print('obstacle color not red, ignoring')
        return None

    # get robot pose (pos, orient, yaw)
    try:
        robot_pos = sim.getObjectPosition(robot, sim.handle_world)
        robot_orient = sim.getObjectOrientation(robot, sim.handle_world)
        robot_yaw = robot_orient[2]
    except Exception as e:
        print(f'cannot find robot pose: {e}')
        return None

    # parse obstacle (head, dist)
    heading = obstacle.get('heading', 0.0)
    distance = obstacle.get('distance', 0.0)

    # allow headings in degrees by converting to radians if magnitude seems large
    if abs(heading) > 2 * np.pi:
        heading = np.deg2rad(heading)

    global_heading = robot_yaw + heading

    # compute world position of obstacle (xy)
    obs_x = robot_pos[0] + distance * np.cos(global_heading)
    obs_y = robot_pos[1] + distance * np.sin(global_heading)
    obstacle_world = np.array([obs_x, obs_y])
    print(f"obstacle world coords: {obstacle_world}")

    # Map the world coordinate to the nearest grid cell by querying all grid cells
    try:
        # build array of all grid coords (x,y) based on the normalized map shape
        H, W = worldmap.norm_map.shape
        grid_coords = np.array([(x, y) for y in range(H) for x in range(W)])
        # convert all grid coords to world coords using existing worldmap method
        try:
            world_coords = worldmap.grid_to_world(grid_coords)
        except Exception:
            # some implementations expect shape (N,2) with float dtype
            world_coords = worldmap.grid_to_world(np.array(grid_coords, dtype=float))

        # world_coords may be two or three columns, take first two columns
        world_coords_xy = np.array(world_coords)[:, :2]
        dists = np.linalg.norm(world_coords_xy - obstacle_world.reshape(1, 2), axis=1)
        idx = int(np.argmin(dists))
        obstacle_grid = tuple(grid_coords[idx])
        print(f"Mapped obstacle to grid cell: {obstacle_grid}")
    except Exception as e:
        print(f"failed to map world to grid: {e}")
        return None

    # Mark the grid cell as an obstacle in the worldmap normalized map
    try:
        # norm_map[row=y, col=x]
        worldmap.norm_map[obstacle_grid[1], obstacle_grid[0]] = 1
        # if there is a raw map attribute, try to update it as well
        if hasattr(worldmap, 'map'):
            try:
                worldmap.map[obstacle_grid[1], obstacle_grid[0]] = 1
            except Exception:
                pass
    except Exception as e:
        print(f"failed to mark obstacle in map: {e}")


    # choose an inflation amount for get_path_points (tunable)
    inflation_amt = 2

    try:
        path_world_xyz = get_path_points(sim, num_iterations, inflation_amt)
    except Exception as e:
        print(f"get_path_points failed: {e}")
        path_world_xyz = None

    if path_world_xyz is None:
        print('No path found.')
        return None

    # convert returned world xyz path to nearest grid cells in provided worldmap
    try:
        path_xy = np.array(path_world_xyz)[:, :2]

        # build array of all grid coords xy
        H, W = worldmap.norm_map.shape
        grid_coords = np.array([(x, y) for y in range(H) for x in range(W)])

        # convert grid coords to world coords for dist
        try:
            grid_world = np.array(worldmap.grid_to_world(grid_coords))
        except Exception:
            grid_world = np.array(worldmap.grid_to_world(np.array(grid_coords, dtype=float)))

        grid_world_xy = grid_world[:, :2]

        path_grid = []
        for p in path_xy:
            dists = np.linalg.norm(grid_world_xy - p.reshape(1, 2), axis=1)
            idx = int(np.argmin(dists))
            path_grid.append(tuple(grid_coords[idx]))

        # remove consecutive duplicates
        reduced_path = []
        for pt in path_grid:
            if not reduced_path or reduced_path[-1] != pt:
                reduced_path.append(pt)

        print(f"Replanned path (grid coords): {reduced_path}")
        return reduced_path
    except Exception as e:
        print(f"failed to convert world path to grid: {e}")
        return None

# Example: how to call encounter_roadblock_and_replan when a sensor detects a static obstacle
    # Uncomment and adapt the values below to test replanning at runtime.
#sensor_obs = {'color':'red', 'heading':0.0, 'distance':1.0}  # heading relative to robot (radians)
#new_path = encounter_roadblock_and_replan(sensor_obs, state_dict, sim, robot, worldmap)
#if new_path is not None:
    #print('Replanned path after sensor obstacle:', new_path)