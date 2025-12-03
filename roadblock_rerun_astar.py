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

    # allow headings in degrees: if magnitude looks large, convert to radians)
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

    # Re-initialize planning state in-place using the same worldmap
    open_set = state_dict['open_set']
    explored_set = state_dict['explored_set']
    parent_table = state_dict['parent_table']
    ctg_table = state_dict['ctg_table']
    ctc_table = state_dict['ctc_table']

    # clear previous search state
    open_set.clear()
    explored_set.clear()
    parent_table.fill(0)
    ctg_table.fill(0)
    ctc_table.fill(0)

    # re-seed start
    start = worldmap.start_grid_coord
    open_set.append(start)
    ctg_table[util.coord_to_index(start)] = cost_to_go(start, worldmap.goal_grid_coord)
    ctc_table[util.coord_to_index(start)] = 0

    # run planning loop
    path_list = None
    counter = 0
    while path_list is None and counter < num_iterations:
        path_list = process_next_node(state_dict)
        counter += 1

    # after replanning, plot current state
    try:
        state_dict['world_map'].plot_current_state(state_dict['explored_set'], state_dict['open_set'], path_list)
    except Exception:
        pass

    return path_list

# Example: how to call update_and_replan when a sensor detects a static obstacle
    # Uncomment and adapt the values below to test replanning at runtime.
    # sensor_obs = {'color':'red', 'heading':0.0, 'distance':1.0}  # heading relative to robot (radians)
    # new_path = encounter_roadblock_and_replan(sensor_obs, state_dict, sim, robot, worldmap)
    # if new_path is not None:
    #     print('Replanned path after sensor obstacle:', new_path)