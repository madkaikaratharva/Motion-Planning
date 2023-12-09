import numpy as np
from IPython.display import display, clear_output
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

# Heurestic Cost Function
def heurestic(current_node, goal_node, Astar=True):

    if Astar:
        h = 1.5*(np.linalg.norm(np.array(current_node) - np.array(goal_node)))
        return h
    else:
        return 0
    

# Grid occupancy check Function
def occupied(node, grid_map):

    return grid_map[node[0],node[1]] != 255


# Check for Invalid Grid Points
def Invalid(node, grid_map):

    upper_bound = np.array([grid_map.shape[0] - 1, grid_map.shape[1] - 1])
    lower_bound = np.array([0, 0])

    node = np.array(node)

    x = node < lower_bound   # if node=[[-1, 5]], returns [[1,0]] else returns [[0,0]] if point inside the grid
    y = node > upper_bound   # if node=[[560, 1001]], returns [[0,1]] else returns [[0,0]] if point inside the grid

    # Returns 0 if node considered lies inside the grid
    if np.sum(x) or np.sum(y):
        return True
    

# Check for Robot's footprint
def space_available(node, grid_map):

    nearest_points = [[0,3], [-3,0], [3,0], [0,-3]]     # Considers 4 neighboring points at a distance of 3 units
    for point in nearest_points:
        neigbor_point = tuple(np.array(node) + np.array(point))
        if grid_map[neigbor_point[0],neigbor_point[1]] != 255:
            return False
    
    return True


# Plot and Visualize the Path
def visualize_path(floor_plan, visited_grids, trajectory, start_node, goal_node, delay=0.1):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(floor_plan, cmap='gray')

    # Plot visited grids
    visited_xs, visited_ys = zip(*visited_grids)
    ax.scatter(visited_xs, visited_ys, color='orange', marker='o', label='Visited Grids')

    # Plot the trajectory
    traj_xs, traj_ys = zip(*trajectory)
    ax.plot(traj_xs, traj_ys, color='blue', linewidth=2, label='Trajectory')

    # Plot start and goal nodes
    ax.scatter([start_node[0], goal_node[0]], [start_node[1], goal_node[1]], color=['green', 'red'], marker='o', label=['Start', 'Goal'])

    ax.legend()
    plt.show(block=False)

    # Pause and clear the output for dynamic visualization
    plt.pause(delay)
    clear_output(wait=True)




