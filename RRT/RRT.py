# Imports
import numpy as np
import cv2
import matplotlib.pyplot as plt
import random

# Read Image
grid_map = cv2.imread('./map/vivocity_freespace.png')
grid_map = cv2.cvtColor(grid_map, cv2.COLOR_BGR2GRAY)

# Define the locations
locations = {
    'start': (345, 95),
    'snacks': (470, 475),
    'store': (20, 705),
    'movie': (940, 545),
    'food': (535, 800)
}

# A helper function to mark the locations on the map
def plot_locations(locations: dict, color: 'str'='black'):
    for key, value in locations.items():
        plt.plot(locations[key][0], locations[key][1], marker="o", markersize=10, markeredgecolor="red")
        plt.text(locations[key][0], locations[key][1]-15, s=key, fontsize='x-large', fontweight='bold', c=color, ha='center')
    return

# Tree Node
class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None

# RRT Algorithm
class RRT():
    def __init__(self,
                 start,
                 goal,
                 iters,
                 grid,
                 step):
        
        self.randomTree = treeNode(start[0], start[1])
        self.goal = treeNode(goal[0], goal[1])
        self.nearest_node = None
        self.iters = min(iters, 10000)
        self.grid = grid
        self.step = step
        self.path_dist = 0
        self.nearest_dist = 10000
        self.num_waypoints = 0
        self.waypoints = []
        self.grid_shape = (1000, 1000)
    
    def add_child(self, locationX, locationY):
        if (locationX == self.goal.locationX):
            self.nearest_node.children.append(self.goal)
            self.goal.parent = self.nearest_node
        else:
            temp_node = treeNode(locationX, locationY)
            self.nearest_node.children.append(temp_node)
            temp_node.parent = self.nearest_node

    # Sample a random point within the grid
    def sample_point(self):
        x = random.randint(1, 1000)
        y = random.randint(1, 1000)
        point = np.array([x, y])
        return point

    # Steer a distance step_size from start to end location
    def steer_to_point(self, location_start, location_end):
        offeset = self.step * self.unit_vector(location_start, location_end)
        point = np.array([location_start.locationX + offeset[0], location_start.locationY + offeset[1]])
        if point[0] >= self.grid_shape[1]:
            point[0] = self.grid_shape[1] - 1
        if point[1] >= self.grid_shape[0]:
            point[1] = self.grid_shape[0] - 1
        return point

    # Check for obstacle
    def if_in_obstacle(self, location_start, location_end):
        u_hat = self.unit_vector(location_start, location_end)
        test_point = np.array([0.0, 0.0])
        for i in range(self.step):
            test_point[0] = location_start.locationX + i * u_hat[0]
            test_point[1] = location_start.locationY + i * u_hat[1]

            # Check if point lies inside of the obstacle
            if self.grid[int(round(test_point[1])), int(round(test_point[0]))] == 0:
                return True
        return False


    # Unit vector between two points which form a vector
    def unit_vector(self, location_start, location_end):
        v = np.array([location_end[0] - location_start.locationX, location_end[1] - location_start.locationY])
        u_hat = v / (np.linalg.norm(v))
        return u_hat

    # find the nearest node from a given unconnected point 
    def find_nearest(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        if dist <= self.nearest_dist:
            self.nearest_node = root
            self.nearest_dist = dist

        # Use recursion to get the closest node
        for child in root.children:
            self.find_nearest(child, point)
        pass

    # find distance between a node and a point
    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0])**2 + (node1.locationY - point[1])**2)
        return dist

    # Check if goal has been reached within step size
    def goalFound(self, point):
        if self.distance(self.goal, point) <= self.step:
            return True
        pass

    # reset nearest_node and nearest_distance
    def reset_nearest_node_distance(self):
        self.nearest_node = None
        self.nearest_dist = 10000

    # Trace the path
    def trace_path(self, goal):
        if goal.locationX == self.randomTree.locationX:
            return
        self.num_waypoints += 1

        current_point = np.array([goal.locationX, goal.locationY])
        self.waypoints.insert(0, current_point)
        self.path_dist += self.step
        self.trace_path(goal.parent)

# Run the algorithm
start = np.array(locations['start'])
goal = np.array(locations['movie'])
iters = 10000
step = 50

fig = plt.figure(figsize=(10, 10), dpi=80)
plt.imshow(grid_map, cmap='gray')
plot_locations
ax = fig.gca()
#ax.add_patch(goal)
plt.xlabel('X-axis')
plt.ylabel('Y-axis')

rrt = RRT(start, goal, iters=iters, grid=grid_map, step=step)

for i in range(rrt.iters):
    # Reset the values
    rrt.reset_nearest_node_distance()
    print("Iterations: ", i)

    point = rrt.sample_point()
    rrt.find_nearest(rrt.randomTree, point)
    new = rrt.steer_to_point(rrt.nearest_node, point)
    flag = rrt.if_in_obstacle(rrt.nearest_node, new)
    if (flag == False):
        rrt.add_child(new[0], new[1])
        plt.pause(0.1)
        plt.plot([rrt.nearest_node.locationX, new[0]], [rrt.nearest_node.locationY, new[1]], 'go', linestyle='--')

        # If goal found, add to path
        if (rrt.goalFound(new)):
            rrt.add_child(goal[0], goal[1])
            print("Reached goal")
            break

# Trace the path back
rrt.trace_path(rrt.goal)
rrt.waypoints.insert(0, start)

# Plot waypoints
for i in range(len(rrt.waypoints)-1):
    plt.plot([rrt.waypoints[i][0], rrt.waypoints[i+1][0]], [rrt.waypoints[i][1], rrt.waypoints[i+1][1]], 'ro', linestyle="--")
    plt.pause(0.1)

# Keep the window open
plt.show(block=True)