import numpy as np
from heapq import heappush, heappop
from utils import heurestic, occupied, Invalid, space_available

def graphsearch(grid, resolution, start, goal, Astar=True):

    # Initialize Params
    grid_map = grid.transpose()    
    start_node = tuple(start)
    goal_node =  tuple(goal)

    try:
        path = []
        opened_list = []
        closed_list = set()
        parents = {}

        # Create (nxn) numpy array to store G-cost g(n) of visited grid points. Initialize the costs to infinity
        g_cost = np.ones(grid_map.shape)*np.inf

        # Initializes G-cost of start node to 0
        g_cost[start_node[0],start_node[1]] = 0 

        if Astar:
            heappush(opened_list, (heurestic(start_node, goal_node), start_node))
        else:
            heappush(opened_list, (np.linalg.norm(np.array(start_node) - np.array(goal_node)), start_node))

        # Define 8-neighbors
        neighbors = [[-1,1],  [0,1],  [1,1],
                     [-1,0],          [1,0],
                     [-1,-1], [0,-1], [1,-1]] 
    
        # Keep Iterating unitil opened_list has elements
        while len(opened_list) != 0:

            # Pick the node with smallest cost as current_node. Pop it out and add it to closed_list to prevent revisitings
            current_node_cost, current_node = heappop(opened_list)
            if current_node in closed_list:
                continue
            if current_node == goal_node:
                break 

            closed_list.add(current_node)

            # Evalutes the best neighbor node for the current node
            for one_neighbor in neighbors:
                neighbor = tuple(np.array(current_node) + np.array(one_neighbor))

                # Checks Invalid points, occupied points, availability of space and whether the point has been visited before
                if not Invalid(neighbor, grid_map) and not occupied(neighbor, grid_map)and space_available(neighbor, grid_map) and not neighbor in closed_list:
                    neighbor_g_cost = g_cost[current_node[0], current_node[1]] + np.linalg.norm(np.array(neighbor) - np.array(current_node))

                    if neighbor_g_cost < g_cost[neighbor[0], neighbor[1]]:
                        heappush(opened_list, (neighbor_g_cost + heurestic(neighbor, goal_node, Astar), neighbor))

                        g_cost[neighbor[0], neighbor[1]] = neighbor_g_cost
                        parents[tuple(neighbor)] = current_node

        # Distance travelled will be the G_cost g(n) of the goal_node multiplied by resolution
        distance_travelled = g_cost[goal_node[0], goal_node[1]]*resolution

        # Trace the path from the goal node to the start node by tracing parents of each subsequent node
        if parents.get(tuple(goal_node)):
            node = goal_node
            while node != start_node:
                path.append(node)
                node = parents[node]
            path.append(start_node)
            path.reverse()
            return path, closed_list, distance_travelled
    
        if not parents.get(tuple(goal_node)):
            print("No path found")
            return [], [], None
    
    except:
        print("Please enter a valid start and/or goal node!")
        return [], [], None