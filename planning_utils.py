import time
from enum import Enum
from queue import PriorityQueue
import numpy as np
from shapely.geometry import Polygon, Point, LineString
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy.linalg as LA
from sklearn.neighbors import KDTree
from udacidrone.frame_utils import global_to_local
import networkx as nx
import math

'''
'''
def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - north_min, 0, north_size-1)),
                int(np.clip(north + d_north - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - east_min, 0, east_size-1)),
                int(np.clip(east + d_east - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)

def create_voxmap(data, voxel_size=5):
    """
    Returns a grid representation of a 3D configuration space
    based on given obstacle data.
    
    The `voxel_size` argument sets the resolution of the voxel map. 
    """
    # minimum and maximum north coordinates
    north_min = np.floor(np.amin(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.amax(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.amin(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.amax(data[:, 1] + data[:, 4]))

    # maximum altitude
    alt_max = np.ceil(np.amax(data[:, 2] + data[:, 5]))
    
    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min)) // voxel_size
    east_size = int(np.ceil(east_max - east_min)) // voxel_size
    alt_size = int(alt_max) // voxel_size

    # Create an empty grid
    voxmap = np.zeros((north_size, east_size, alt_size), dtype=np.bool)

    for i in range(data.shape[0]):
        # TODO: fill in the voxels that are part of an obstacle with `True`
        #
        # i.e. grid[0:5, 20:26, 2:7] = True
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        obstacle = [
            int(north - d_north - north_min) // voxel_size,
            int(north + d_north - north_min) // voxel_size,
            int(east - d_east - east_min) // voxel_size,
            int(east + d_east - east_min) // voxel_size,
        ]

        height = int(alt + d_alt) // voxel_size
        voxmap[obstacle[0]:obstacle[1], obstacle[2]:obstacle[3], 0:height] = True

    return voxmap


def obstacle_polygons(data, safety_distance):
    obstacle_polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        obstacle = [north - d_north - safety_distance, north + d_north + safety_distance,
                    east - d_east - safety_distance, east + d_east + safety_distance]
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), 
                   (obstacle[1], obstacle[3]), (obstacle[1], obstacle[2])]
        height = alt + d_alt
        t = Polygon(corners)
        obstacle_polygons.append((t, height))

    return obstacle_polygons       


def point_sampler(data, nSamples, drone_altitude):
    xmin = np.min(data[:, 0] - data[:, 3])
    xmax = np.max(data[:, 0] + data[:, 3])
    
    ymin = np.min(data[:, 1] - data[:, 4])
    ymax = np.max(data[:, 1] + data[:, 4])
    
    #zmin = drone_altitude - 0.1
    zmax = drone_altitude
    
    xvals = np.random.uniform(xmin, xmax, nSamples)
    yvals = np.random.uniform(ymin, ymax, nSamples)
    #zvals = np.random.uniform(zmin, zmax, nSamples)
    zvals = np.ones(xvals.shape) * zmax
    samples = np.array(list(zip(xvals, yvals, zvals)))
    max_poly_dimension = 2 * np.max((data[:, 3], data[:, 4]))
    
    return samples, max_poly_dimension
   
def create_nodes(data, nSamples, drone_altitude, safety_distance):
    collision_free_nodes = []
    obstacle = obstacle_polygons(data, safety_distance)
    samples, max_poly_dimension = point_sampler(data, nSamples, drone_altitude)
    centers = np.array([(p[0].centroid.x, p[0].centroid.y) for p in obstacle])
    tree = KDTree(centers, metric='euclidean')
    
    collision_free_nodes = []
    for s in samples:
        in_collision = False
        idxs = list(tree.query_radius(np.array([s[0], s[1]]).reshape(1, -1), r=max_poly_dimension)[0])
        if len(idxs) > 0:
            for ind in idxs: 
                p = obstacle[int(ind)]
                if p[0].intersects(Point(s[0], s[1])) and p[1] >= s[2]:
                    in_collision = True

        if not in_collision:
            collision_free_nodes.append(s)
                
    return collision_free_nodes  

    
def nodes_connect(obstacle, n1, n2):
    l = LineString([n1, n2])
    for (obst_poly, obst_height) in obstacle:
        if obst_poly.crosses(l):
            return False
    return True
 

def create_graph(data, nodes, k, safety_distance):
    aa = time.time()
    obstacle = obstacle_polygons(data, safety_distance)
    g = nx.Graph()
    tree = KDTree(nodes)
    for n1 in nodes:
        idxs = tree.query([n1], k, return_distance=False)[0]
        
        for idx in idxs:
            n2 = nodes[idx]
            if np.allclose(n2, n1):
                continue
            if nodes_connect(obstacle, n1, n2):
                n1_tup = ((n1[0], n1[1], n1[2]))
                n2_tup = ((n2[0], n2[1], n2[2]))
                dist = LA.norm(np.array(n2_tup) - np.array(n1_tup))
                g.add_edge(n1_tup, n2_tup, weight=dist)

    bb = time.time()
    print('Graph construction time: ', bb-aa)
    return g

class Action(Enum):
    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    SOUTH_EAST = (1, 1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    NORTH_WEST = (-1, -1, np.sqrt(2))

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):

    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
        
    return valid_actions

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def a_star(graph, start, goal):
    """Modified A* to work with NetworkX graphs."""
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                branch_cost = current_cost + cost
                queue_cost = branch_cost + heuristic(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))
                    
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('Try increasing nSample')
        print('**********************') 
    return path[::-1], path_cost



def closest_point(graph, point_3d):
    """
    Compute the closest point in the `graph`
    to the `point_3d`.
    """
    current_point = (point_3d[0], point_3d[1], point_3d[2])
    closest_point = None
    dist = 100000
    for p in graph.nodes:
        d = LA.norm(np.array(p) - np.array(current_point))
        if d < dist:
            closest_point = p
            dist = d
    return closest_point

def collinearity_prune(path, epsilon=1e-4):
    """
    Prune path points from `path` using collinearity.
    """
    def point(p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def collinearity_check(p1, p2, p3):
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])
        if collinearity_check(p1, p2, p3):

            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path


def calculate_waypoints(global_start, global_goal, global_home, data, drone_altitude, safety_distance, nSamples, k):
    # Calculate graph and offsets
    grid, north_offset, east_offset = create_grid(data, drone_altitude, safety_distance)
    collision_free_nodes= create_nodes(data, nSamples, drone_altitude, safety_distance)
    graph = create_graph(data, collision_free_nodes, k, safety_distance)

    local_position = global_to_local(global_start, global_home)
    graph_start = closest_point(graph, local_position)
    local_goal = global_to_local(global_goal, global_home)
    graph_goal = closest_point(graph, local_goal)
    # Find path
    a = time.time()
    path, path_cost = a_star(graph, graph_start, graph_goal)
    b = time.time()
    print('A* search time:' , b-a)
    #print('path: ', path)
    print('path cost: ', path_cost)
    print('path length: ', len(path))

    pruned_path = collinearity_prune(path, epsilon=1e-4)

    return [[int(p[0]), int(p[1]), math.ceil(p[2]), 0] for p in path], path, pruned_path, graph, grid

def visualize_path(data, g_start, g_goal, grid, graph, path):

    fig = plt.figure(figsize=(20,10))   
    plt.imshow(grid, origin='lower')
    nmin = np.min(data[:, 0])
    emin = np.min(data[:, 1])
    
    for n1 in graph.nodes:
        plt.scatter(n1[1] - emin, n1[0] - nmin, c='red')

    for (n1, n2) in graph.edges:
        plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black', alpha = 0.3)
        
    path_pairs = zip(path[:-1], path[1:])
    for (n1, n2) in path_pairs:
        plt.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'blue', linewidth=4.0)
    
    plt.xlabel('EAST')
    plt.ylabel('NORTH')
    plt.show()
    