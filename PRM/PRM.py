import math
import random
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from queue import PriorityQueue
import time

start = (0, 0) #qstart
goal = (6, 10) #qgoal
obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2), (9, 5, 2), (8, 10, 1)] #Cobj
robot_radius = 0.8
starttime = time.time()

# Define the bounds of the configuration space
x_min = 0
x_max = 15
y_min = 0
y_max = 15


# Define the number of nodes to generate
num_nodes = 500

# Generate a random sample of nodes in the configuration space (Cfree)
nodes = []
while len(nodes) < num_nodes:
    x = np.random.uniform(x_min, x_max)
    y = np.random.uniform(y_min, y_max)
    is_collision = False
    for ox, oy, r in obstacleList:
        if math.sqrt((ox - x) ** 2 + (oy - y) ** 2) <= r + robot_radius:
            is_collision = True
            break
    if not is_collision:
        nodes.append([x, y])

nodes.append(list(start))
nodes.append(list(goal))

nodes = np.array(nodes)
#print(np.around(nodes,2))

# Define the number of nearest neighbors to consider
k = 10

# Compute the Euclidean distance between each pair of points
distances = np.sqrt(((nodes[:, np.newaxis] - nodes) ** 2).sum(axis=2))

# Find the k-nearest neighbors for each point
neighbors = np.argsort(distances, axis=1)[:, 1:k+1]

# Initialize an empty list to hold the edges
edges = []

def euclidean_distance(node1,node2):
    return math.sqrt((node1[0]-node2[0])**2+(node1[1]-node2[1])**2)
    
# Add edges between each point and its k-nearest neighbors
for i in range(num_nodes):
    for j in neighbors[i]:
        edges.append((i, j))
# print(edges)

# Dijkstra's Algorithm

# visualisation: 
fig, ax = plt.subplots()
ax.scatter(nodes[:, 0], nodes[:, 1])

for obstacle in obstacleList:
    x, y, r = obstacle
    circle = Circle((x, y), r, fill = False, edgecolor = 'black')
    ax.add_patch(circle)

for i, j in edges:
    ax.plot([nodes[i, 0], nodes[j, 0]], [nodes[i, 1], nodes[j, 1]], color='black')
patches = []
robot = Circle(start, robot_radius, fill=True, color='red')
patches.append(ax.add_patch(robot))

plt.show()


end_time = time.time()

print("Runtime:", end_time - starttime, "seconds")



















