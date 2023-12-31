# Path Planning Algorithms for Mobile Robots
This repository contains **"Path Planning Algorithms for Mobile Robots,"** a collection of popular algorithms written in Python. These are integral to robotic navigation, control, and obstacle avoidance. This repository has been created to provide a sample insight into the world of robotic path planning algorithms, complete with visualizations and code descriptions.

 A key concept in robotic path planning is the **Configuration Space**, or **"C-space"** for short, an abstract space that represents all possible configurations or states of a robot, such as position and orientation. C-space helps in visualizing the constraints and obstacles, allowing algorithms to find optimal and feasible paths that avoid collisions. **This repository mainly focuses on a mobile robot (such as a rover) moving in a 2D plane, so the C-space is 2-dimensional**. However, for a robotic arm with n-joints, it can have n-dimensions, and for more complex robots, the dimensionality can increase. This is an important computational consideration when dealing with high-dimensional C-space. 

**The following below are the algorithms and their descriptions (red lines represent shortest path and hollow circles are obstacles):**
## Rapidly-Exploring Random Trees
- **RRT (Rapidly-Exploring Random Trees)**: A probabilistic algorithm for efficiently exploring search spaces. It creates a rapidly growing tree by initializing a "root" of the tree and randomly sampling the configuration space, finding feasible paths through complex and high-dimensional environments. This algorithm is great for robotic systems such as robotic graspers that occupy higher-dimensional environments than a mobile robot, such as a rover.
- **RRT-Connect**: Similar to RRT, but instead builds two trees to converge towards a solution more quickly. One tree grows from the start node and another from the goal node, eventually connecting to form a complete path. Often quicker and less computationally taxing than RRT.
  
 **Below is an example of RRT:**
  <p align="center">
   <img src="https://raw.githubusercontent.com/mdsamirhussain/MobileRobot-PathPlanners/main/images/RRT.png" width="400" height="300">
  </p>
  
## Probabilistic Roadmap

- **PRM**: Randomly samples nodes in the configuration space and constructs a graph by connecting the nodes. The sampling is carried out within a bounded configuration space, confined to a finite area of choice. This method includes a collision detection algorithm to ensure the robot avoids obstacles. Once the graph is constructed, graph traversal algorithms such as Dijkstra's or A* can be used to query a path from the start node to the goal node within the graph.
- **PRM***: The optimal variant of the PRM algorithm, designed to find a more efficient path using a heuristic to guide the search by employing the A* algorithm. 

**Below is an example of PRM:**
 <p align="center"> 
 <img src="https://raw.githubusercontent.com/mdsamirhussain/MobileRobot-PathPlanners/main/images/PRM-Dijkstra.png" width="400" height="300">
 </p>
 
 ## A* Algorithm
 
 -  **A-star Algorithm**: A* Algorithm is an optimal pathfinding algorithm which uses additional heuristics to find the shortest path in a 2D environment. It doesn't necessarily create a graph, it directly searches the space from start to goal, using whatever representation of the problem space is provided due to it being able to work on various data structures. The heuristic function is used for prioritising the search for the algorithm to explore the most essential nodes first. This algorithm also finds its use cases for various types of graphs, meshes and roadmaps.

**Below is an example of A-star Algorithm:**
 <p align="center">
<img src="https://raw.githubusercontent.com/mdsamirhussain/MobileRobot-PathPlanners/main/images/Astar.png" width="400" height="300">
 </p>

