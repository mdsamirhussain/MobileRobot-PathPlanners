# Path Planning Algorithms for Mobile Robots
This repository contains "Path Planning Algorithms for Mobile Robots," a collection of popular algorithms written in Python. These are integral to robotic navigation, control, and obstacle avoidance. This repository has been created to provide a sample insight into the world of robotic path planning algorithms, complete with visualizations and code descriptions.

 A key concept in robotic path planning is the Configuration Space, or "C-space" for short, an abstract space that represents all possible configurations or states of a robot, such as position and orientation. C-space helps in visualizing the constraints and obstacles, allowing algorithms to find optimal and feasible paths that avoid collisions. **This repository mainly focuses on a mobile robot (such as a rover) moving in a 2D plane, so the C-space is 2-dimensional**. However, for a robotic arm with n-joints, it can have n-dimensions, and for more complex robots, the dimensionality can increase. This is an important computational consideration when dealing with high-dimensional C-space.

**The following algorithms included:**
- **RRT (Rapidly-Exploring Random Trees)**: A probabilistic algorithm for efficiently exploring search spaces. It creates a rapidly growing tree by initializing a "root" of the tree and randomly sampling the configuration space, finding feasible paths through complex and high-dimensional environments. This algorithm is great for robotic systems such as robotic graspers that occupy higher-dimensional environments than a mobile robot, such as a rover.
- **RRT-Connect**: Similar to RRT, but instead builds two trees to converge towards a solution more quickly. One tree grows from the start node and another from the goal node, eventually connecting to form a complete path. Often quicker and less computationally taxing than RRT.
  
 **Below is an example of RRT:**
  
   ![RRT]<img src="https://github.com/mdsamirhussain/MobileRobot-PathPlanners/assets/115653635/00588ccc-748e-4197-bafb-5795a285864" width="400" height="300">
- **PRM (Probabilistic Roadmap)**: Randomly samples nodes in the configuration space and constructs a graph by connecting the nodes. The sampling is carried out within a bounded configuration space, confined to a finite area of choice. This method includes a collision detection algorithm to ensure the robot avoids obstacles. Once the graph is constructed, graph traversal algorithms such as Dijkstra's or A* can be used to query a path from the start node to the goal node within the graph.
- **PRM***: The optimal variant of the PRM algorithm, designed to find a more efficient path using a heuristic to guide the search by employing the A* algorithm.
  
**Below is an example of PRM:**
  
 ![PRM](https://github.com/mdsamirhussain/MobileRobot-PathPlanners/assets/115653635/d11bde43-8199-42b0-ac30-b4869a0a462c)
