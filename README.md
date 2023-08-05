# Path Planning Algorithms for Mobile Robots
 This repository contain "Path Planning Algorithms for Mobile Robots", a collection of popular algorithms written in Python, which is an integral aspect of robotic navigation, control and obstacle avoidance. This repository has been made to give a little sample of into the world of robotic path planning algorithms with visualisation and description of code, to gain an insight into path planning algorithms. 

 A key concept in robotic path planning is the **Configuration Space** or for short, **"C-space"**, an abstract space that represents all possible configurations or states of a robot, such as position and orientation. C-space helps in visualing the constraints and obstacles, allowing algorithms to find optimal and feasible paths that avoid collision. **This repository mainly focusing on a mobile robot (such as a rover) moving in a 2D plane, hence C-space is 2-dimensional.** However, for a robotic arm it can have n-dimensions (since a robotic arm can have n-joints), for more complex robots the dimensionality can increase; this is an important computational consideration when dealing with high C-space.

**The following algorithms included:**
- **RRT (Rapidly-Exploring Random Trees)**: A probablistic algorithm for efficiently exploring search spaces. Creates a rapidly growing tree by initalising a "root" of the tree, by randomly sampling the configuration space, finding feasible paths through complex and high-dimensional environment. This algorithm is great robotic systems such as robotic graspers which occupy higher dimensional environments then a mobile robot such as a rover.
- **RRT-Connect**: Similar to RRT, however instead builds a second tree to converge towards a solution quicker. One tree from the start node and another from the goal node, which eventually connect to form a complete path. Often quicker then RRT and less computationally taxing.
- **PRM (Probabilistic Roadmap)**: Randomly samples nodes in the configuration space and then constructs a graph by connecting the nodes. The sampling of nodes is carried out within a bounded configuration space, confined to a finite area of choice. Also contains collision detection algorithm within the code, to make sure the robot avoid obstacles. Once constructed a graph traversal algorithms such as Dijkstra's or A* algorithm can be used to query a path from the start node to the goal node within the graph.
- **PRM***: The optimal variant of the PRM algorithm, to find a more efficient path using a heuristic to guide the search by employing the A* algorithm.
