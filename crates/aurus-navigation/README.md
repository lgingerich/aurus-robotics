## Navigation Stack Overview

1.  **Map Building (SLAM - Simultaneous Localization and Mapping):**
    *   **Purpose:** Create/update a map of the factory environment.
    *   **Key Algorithms:** GMapping, Cartographer, Hector SLAM.
    *   **Output:** Typically a 2D occupancy grid map.

2.  **Exploration (Optional - for autonomous map generation):**
    *   **Purpose:** Autonomously navigate unknown areas to build a complete map.
    *   **Key Algorithms:** Frontier-based Exploration, Information Gain methods.

3.  **Localization:**
    *   **Purpose:** Continuously determine the robot's precise position and orientation (pose) within the existing map.
    *   **Key Algorithms:** AMCL (Adaptive Monte Carlo Localization), Kalman Filters (EKF/UKF) for sensor fusion.

4.  **Obstacle Representation (Costmaps):**
    *   **Purpose:** Create a dynamic, layered representation of the environment for path planning, indicating free space, obstacles, and areas to avoid.
    *   **Layers:** Static map layer, real-time obstacle layer, inflation layer (robot footprint + safety margin).

5.  **Global Path Planning:**
    *   **Purpose:** Find an optimal, collision-free path from the robot's current location to a goal location using the static map information.
    *   **Key Algorithms:** A* (A-star), Dijkstra, RRT/RRT*.

6.  **Local Path Planning & Collision Avoidance:**
    *   **Purpose:** Generate immediate, feasible velocity commands to follow the global path while reacting to dynamic obstacles detected in real-time (using the costmap).
    *   **Key Algorithms:** DWA (Dynamic Window Approach), TEB (Timed Elastic Band) Local Planner, Trajectory Rollout.
