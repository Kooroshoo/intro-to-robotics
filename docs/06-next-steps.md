# Next Steps

We covered the essentials: representing pose, computing the commands that drive a robot to a goal, planning a path around obstacles, and deciding which behavior to run. Robotics is much bigger than that, though. Here are some natural next topics, and the gap each one fills.

- **Dynamics and Torque Control** — everything here stopped at position and velocity, never force. Dynamics adds gravity, inertia, and joint coupling, so torque becomes the control variable.

- **Computer Vision** — the only sensor used here was lidar, reporting just distance and angle. Vision reads far richer information straight from a camera: what objects are, where they are, and what a scene looks like.

- **Localization and Mapping (SLAM)** — odometry drifts, and mapping and localization were treated as separate problems. SLAM builds the map and locates the robot in it at the same time, correcting drift with sensor fusion.

- **Smoother Motion Planning** — grid-based search returns a jagged path that ignores the robot's motion limits. RRT/RRT\* search continuous space instead, and trajectory smoothing makes the result trackable.

- **Manipulation and Grasping** — arm control stopped at reaching a pose, never touching anything. Grasp planning and force control let an arm make and hold contact instead.

- **Simulation** — every equation here ran in the abstract, never against a physics engine. Simulators like Isaac Sim, MuJoCo, or Gazebo are where controllers and policies actually get trained and tested before touching real hardware.

- **Reinforcement Learning** — every controller here was model-based, derived from explicit equations. Reinforcement learning learns a policy from trial and error instead.

- **Robotics System Design** — every algorithm here was studied in isolation, never wired into a running robot. This is where perception, planning, and control connect into one system, through middleware like ROS.

