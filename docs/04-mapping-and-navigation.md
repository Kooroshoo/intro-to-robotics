# Part IV: Mapping and Navigation

## Why Navigation?

So far, every controller we've built assumes a clear, straight-line path to the goal: sense the error, and drive it to zero. But what happens when there's an obstacle sitting directly between the robot and its goal?

<p markdown="1" style="text-align:center;">
![A straight path to the goal blocked by an obstacle](assets/images/obstacle_navigation.svg)
</p>

A controller alone can't solve this — it only knows how to reduce distance and heading error, not what's in the way. To get around the obstacle, the robot first needs a **map** of its environment, and then a **path** through that map for the controller to follow. This chapter covers both.

## Configuration Space and Waypoints

The way to actually build that path is to first inflate the obstacle by the robot's own size, until the robot can be treated as a single point. Any point outside this inflated boundary is guaranteed collision-free, no matter which way the robot is facing.

A path planner searches this free space for a sequence of **waypoints** — intermediate goal poses that string together into a full path around the obstacle. The controller from Chapter 3 doesn't change at all: it just drives to each waypoint in turn, one at a time, until it reaches the final goal.

<p markdown="1" style="text-align:center;">
![The obstacle inflated in configuration space, with a sequence of waypoints planned around it](assets/images/waypoints_cspace.svg)
</p>

This growing trick works because we're treating the robot as a circle: since it looks the same from every angle, inflating the obstacle by its radius gives a single 2D map that's valid no matter the robot's heading.

<div markdown="1" style="display:flex; justify-content:center; align-items:flex-start; flex-wrap:wrap; gap:24px;">

![A room with obstacles, drawn to their actual physical size](assets/images/cspace_workspace.svg)

![The same room in configuration space: the robot is a point, and the obstacles grow to make room for it](assets/images/cspace_configuration.svg)

</div>

A non-circular mobile robot wouldn't get away with just one 2D map like this — it would need a different one for every possible heading.

<div markdown="1" style="display:flex; justify-content:center; align-items:flex-start; flex-wrap:wrap; gap:24px;">

![A rectangular robot facing 0 degrees, with its inflated obstacle boundary](assets/images/cspace_rect_theta0.svg)

![The same obstacle and robot facing 90 degrees instead: the inflated boundary is a different shape](assets/images/cspace_rect_theta90.svg)

</div>

A robot's **configuration space** (or **C-space**) is the minimal set of numbers that fully describes its pose — for a mobile robot, that's its position and heading $(x, y, \theta)$; for an arm, it's the joint angles $(q_1, \dots, q_n)$. 


<div markdown="1" style="display:flex; justify-content:center; align-items:flex-start; flex-wrap:wrap; gap:24px;">

![A 2-link arm reaching toward an obstacle in its physical workspace](assets/images/arm_cspace_workspace.svg)

![The same obstacle in the arm's configuration space: an axis per joint angle, and a blocked region instead of a solid shape](assets/images/arm_cspace_plot.svg)

</div>

## Mapping

In order to build the waypoints from the last section, we first need a map of the environment. This map can be acquired **offline** — built ahead of time, before the robot ever moves — or **online**, with the robot exploring on its own and building the map as it goes, until it has learned enough of the room to plan a path.

One common way to build a map online is to scan the environment with sensors — lidar, cameras, sonar — that measure distance to nearby surfaces. Sweeping these measurements around the robot marks a point everywhere a beam hits something solid, building up a **point cloud** of the space around it.

<div markdown="1" style="display:flex; justify-content:center; align-items:flex-start; flex-wrap:wrap; gap:24px;">

![A robot in a room, with lidar rays hitting the walls and obstacles around it](assets/images/lidar_scan_room.svg)

![The resulting point cloud: just the scattered points where the rays stopped](assets/images/lidar_point_cloud.svg)

</div>

That point cloud can then be turned into an **occupancy grid**: divide the room into a grid of cells, and mark each one `1` if any point fell inside it, or `0` if it's still empty. The path planner can search directly over this grid instead of the raw points.

<p markdown="1" style="text-align:center;">
![The point cloud converted into a grid of occupied (1) and free (0) cells](assets/images/occupancy_grid_binary.svg)
</p>

We can inflate the obstacle in the grid to account for the robot's own size using a simple image-processing technique called **convolution**: slide a small window — a **kernel**, sized to the robot's footprint — across the grid, and mark a cell occupied if any occupied cell falls inside the kernel centered on it. The obstacle grows by exactly the kernel's size.

<div markdown="1" style="display:flex; justify-content:center; align-items:flex-start; flex-wrap:wrap; gap:24px;">

![A 3x3 kernel positioned over a grid, next to a 2x2 occupied block](assets/images/convolution_before.svg)

![The same grid after convolution: every cell the kernel could reach from the obstacle is now occupied too](assets/images/convolution_after.svg)

</div>


