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


