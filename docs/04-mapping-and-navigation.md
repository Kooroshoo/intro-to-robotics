# Part IV: Mapping and Navigation

## Why Navigation?

So far, every controller we've built assumes a clear, straight-line path to the goal: sense the error, and drive it to zero. But what happens when there's an obstacle sitting directly between the robot and its goal?

<p markdown="1" style="text-align:center;">
![A straight path to the goal blocked by an obstacle](assets/images/obstacle_navigation.svg)
</p>

A controller alone can't solve this — it only knows how to reduce distance and heading error, not what's in the way. To get around the obstacle, the robot first needs a **map** of its environment, and then a **path** through that map for the controller to follow. This chapter covers both.
