# Spatial Representation and Transformations

To describe a robot in 3D space, we must account for both its position (translation) and its orientation (rotation) relative to a world coordinate frame. 

## 3D Rotation
A 3D rotation matrix $R$ is a $3 \times 3$ matrix representing the orientation of a frame $\{B\}$ relative to a frame $\{A\}$. It is constructed using the dot products of the unit vectors of the axes, effectively computing the cosines of the angles between them:

![Constructing a 3D rotation matrix](https://placehold.co/800x400?text=3D+Rotation+Matrix)

## Translation and Homogeneous Coordinates
To combine rotation $R$ and translation $P$ (a $3 \times 1$ vector) into a single linear operation, we use a $4 \times 4$ **Homogeneous Transformation Matrix** ($T$). This prevents us from having to do separate multiplication and addition steps:

$$^A_B T =  \begin{bmatrix} ^A_B R & ^A P \\ 0 \quad 0 \quad 0 & 1 \end{bmatrix}$$

A point $^B Q$ in the robot's local frame is transformed to the world frame $^A Q$ by appending a $1$ to make it a $4 \times 1$ vector:

![Combining rotation and translation into a homogeneous transform](https://placehold.co/800x400?text=Homogeneous+Transform)

---

## 2D Planar Robots (Differential Drive)
For a wheeled robot moving on a flat surface, the $Z$ translation is constantly $0$, and the only rotation is around the $Z$-axis (yaw, denoted as $\alpha$). The massive $4 \times 4$ matrix simplifies into a $3 \times 3$ transformation matrix:

$$^W_R T =  \begin{bmatrix} \cos\alpha & -\sin\alpha & \Delta x \\ \sin\alpha & \cos\alpha & \Delta y \\ 0 & 0 & 1 \end{bmatrix}$$

![Simplification of the homogeneous transform for planar robots](https://placehold.co/800x400?text=2D+Transform+Matrix)

When using sensors like LiDAR on these planar robots, we often convert from Polar coordinates $(r, \phi)$ to Cartesian coordinates $(x, y)$ before applying the transformation matrix:

![Converting LiDAR polar readings to Cartesian coordinates](https://placehold.co/800x400?text=Polar+to+Cartesian)