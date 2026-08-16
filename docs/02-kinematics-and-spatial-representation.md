# Part II: Kinematics and Spatial Representation

## How Do We Know Where a Robot Is in a Scene?

To describe a robot's position in space, we must account for both its **position** (translation) and its **orientation** (rotation). This position and rotation are always defined relative to a world coordinate frame. This representation is known as **Cartesian space**.

![Frame Transformation](assets/images/frame_transform.svg)

## Describing a Position

A robot's location can be described with three values, one along each of the $X$, $Y$, and $Z$ axes, measured relative to the world frame:

$$
\vec p = \begin{bmatrix}c_1\\c_2\\c_3\end{bmatrix}
$$

A robot also has an orientation — its roll, pitch, and yaw. We can expand the previous formula in the following manner:

$$
\vec p = \begin{bmatrix}c_1\\c_2\\c_3\end{bmatrix} = c_1\begin{bmatrix}1\\0\\0\end{bmatrix} + c_2\begin{bmatrix}0\\1\\0\end{bmatrix} + c_3\begin{bmatrix}0\\0\\1\end{bmatrix} = c_1\vec b_1 + c_2\vec b_2 + c_3\vec b_3
$$

$$
\vec p = \begin{bmatrix}\vec b_1 & \vec b_2 & \vec b_3\end{bmatrix}\begin{bmatrix}c_1\\c_2\\c_3\end{bmatrix}
$$

$$
\vec p = \begin{bmatrix}1&0&0\\0&1&0\\0&0&1\end{bmatrix}\begin{bmatrix}c_1\\c_2\\c_3\end{bmatrix} = R\vec c
$$

Here, $R$ represents the robot's rotation. When the robot's orientation matches the world's, $R = I$ and $\vec p = \vec c$.

$R$ can also be built directly from the two frames' axes, without knowing the rotation angle beforehand. Each entry is the dot product between an axis of the robot frame and an axis of the world frame:

![Rotation matrix from axis dot products](assets/images/rotation_dot_product.svg)

$$
\begin{pmatrix}x\\y\\z\end{pmatrix}_{world} = \begin{pmatrix}\vec x_R\cdot\vec x_W & \vec y_R\cdot\vec x_W & \vec z_R\cdot\vec x_W \\ \vec x_R\cdot\vec y_W & \vec y_R\cdot\vec y_W & \vec z_R\cdot\vec y_W \\ \vec x_R\cdot\vec z_W & \vec y_R\cdot\vec z_W & \vec z_R\cdot\vec z_W\end{pmatrix} \cdot \begin{pmatrix}x\\y\\z\end{pmatrix}_{robot}
$$

Each dot product is just the cosine of the angle between the two axes, scaled by their lengths ("Dot Product"):

$$
\vec x_R\cdot\vec x_W = \cos\alpha\,|\vec x_R||\vec x_W|
$$

Since each axis is a unit vector, its length is $1$, so the dot product simplifies to just the cosine of the angle between them:

$$
\vec x_R\cdot\vec x_W = \cos\alpha
$$

For a planar robot rotating only around $Z$ (yaw, $\alpha$), most of these dot products reduce to $0$ or $1$, and the matrix collapses into the familiar 2D rotation matrix:

$$
\begin{pmatrix}x\\y\\z\end{pmatrix}_{world} = \begin{pmatrix}\cos\alpha & -\sin\alpha & 0\\ \sin\alpha & \cos\alpha & 0 \\ 0 & 0 & 1\end{pmatrix}\begin{pmatrix}x\\y\\z\end{pmatrix}_{robot}
$$

## Combining Rotation and Translation

A point is often measured with respect to the robot's frame, but we need its position in the world frame instead.

![Frame Transformation with a Target Point](assets/images/frame_transform_target.svg)

$$
Q_{world} = R \cdot Q_{robot} + P
$$

$$
\begin{pmatrix}q_x\\q_y\\q_z\end{pmatrix}_{world} = \begin{pmatrix}\cos\alpha & -\sin\alpha & 0\\ \sin\alpha & \cos\alpha & 0 \\ 0 & 0 & 1\end{pmatrix}\begin{pmatrix}q_x\\q_y\\q_z\end{pmatrix}_{robot} + \begin{pmatrix}p_x\\p_y\\p_z\end{pmatrix}
$$

$R$ is the rotation matrix derived above, $Q_{robot}$ is the target point's coordinates in the robot frame, and $P$ is the translation between the two origins.


## The Transformation Matrix

To combine rotation and translation into a single operation, we use a **Homogeneous Transformation Matrix** ($T$). On a flat plane, rotation only happens around one axis (yaw, $\alpha$):

$$
T^W_R = \begin{bmatrix}\cos\alpha & -\sin\alpha & p_x \\ \sin\alpha & \cos\alpha & p_y \\ 0 & 0 & 1\end{bmatrix}
$$

A point measured in the robot frame ($Q_{robot}$, e.g. the **Target Point** shown above) is converted to the world frame with:

$$
Q_{world} = T^W_R \cdot Q_{robot}
$$

$$
\begin{bmatrix}q_x\\q_y\\1\end{bmatrix}_{world} = \begin{bmatrix}\cos\alpha & -\sin\alpha & p_x \\ \sin\alpha & \cos\alpha & p_y \\ 0 & 0 & 1\end{bmatrix}\begin{bmatrix}q_x\\q_y\\1\end{bmatrix}_{robot}
$$

### 3D Transformation Matrix

Robots that also move or rotate vertically (drones, arms, legged robots) need a third axis, $Z$. The same idea scales up to a $4 \times 4$ matrix:

![Frame Transformation in 3D](assets/images/frame_transform_3d.svg)

$$
T^W_R =
\begin{bmatrix}
R_{3 \times 3} & P_{3 \times 1} \\
0 & 0 & 0 & 1
\end{bmatrix}
=
\begin{bmatrix}
r_{11} & r_{12} & r_{13} & p_x \\
r_{21} & r_{22} & r_{23} & p_y \\
r_{31} & r_{32} & r_{33} & p_z \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Each column of $R$ is one of the robot frame's basis vectors expressed in world coordinates, and $P$ is the translation between the two origins. The same rule as before applies: append a $1$ to a point and multiply by $T$ to move it between frames.

For a robot that only yaws (rotates around $Z$), the abstract $r_{ij}$ entries fill in with the same $\cos\alpha / \sin\alpha$ terms as the 2D case, leaving $Z$ untouched:

$$
T^W_R =
\begin{bmatrix}
\cos\alpha & -\sin\alpha & 0 & p_x \\
\sin\alpha & \cos\alpha & 0 & p_y \\
0 & 0 & 1 & p_z \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

### Reversing a Transformation

Going the other way — from the world frame back into the robot frame — doesn't require recomputing anything. Since $R$ is orthogonal, its inverse is just its transpose ($R^{-1} = R^T$), so:

$$
T^{-1} = \begin{bmatrix} R^T & -R^T P \\ 0\ 0\ 0 & 1 \end{bmatrix}
$$

Rotation is undone with $R^T$, and the translation must first be rotated by $R^T$ as well before being subtracted — since $P$ was expressed in the world's orientation, not the robot's.

### Chaining Transformations

Frames can be linked in a chain — a world frame, a robot base, and a sensor mounted on the robot, for example. To move a point through the whole chain, multiply the transformation matrices together:

$$
T^A_C = T^A_B \cdot T^B_C
$$

![Chaining transformations across three frames](assets/images/frame_chain.svg)

Once combined, the result behaves exactly like any other transformation matrix — a point measured in frame $\{C\}$ is converted all the way into frame $\{A\}$ in one multiplication:

$$
Q_A = T^A_C \cdot Q_C
$$

## Examples in Practice

### Mobile Robot: Odometry

In practice, a robot rarely knows one large offset $P$ up front — it accumulates many small motions instead. At each timestep, it moves a small step $\Delta x$ forward along its own $X_R$ axis, then updates its heading by a small yaw rate:

$$
\begin{pmatrix}\Delta x\\ \Delta y\end{pmatrix}_{world} = \begin{pmatrix}\cos\alpha & -\sin\alpha\\ \sin\alpha & \cos\alpha\end{pmatrix}\begin{pmatrix}\Delta x\\ 0\end{pmatrix}_{robot}
$$

```python
xw = xw + np.cos(alpha) * deltaX
yw = yw + np.sin(alpha) * deltaX
alpha = alpha + deltaOmegaZ
```

This is the rotation-only case from before ($Q_{world} = R \cdot Q_{robot}$), applied to a small step instead of a fixed point. The running estimate $(x_w, y_w)$ plays the role of the accumulated translation $P$: every step rotates the local motion into world coordinates and adds it to that running total, while $\alpha$ keeps track of the robot's current heading.

### Arm Robot: Chaining Joint Transforms

A robot arm is a chain of rigid links connected by joints, and each joint contributes its own small transformation: a rotation by the joint's current angle, followed by a fixed translation along the link to the next joint. Finding the end-effector's position in the world frame is exactly the chaining problem from above, applied once per joint:

$$
T^W_{ee} = T^W_1 \cdot T^1_2 \cdot T^2_3 \cdots T^{n-1}_{ee}
$$

Each $T^{i-1}_i$ only changes as its joint rotates; the rest of the chain stays fixed. Multiplying them all together converts a point measured at the end-effector directly into the world frame — this is what's known as **forward kinematics**.