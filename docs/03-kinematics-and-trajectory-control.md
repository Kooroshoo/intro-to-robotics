# Part III: Kinematics and Trajectory Control

Once we know where the robot is, we use control algorithms to move it to a goal. This is treated as an optimization problem: we want to minimize a error.



## Defining the Error

Before a robot can move toward a goal, it needs to know exactly how "wrong" its current pose is. The error between a robot's current pose $(x_r, y_r, \theta_r)$ and a goal pose $(x_g, y_g, \theta_g)$ is three-fold: a distance to cover, a direction to face to get there, and a final heading to end up at.

<p markdown="1" style="text-align:center;">
![The three components of a robot's pose error: distance, direction, and heading](assets/images/goal_error.svg)
</p>

**Distance** ($\rho$) is just the straight-line distance to the goal:

$$
\rho = \sqrt{(x_g - x_r)^2 + (y_g - y_r)^2}
$$

**Direction** ($\alpha$) is the angle the robot needs to turn to face the goal, measured relative to its current heading:

$$
\alpha = \text{atan2}(y_g - y_r,\ x_g - x_r) - \theta_r
$$

**Heading error** ($\theta_e$) is simply how far off the robot's final orientation is from the goal's:

$$
\theta_e = \theta_g - \theta_r
$$

These three values — $\rho$, $\alpha$, $\theta_e$ — are exactly what gradient descent below tries to drive to zero.

## Forward Kinematics

Odometry, from the previous lecture, is a special case of **forward kinematics**: given a robot's actuator values, where does it end up?

A wheeled robot like the E-puck only has 2 of 6 possible degrees of freedom — translation along $X_R$ and rotation around $Z$:

<p markdown="1" style="text-align:center;">
![A mobile robot's two degrees of freedom: forward step and rotation](assets/images/odometry_step.svg)
</p>

$$
\Delta x = \frac{r\Delta\phi_l + r\Delta\phi_r}{2} \qquad \Delta\omega_z = \frac{r\Delta\phi_r - r\Delta\phi_l}{d}
$$

That's just the local step. Rotating it by the robot's current heading $\alpha$ and adding it to the previous pose gives its actual position in the world:

$$
x_w = x_w + \Delta x\cos\alpha \qquad y_w = y_w + \Delta x\sin\alpha \qquad \alpha = \alpha + \Delta\omega_z
$$

A robot arm's forward kinematics works the same way, just with a different actuator: each **joint angle** contributes a rotation followed by a fixed translation along the link to the next joint. Finding the end-effector's position is the same frame-chaining problem from spatial representation, applied once per joint:

<p markdown="1" style="text-align:center;">
![A 2-link robot arm, with each joint contributing its own transformation to the chain](assets/images/arm_forward_kinematics.svg)
</p>

$$
T^W_{ee} = T^W_1 \cdot T^1_2 \cdot T^2_3 \cdots T^{n-1}_{ee}
$$

Each $T^{i-1}_i$ has the same rotate-then-translate structure as the odometry matrix above — a rotation by the joint's own angle $\theta_i$, then a fixed translation of length $L_i$ (the link) along the rotated axis. For the 2-link arm:

$$
T^{i-1}_i = \begin{bmatrix} \cos\theta_i & -\sin\theta_i & L_i\cos\theta_i \\ \sin\theta_i & \cos\theta_i & L_i\sin\theta_i \\ 0 & 0 & 1 \end{bmatrix}
$$

Only $\theta_i$ changes as the joint rotates — $L_i$ is fixed by the arm's geometry. Multiplying the two matrices out:

$$
T^0_2 = T^0_1 \cdot T^1_2 = \begin{bmatrix} \cos(\theta_1+\theta_2) & -\sin(\theta_1+\theta_2) & L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2) \\ \sin(\theta_1+\theta_2) & \cos(\theta_1+\theta_2) & L_1\sin\theta_1 + L_2\sin(\theta_1+\theta_2) \\ 0 & 0 & 1 \end{bmatrix}
$$

The rotation part collapsed to $\theta_1+\theta_2$ because chaining two rotations just adds their angles. Reading the translation column straight off this matrix gives the end-effector's pose directly in the world frame:

$$
x_{ee} = L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2) \qquad y_{ee} = L_1\sin\theta_1 + L_2\sin(\theta_1+\theta_2)
$$

$$
\theta_{ee} = \theta_1 + \theta_2
$$

## Holonomic vs. Non-Holonomic Motion

The two forward kinematics above behave differently in an important way — whether a closed loop in joint space also closes the loop in the workspace:

- **Arm (holonomic):** the end-effector's pose depends only on the current joint angles $\theta_1, \theta_2$. Move the joints along any path and return them to their starting angles, and the end-effector is guaranteed to be back at its starting pose too.

- **Mobile robot (non-holonomic):** drive the wheels around and return them to their original rotation, and the robot won't generally be back at its starting position. The world pose depends on the path taken, not just the final wheel angles.

This difference makes us calculate the error differently. The arm can move freely in any direction, so the error vector can simply be:

$$
e = \begin{bmatrix} x_g \\ y_g \\ \theta_g \end{bmatrix} - \begin{bmatrix} x_r \\ y_r \\ \theta_r \end{bmatrix}
$$

The mobile robot can't move that freely, so its error has to be expressed in the $\rho$, $\alpha$, $\theta_e$ terms defined earlier instead:

$$
e = \begin{bmatrix} \rho \\ \alpha \\ \theta_e \end{bmatrix}
$$

## Gradient Descent
Gradient descent is an algorithm used to find the minimum of a function. By continuously moving in the direction opposite to the mathematical slope, the robot "descends" the error curve until the error reaches zero.

![Gradient descent applied to an error function](https://placehold.co/800x400?text=Gradient+Descent+Graph)

## The Jacobian Matrix
In multi-degree-of-freedom systems, the simple 1D derivative is replaced by the **Jacobian Matrix** ($J$), which contains all the partial derivatives of the robot's kinematics. 

The control law for inverse kinematics to reduce an error $e$ becomes:

$$\Delta q = -J^+ e$$

*(Where $J^+$ is the pseudo-inverse of the Jacobian).*

### Implementation on Complex Robots
For a humanoid robot or industrial arm, $J$ is a large, dynamically changing matrix (e.g., $6 \times 30$). Computing the pseudo-inverse $J^+$ requires heavy real-time numerical solvers in the software loop.

### Simplification for Wheeled Robots
For a 2D differential drive robot, the Jacobian is a simple, static matrix. Because it is so simple, we can invert it algebraically on paper *before* writing the code. 

As shown below, the matrix math collapses into two simple linear equations for the left and right wheels, creating a basic Proportional (P) Controller:

![Algebraic simplification of the Jacobian inverse](https://placehold.co/800x400?text=Gradient+Descent+Graph)

### Python Code Example
Instead of calculating complex matrices in our `while` loop, our simplified Python code looks like this:

```python
# Calculate Errors
rho = np.sqrt((xw - target_x)**2 + (yw - target_y)**2)
alpha = np.arctan2(target_y - yw, target_x - xw) - theta

# Algebraic simplification of the Jacobian Inverse
phil = min(-alpha * p1 + rho * p2, 6.28)
phir = min(alpha * p1 + rho * p2, 6.28)

# Set motor velocities
motor_left.setVelocity(phil)
motor_right.setVelocity(phir)