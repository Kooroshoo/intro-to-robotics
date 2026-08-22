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

## Differential Kinematics

Forward kinematics relates actuator values to pose. **Differential kinematics** looks at the same relationship one derivative up: instead of position, we're relating *velocities*.

The derivative of distance is speed — $x$ is distance, $\dot x$ is speed. (You'll also see this written as $x'$; both mean the same thing.)

Applying this to the mobile robot: replace each incremental step ($\Delta x$, $\Delta\omega_z$, $\Delta\phi_l$, $\Delta\phi_r$) with its derivative, and the odometry formulas from earlier become a velocity relationship instead:

$$
\dot x = \frac{r\dot\phi_l + r\dot\phi_r}{2} \qquad \dot\omega_z = \frac{r\dot\phi_r - r\dot\phi_l}{d}
$$

Nothing about the geometry changed — $\dot\phi_l$ and $\dot\phi_r$ are just the wheels' angular velocities instead of a single timestep's rotation.

### Inverse Differential Kinematics

We can also go the other way: solve those same two equations for $\dot\phi_l$ and $\dot\phi_r$, to find the wheel velocities that produce a *desired* $\dot x$ and $\dot\omega_z$:

$$
\dot\phi_l = \frac{2\dot x - \dot\omega_z d}{2r} \qquad \dot\phi_r = \frac{2\dot x + \dot\omega_z d}{2r}
$$


## Minimizing Error

At the beginning of this lecture, we defined a robot's error as the difference between its current pose and the goal pose it needs to reach. Driving the robot to its goal is really just minimizing this error function, and the technique we'll use to do it is **gradient descent**.

Take the simplest possible error function, $y = x^2$: its minimum sits at $x=0$, and its derivative at any point tells us which way is downhill.

<p markdown="1" style="text-align:center;">
![The minimum of y = x^2 sits where the slope is zero](assets/images/gradient_descent_parabola.svg)
</p>

Stepping opposite that derivative, a little at a time, always walks you toward the minimum — no matter which side you started on. With more variables, this becomes stepping opposite the full gradient, sliding down toward the center of the error function's contours:

<p markdown="1" style="text-align:center;">
![Gradient descent converging toward the minimum of an error function](assets/images/gradient_descent_contour.svg)
</p>

This is the exact technique we'll use to build the robot's controller below: measure the error, step opposite its gradient, and repeat until it reaches zero.

### Gradient Descent

In discrete time, this stepping process becomes an update rule: at each timestep $k$, nudge $x$ opposite the derivative, scaled by a gain $L$ that controls how big each step is:

$$
x_{k+1} = x_k - L\,f'(x_k)
$$

### Typical Controller Behavior

That gain $L$ is what determines whether the controller actually converges. Given a step input to track, the response can look very different depending on how it's tuned:

<p markdown="1" style="text-align:center;">
![Typical controller behaviors: stable, marginally stable, and unstable responses to a step input](assets/images/controller_behavior.svg)
</p>

- **Stable** — the error settles at zero, either smoothly or with some decaying oscillation on the way there.
- **Marginally stable** — the error keeps oscillating around zero forever, never quite settling.
- **Unstable** — the error grows without bound, either steadily or through ever-larger oscillations. A gain that's too large is a common cause of this.

### For the Mobile Robot

Let's apply this directly to the mobile robot. Its control inputs are $\dot x$ and $\dot\theta_r$, and its error components $\rho$, $\alpha$ from earlier are already given in polar coordinates. A convenient error function to minimize is the sum of their squares:

$$
e = \rho^2 + \alpha^2
$$

Following the gradient descent rule, each control input steps opposite the error's partial derivative with respect to its matching term:

$$
\dot x = -p_2\frac{\partial e}{\partial \rho} \qquad \dot\theta_r = -p_1\frac{\partial e}{\partial \alpha}
$$

Solving for the wheel speeds with the inverse differential kinematics from earlier:

$$
\dot\phi_l = \frac{2p_2\rho - p_1\alpha d}{2r} \qquad \dot\phi_r = \frac{2p_2\rho + p_1\alpha d}{2r}
$$

Compare this with the intuitive result: drive faster the farther the goal is, and turn faster the more misaligned the heading is. Absorbing the constants into $p_1$ and $p_2$, this collapses into the simple P-controller used in the code below:

```
leftwheel  = -p1 * alpha + p2 * rho
rightwheel =  p1 * alpha + p2 * rho
```

## Examples in Practice