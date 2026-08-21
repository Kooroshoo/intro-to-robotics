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