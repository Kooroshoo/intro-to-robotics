# Part III: Kinematics and Trajectory Control

Given a robot's joint angles, where does it end up? And once we know where it is, how do we drive it to a goal? This chapter covers both: **forward kinematics** computes a robot's pose from its joints, and **differential kinematics** (Jacobians) drives control toward a target, treated as an optimization problem where we minimize the distance error.

## Forward Kinematics

A robot arm is a chain of rigid links connected by joints, and each joint contributes its own small transformation: a rotation by the joint's current angle, followed by a fixed translation along the link to the next joint. Finding the end-effector's position in the world frame is exactly the frame-chaining problem from spatial representation, applied once per joint:

$$
T^W_{ee} = T^W_1 \cdot T^1_2 \cdot T^2_3 \cdots T^{n-1}_{ee}
$$

Each $T^{i-1}_i$ only changes as its joint rotates; the rest of the chain stays fixed. Multiplying them all together converts a point measured at the end-effector directly into the world frame — this is what's known as **forward kinematics**.

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