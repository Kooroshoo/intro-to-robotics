# Differential Kinematics and Control

Once we know where the robot is, we use control algorithms to move it to a goal. This is treated as an optimization problem: we want to minimize the distance error.

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

![Algebraic simplification of the Jacobian inverse](https://placehold.co/800x400?text=Jacobian+Inverse)

!!! abstract "Theorem: The Jacobian Inverse"
    For a planar differential drive robot, the Jacobian matrix can be inverted algebraically because the system is non-holonomic but simplified into a 2D constraint space.
    
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
```