# Part I: Introduction to Robotic Systems

Before diving into the math behind robot motion, it helps to establish a shared vocabulary: what a robot actually is, what lets it sense and act, and how we classify the different kinds you'll encounter throughout this book.

## What Is a Robot?

A robot is a programmable physical system that senses its environment, makes decisions, and acts on the world to accomplish a task. At its core, every robot repeats the same loop: **sense → think → act**.

## Sensors and Actuators

* **Sensors** are how a robot perceives its environment and its own state — cameras, LiDAR, wheel encoders, IMUs, and force/touch sensors all report measurements back to the robot's controller.
* **Actuators** are how a robot acts on the world — motors, servos, and hydraulic or pneumatic drives that turn commands into physical motion.

Everything in the chapters that follow is really about closing the loop between these two: turning sensor measurements into the right actuator commands.

## Degrees of Freedom (DOF)

A robot's **degrees of freedom** are the independent ways it can move — each one usually corresponds to a single motor or joint. A wheeled robot moving on a flat floor has 3 DOF ($x$, $y$, and heading $\theta$); a 6-axis industrial arm has 6 DOF, letting its end-effector reach any position *and* orientation in 3D space.

## Mobile Robots vs. Manipulators

* **Mobile robots** (wheeled, legged, aerial) move their entire body through the environment. Their kinematics describes how wheel or leg motion translates into changes in position and heading.
* **Manipulators** (robot arms) stay fixed at their base and move an end-effector through a chain of joints. Their kinematics describes how joint angles translate into the position and orientation of that end-effector.

Both are described with the same underlying math — frames, rotations, and transformations — which is exactly where this book picks up next.