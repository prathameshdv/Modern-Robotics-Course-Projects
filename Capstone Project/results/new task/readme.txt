Modern Robotics Capstone Project – youBot Mobile Manipulation

Controller
----------
The robot uses a task-space feedback controller with feedforward plus
proportional–integral (PI) feedback.

The commanded end-effector twist is:
V = Ad_X^{-1}X_d V_d + Kp X_err + Ki ∫ X_err dt

Controller Gains
----------------
Proportional gain:
Kp = 5 * I_6

Integral gain:
Ki = 0.5 * I_6

Time step:
dt = 0.01 s

Trajectory Generation
---------------------
The reference trajectory consists of eight segments generated using
ScrewTrajectory in SE(3):

1. Move from initial end-effector pose to a standoff above the cube
2. Move vertically down to the grasp pose
3. Close the gripper
4. Move vertically back to the standoff
5. Translate to the goal standoff
6. Move vertically down to the placement pose
7. Open the gripper
8. Retreat to the goal standoff

Grasp Orientation Constraint (Self-Collision Avoidance)
-------------------------------------------------------
The planner is purely kinematic and does not perform collision checking.
When arbitrary cube configurations are used, the arm may bend backward
over the mobile base, resulting in self-collisions.

To prevent this behavior, the grasp and standoff frames were defined
with a fixed orientation that forces the end-effector to approach the
cube from the front and above. This constrains the arm to a forward,
elbow-down configuration and prevents backward bending onto the chassis,
without modifying the controller, gains, or Jacobian.

Cube Configurations
-------------------
The cube initial and goal configurations are specified using the
homogeneous transforms T_sc_init and T_sc_goal. Only the x and y
components are changed to define new tasks, while the cube height
(z = 0.025 m) and grasp orientation remain fixed.

Results
-------
The file youbot_trajectory.csv contains the complete robot configuration
trajectory for playback in CoppeliaSim.

The file Xerr.csv logs the six components of the end-effector
configuration error over time.

A plot of X_err versus time is provided in Xerr_plot.pdf, showing
convergence of all error components to zero, indicating stable closed-
loop behavior.

