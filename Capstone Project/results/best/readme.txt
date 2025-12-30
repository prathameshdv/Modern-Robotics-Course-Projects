Modern Robotics Capstone Project – youBot Mobile Manipulation

Controller Type
---------------
The end-effector motion is controlled using a task-space feedback controller with
feedforward plus proportional–integral (PI) feedback, as described in
Modern Robotics (Lynch & Park).

The commanded body twist V is computed as:
V = Ad_X^{-1}X_d V_d + Kp X_err + Ki ∫ X_err dt

where X_err is the configuration error between the actual and desired
end-effector configurations.

Feedback Gains
--------------
Proportional gain:
Kp = 5 * I_6

Integral gain:
Ki = 0.5 * I_6

Time step:
dt = 0.01 s

Trajectory Generation
---------------------
The reference trajectory is generated using an 8-segment screw trajectory:
1. Move from initial end-effector pose to cube standoff (above cube)
2. Move from standoff to grasp pose
3. Close gripper (grasp)
4. Move back to standoff
5. Move to goal standoff
6. Move to goal grasp pose
7. Open gripper (release)
8. Return to goal standoff

Each trajectory segment is generated using ScrewTrajectory from the
Modern Robotics library.

NewTask: Cube Initial and Goal Configurations
---------------------------------------------
Initial cube configuration (T_sc_init):

T_sc_init =
[ 1  0  0  1.0
  0  1  0  0.0
  0  0  1  0.025
  0  0  0  1 ]

Goal cube configuration (T_sc_goal):

T_sc_goal =
[ 0  1  0   0.0
 -1  0  0  -1.0
  0  0  1   0.025
  0  0  0   1 ]

Results
-------
The file youbot_trajectory.csv contains the full robot configuration
trajectory for playback in CoppeliaSim.

The file Xerr.csv logs the six components of the end-effector configuration
error over time.

A plot of all six elements of X_err versus time is provided in Xerr_plot.pdf,
showing convergence of the error to zero, demonstrating stable closed-loop
behavior.

