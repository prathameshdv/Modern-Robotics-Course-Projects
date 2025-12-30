#  Mobile Manipulation Capstone Project 

This repository contains my implementation of the **Modern Robotics Capstone Project**, where a **KUKA youBot mobile manipulator** performs an autonomous **pick-and-place task** using task-space feedback control and screw-theoretic trajectory planning.

The implementation follows the methods described in *Modern Robotics: Mechanics, Planning, and Control* by Lynch and Park.

---

##  Project Overview

The robot autonomously:
1. Moves from its initial configuration to a cube on the table  
2. Grasps the cube using a parallel gripper  
3. Transports the cube to a target location  
4. Places the cube at the desired final pose  

The project integrates:
- Screw-theoretic trajectory generation
- Task-space feedback control
- Feedforward + PI control
- Jacobian-based inverse kinematics

---

## Motion Planning

The reference motion for the end-effector is generated using screw-theoretic
trajectory interpolation in SE(3). The pick-and-place task is decomposed into
eight sequential segments, including approach, grasp, transport, and placement
motions.

### Planning Parameters
- Trajectory type: ScrewTrajectory in SE(3)
- Number of segments: 8
- Time step: `dt = 0.01 s`
- Interpolation order: Cubic time scaling

---

## Robot Kinematics

The robot kinematics are modeled using the product of exponentials formulation
for the manipulator arm and rigid-body transformations for the mobile base. The
end-effector pose is computed by chaining the chassis, base, and arm transforms.

### Kinematic Parameters
- Arm representation: Body-frame screw axes (`Blist`)
- Arm home configuration: `M0e`
- Base-to-arm transform: `Tb0`
- Degrees of freedom: 5 (arm) + 4 (wheels)

---

## Wheel Motion and Odometry

The motion of the mobile base is governed by a kinematic odometry model for the
four omnidirectional wheels of the youBot. Wheel angular velocities are mapped
to a body-frame chassis twist and integrated over time.

### Wheel Parameters
- Wheel radius: `r = 0.0475 m`
- Half-length of chassis: `l = 0.235 m`
- Half-width of chassis: `w = 0.15 m`
- Integration method: Body-frame twist integration

---

## Control Strategy

A task-space feedback controller is used:

V = Ad\_{X⁻¹X_d} V_d + Kp X_err + Ki ∫ X_err dt

where:
- X_err is the configuration error in SE(3)
- V_d is the feedforward twist from the reference trajectory

### Controller Parameters
- Proportional gain: `Kp = 5 · I₆`
- Integral gain: `Ki = 0.5 · I₆`
- Time step: `dt = 0.01 s`

## 📊 Results

### Full System Demonstration


https://github.com/user-attachments/assets/da9114d4-3971-4300-b530-d85385e93674




This video shows the complete execution of the mobile manipulation task in
CoppeliaSim, including navigation, grasping, transportation, and placement
of the cube. The robot tracks a screw-theoretic reference trajectory using
task-space feedback control while coordinating both the mobile base and
manipulator arm.

---

###  End-Effector Tracking Error

The plot below shows the six components of the end-effector configuration
error \(X_{\text{err}}\) over time, demonstrating convergence of both
position and orientation errors.

📄 **Error Plot:** [`Xerr_plot.pdf`](results/Xerr_plot.pdf)

