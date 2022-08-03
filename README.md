# ROS packages and code samples of software research projects implemented during my thesis.

## Kinematic controller in Katana manipulator
**Package:** Katana Tutorials

**Source code:** follow_online_trajectory.cpp

**Comments:** Implementation an online kinematic trajectory tracking algorithm for the End-effector of the Katana manipulator. The input pose is calculated by a motion planning algorithm that adapts the robot motion to a constantly changing surface.

## Stiffness controller in WidowX Manipulator
**Package:** widowx_torque_control_code/widowx_controller

**Source code:** widowx_stiffness_state_space_controller_5DOF.py & widowx_position_controller.py

**Comments:** Implementation in Python of an online kinematic position controller and a Stiffness controlle that is transforming forces and torques in the task space into joint torques of the widowx arm through the arbotix controller. Theoretical background for the stiffness controller can be found in [[1]](#1).

**Package:** widowx_torque_control_code/widowx_driver

**Source code:** widowx_torque_driver.py & widowx_position_driver.py

**Comments:** Implementation in Python of a position and torque drivers that tranfer the desired joint positions or torques to the robot via the Arbotix interface.

## Interactive Motion Planning on the Mesh of the deformable object
**Standalone source code:** expmapCL.cpp

**Comments:** Implementation in C++ of a Planning framework of interactive trajectories on the mesh representation of a deformable object. This code is updating the 2D representations of the incoming 3D mesh and calculates the appropriate transformations to calculate the desired pose that is provided as input to the previously described Stiffness controller in the WidowX robotic arm.

![alt-text](https://github.com/athdom/demo_code/tree/main/images/ezgif.com-gif-maker.gif)

_(This code is part of the publication "Interaction Control of a Robotic Manipulator with the Surface of a Deformable Object" - Under review)_

## Interactive Motion Planning over a moving surface perceived with RGB-D camera
**Standalone source code:** surface_interaction.cpp & surface_interaction.h

**Comments:** Implementation in C++ of an interactive planning algorithm over a moving surface perceived with an RGB-D camera. 
The PointCloud data of an RGB-D camera are transformed to a virtual camera that is assumed perpedicular to the moving surface. 
Restricted areas and obstacles perceived on the surface are avoided with a Navigation function controller implemented on the image plane. 

_(This code is part of the publication referenced in [[2]](#2).)_

## References
<a id="1">[1]</a> 
"Villani, L. and De Schutter, J., 2016. Force control. In Springer handbook of robotics (pp. 195-220). Springer, Cham."

<a id="2">[2]</a> 
Dometios AC, Zhou Y, Papageorgiou XS, Tzafestas CS, Asfour T. Vision-based online adaptation of motion primitives to dynamic surfaces: application to an interactive robotic wiping task. IEEE Robotics and Automation Letters. 2018 Jan 31;3(3):1410-7