# Geomagic Touch Mediated Haptic Interface

## Purpose
The Telehaptics Ros Python package allows users to drive the right arm of a Baxter or Sawyer research robot using the stylus of a Geomagic Touch (formerly the Phantom Omni). If external force is applied to the end effector, the Geomagic Touch will remotely generate the same forces on the user's hand - creating a closed-loop biofeedback interface.

## Table of Contents
[PS3 Node Network](#ps3-node-network)<br>
[Geomagic Touch Node Network](#geomagic-touch-node-network)<br>
[Launch Files](#launch-files)<br>
[PS3 Controls](#ps3-controls)<br>
[Set-up Instructions](#set-up-instructions)<br>
[Current Status](#current-status)

## PS3 Node Network
<b>joystick_reference_targets.py</b><br>
This node generates target poses using the position values published by `joy_node`. As a safety precaution, it also ensures that Baxter will not move unless the L1 trigger on a PS3 console is held down.

<b>velocity_control.py</b><br>
This node finds and uses the body Jacobian and twists of Baxter's right arm to compute the angular velocities needed to move Baxter's right arm from its current pose to a target pose. It employs the naive damped least-squares method to eliminate jerky movements near singularities.

(Sawyer equivalent: `sawyer_velocity_control.py`)

<b>gripper_control.py</b><br>
This node closes Baxter's right gripper when the R1 trigger on a PS3 console is held down.

(Sawyer equivalent: `sawyer_gripper_control.py`)

## Geomagic Touch Node Network
<b>omni_reference_targets2.py</b><br>
This node generates target poses by scaling and mapping the transform between the Touch's stylus frame and its base frame. A subscriber to `omni1_joint_states` also associates rotations about the x-axis of the stylus frame to rotations about the z-axis of the target frame.

<b>velocity_control.py</b><br>
The velocity control script used to control Baxter via the Touch is the same as that which was used to control the robot via the PS3 controller.

<b>omni_gripper_control.py</b><br>
This node closes Baxter's right gripper when the white button on the Touch's stylus is held down.

## Launch Files

`simstate.launch` : starts up a basic rviz simulation of Baxter which displays individual joint angles that can be manipulated using slider bars

`basicsys.launch` : starts up a basic velocity controller which tells Baxter to move to hardcoded poses

`joysys.launch` : starts up the PS3 Baxter controller demo

`sawyer_joysys.launch` : starts up the PS3 Sawyer controller demo

`omnisys.launch` : starts up the Touch Baxter controller demo

## PS3 Controls
![ps3console](https://github.com/stephanniec/baxter_telehaptics/blob/master/imgs/ps3_schematic.png)<br>
~~~
L1 button : Hold down to enable robot movement
R1 button : Hold down to close robot end-effector

Left stick (L/R) : Horizontal movement along the Y-axis
Left stick (U/D) : Vertical movement along the Z-axis
Right stick (U/D) : Horizontal movement towards or away from user along the X-axis
Right stick (L/R) : Rotates the gripper clockwise or counterclockwise
~~~

## Set-up Instructions
After cloning `baxter_telehaptics` into a catkin workspace, please install the following auxiliary packages:

* `joy`
* `hrl-kdl`
* `sawyer_robot`
* `baxter_interface`

## Current Status
* PS3 Controller UI - done
* Velocity controller - done
* Geomagic Touch UI - done
* Force feedback node - WIP
