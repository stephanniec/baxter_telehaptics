# Geomagic Touch Mediated Haptic Interface

## Purpose
The Telehaptics Ros Python package allows users to drive the right arm of a Baxter or Sawyer research robot using the stylus of a Geomagic Touch. If external force is applied to the end effector, the Geomagic Touch will remotely generate the same forces on the user's hand - creating a closed-loop biofeedback interface.

## Node Network
<b>joystick_reference_targets.py</b><br>
This node generates target poses using the position values published by joy_node. As a safety precaution, it also ensures that Baxter will not move unless the L1 trigger on a PS3 console is held down.

<b>velocity_control.py</b><br>
This node finds and uses the body Jacobian and twists of Baxter's right arm to compute the angular velocities needed to move Baxter's right arm from its current pose to a target pose. It employs the naive damped least-squares method to eliminate jerky movements near singularities.

(Sawyer equivalent: sawyer_velocity_control.py)

<b>gripper_control.py</b><br>
This node closes Baxter's right gripper when the R1 trigger on a PS3 console is held down.

(Sawyer equivalent: sawyer_gripper_control.py)

## Launch Files

> simstate.launch

> basicsys.launch

> joysys.launch

> sawyer_joysys.launch

## PS3 Controls
![ps3console](https://github.com/stephanniec/baxter_telehaptics/blob/master/imgs/ps3_schematic.png)<br>
<b>L1 button</b> || Hold down to enable robot movement<br>
<b>R1 button</b> || Hold down to close robot end-effector

<b>Left stick (L/R)</b> || Horizontal movement along the Y-axis<br>
<b>Left stick (U/D)</b> || Vertical movement along the Z-axis<br>
<b>Right stick (U/D)</b> || Horizontal movement towards or away from user along the X-axis<br>
<b>Right stick (L/R)</b> || Rotates the gripper clockwise or counterclockwise

## Current Status
* PS3 Controller UI - done
* Velocity controller - done
* Geomagic Touch UI - WIP
* Force feedback node - WIP
