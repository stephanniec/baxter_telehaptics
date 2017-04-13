# Geomagic Touch Mediated Haptic Interface

## Purpose
The Telehaptics Ros Python package allows users to drive the right arm of a Baxter research robot using the stylus of a Geomagic Touch. When the end effector of the arm experiences external force, the Geomagic Touch replicates the same forces on the user's hand - creating a closed-loop biofeedback interface.

## Node Network
<b>joystick_reference_targets.py</b><br>
This node generates target poses using the position values published by joy_node. As a safety precaution, it also ensures that Baxter will not move unless the L1 trigger on a PS3 console is held down.

<b>velocity_control.py</b><br>
This node finds and uses the body Jacobian and twists of Baxter's right arm to compute the angular velocities needed to move Baxter's right arm from its current pose to a target pose. It employs the naive damped least-squares method to eliminate jerky movements near singularities.

<b>gripper_control.py</b><br>
This node closes Baxter's right gripper when the R1 trigger on a PS3 console is held down.

## Launch Files

> simstate.launch

> basicsys.launch

> joysys.launch

## PS3 Controls
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
