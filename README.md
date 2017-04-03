# Geomagic Touch Mediated Haptic Interface

## Purpose
The Telehaptics Ros Python package allows users to drive the right arm of a Baxter research robot using the stylus of a Geomagic Touch. When the end effector of the arm experiences external force, the Geomagic Touch replicates the same forces on the user's hand - creating a closed-loop biofeedback interface.

## Node Network
![nodenetwork](https://github.com/stephanniec/baxter_telehaptics/blob/master/imgs/telehaptics_nodenetwork1.png)

> drawshape_ui.py

> commander.py

> velcon.py

## Launch Files

> simstate.launch

> telehaptics.launch

## Current Status
* Basic UI - done
* Velocity controller - done
* Geomagic Touch UI - WIP
* Force feedback node - WIP
