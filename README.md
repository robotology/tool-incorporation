## Overview

This repository contains the modules to perform tool incorporation, visualization and feature extraction, in order to fully represent a tool on the iCub's hand, as well as a library of YARP functions to deal with common PCL Pointcloud operations. Thus, all of its functions assume that the robot is holing a tool in its hand.

## Tool Incorporation
Tool incorporation referers to the process whereby the iCub can obtain a fast and reliable estimation of a tool’s geometry, reach and pose with respect to the iCub’s hand, in order to attach it to the robot’s kinematic chain, thereby enabling dexterous tool use. All the functions involved are performed by the module __tool-incorporation__

The main functionalities provided by this module are the following:
- Tool recognition through communication with the onTheFlyRecognition app.
- Tool 3D model reconstruction from stereo-vision.
- Pose estimation using alignment for known models.
- Pose estimatin by means of tool's intrinsic reference frame estimation for new models.
- Tool tip estimation by from the tool model and estimated pose
- Communication with toolFeatExt for OMS-EGI feature extraction.

For detailed help on the function calls to perform this, as well as their parameters and outputs, call the rpc help. 


### Tool Recognition
Tool recognition is so that its model can be loaded if the tool is known, or its visual appearance learned otherwise. 

Detailed documentation at: 
XXX


- [toolIncorporator] This module deals with 3D tool exploration, partial reconstruction, alignment, pose estimation, etc. 



xxx Explain the location of the sample clouds and how to add new ones.


## Visualization

- [show3D] This module opens a pointcloud visualizer, and adds a series of functionalities in top of the standard viewer, including visualization of normals, bounding boxes, tooltip, filtering, features, etc. Detailed documentation at: 

## Feature Extraction
These functions are distrubuted in the following 3 modules:

- [toolFeatExt] This module performs feature extraction from the oriented cloud models. In particular, it extract the OMS-EGI descriptor, described in XXX. 

## PCL-YARP library




