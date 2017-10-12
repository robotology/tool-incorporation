## Overview

This repository contains the modules to perform tool incorporation, visualization and feature extraction, in order to fully represent a tool on the iCub's hand, as well as a library of YARP functions to deal with common PCL Pointcloud operations. Thus, all of its functions assume that the robot is holing a tool in its hand.

## Modules
### toolIncorporator
The tool-incorporator module provides all the functionalities required for tool incorporation, with which we refer to the process whereby the iCub can obtain a fast and reliable estimation of a tool’s geometry, reach and pose with respect to the iCub’s hand, in order to attach it to the robot’s kinematic chain, thereby enabling dexterous tool use. 

The main functionalities provided by this module are the following:
- Tool recognition through communication with the onTheFlyRecognition app: Naturally, for recognition to be successful, prior training is required. A sample dataset of 3 tools (rake, stick, shovel) is provided [here](https://github.com/robotology/tool-incorporation/tree/master/app/dataset). In order to be available by the OTFR pipeline, it has to be copied to the local `himrep` installed context.
- Tool 3D model reconstruction from stereo-vision.
- Pose estimation using alignment for known models.
- Pose estimatin by means of tool's intrinsic reference frame estimation for new models.
- Tool tip estimation by from the tool model and estimated pose
- Communication with toolFeatExt for OMS-EGI feature extraction.

For detailed help on the function calls to perform this, as well as their parameters and outputs, call the rpc help. 

### Show3D
The module show3D provides an enhanced pointcloud visualized. On top of the functionalities provided by the standard PCL classifier, show3D adds a series of functionalities accessible through rpc calls, including visualization of normals, bounding boxes, tooltip, filtering, features, etc. 

### toolFeatExt
The toolFeatExt module performs feature extraction from oriented cloud models. In particular, it extract the OMS-EGI descriptor, introduced in the following paper: 

- T. Mar, V. Tikhanoff, G. Metta, L. Natale "Multi-model approach based on 3D functional features for tool affordance learning in robotics", _Humanoids 2015_, Seoul. 

## YarpCloud Library

Additionally, this repository provides a simple library to facilitate operating with PCL pointclouds in YARP. The offered functionalities include

- Loading and saving pointclouds from/into different formats (ply, pcd, off, coff)
- Transformation from pointcloud to bottle and viceversa, for reading and writing them onto ports.
- Transformation from Eigen matrices (used in PCL) to YARP matrices.
- Adding noise to the current pointcloud. 
- Downsampling and scaling pointclouds
- Changing cloud color for clear multiple cloud visualization


## Dependencies
All modules require
- [YARP](https://github.com/robotology/yarp)
- [iCub](https://github.com/robotology/icub-main)
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)
- [Pointcloud Library](http://pointclouds.org/) (PCL)

## How to compile
In `Linux systems` code can be compiled as follows:
```
git clone https://github.com/robotology/tool-incorporation.git
cd tool-incorporation
mkdir build; cd build
ccmake ..
make install
```
This will have compiled and installed the YarpCloud library, required to compile the rest of the modules. 
Now they can be compiled, in the following way.
On the build folder, type
```
ccmake ..
set BUILD_TOOLINCORPORATOR to ON 
set BUILD_SHOW3D to ON 
set BUILD_TOOLFEATEXT to ON 
make install
``` 

## Links to related repositories

 * Although this repository is independent, most of its methods are intended to work together with the modules and applications in the [tool-affordances](https://github.com/robotology/tool-incorporation) repository.
 * In order be able to recognize given tools, a classifier has to have been trained beforehand. In our applications, we use the methods provided on the [onTheFlyRecognition](https://github.com/robotology/onthefly-recognition) application.
 
 

## License
Material included here is Copyright of _iCub Facility - Istituto Italiano di Tecnologia_
and is released under the terms of the GPL v2.0 or later. See the file LICENSE for details.


