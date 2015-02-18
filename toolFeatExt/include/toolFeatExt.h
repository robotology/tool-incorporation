/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Tanis Mar
 * email:  tanis.mar@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef __TOOLFEATEXT_H__
#define __TOOLFEATEXT_H__

// Includes
#include <iostream>
#include <math.h>
#include <vector>
#include <ctime>

// YARP - iCub libs
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/Vocab.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>

//PCL libs
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>

//for the thrift interface
#include "ToolFeat3DwithOrient.h"
#include "VoxFeat.h"

/**********************************************************
    PUBLIC METHODS
/**********************************************************/

/**********************************************************/
class ToolFeatExt : public yarp::os::RFModule
{
protected:
    /* module parameters */
    yarp::os::RpcServer handlerPort;  // port to handle messages
    std::string path;            // path to folder with .ply or .pcd files
    std::string fname;           // name of the .ply or .pcd cloud file

// add the port to send out the features via thrift.

    /* class variables */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;// Point cloud

    bool verbose;
    int maxDepth;
    int binsPerDim;

    bool closing;
    bool cloudLoaded;

    /* functions*/
    bool transformFrame(const yarp::sig::Matrix toolPose = yarp::math::eye(4));
    bool loadCloud();
    int  computeFeats();

    /* helper functions */
    yarp::sig::Matrix   eigMat2yarpMat(const Eigen::MatrixXf eigMat);
    Eigen::MatrixXf     yarpMat2eigMat(const yarp::sig::Matrix yarpMat);
    yarp::sig::Matrix   getCanonicalRotMat(const int deg = 0);

public:

    // Access to class variables
    bool setVerbose(const std::string& verb);

    // RF modules overrides
    bool						configure(yarp::os::ResourceFinder &rf);
    bool						interruptModule();
    bool						close();
    bool						updateModule();
    double						getPeriod();
    bool                        respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);


};

#endif
