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
#include <yarp/os/RpcServer.h>
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

#include <tool3DFeat_IDLServer.h>

/**********************************************************
    PUBLIC METHODS
/**********************************************************/

/**********************************************************/
class ToolFeatExt : public yarp::os::RFModule, public tool3DFeat_IDLServer
{
protected:
    /* module parameters */
    yarp::os::RpcServer rpcInPort;  // port to handle incoming commands

    yarp::os::Port      feat3DoutPort; // Port where the features of the tool are send out (as a thrift Tool3DwithOrient struct)
    std::string path;            // path to folder with .ply or .pcd files
    std::string fname;           // name of the .ply or .pcd cloud file

// add the port to send out the features via thrift.

    /* class variables */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_orig; // Point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;      // Point cloud of the transformed model
    yarp::sig::Matrix                   rotMat;     // Rotation Matrix specifying grasp pose

    bool verbose;
    int maxDepth;
    int binsPerDim;

    bool closing;
    bool cloudLoaded;
    bool cloudTransformed;

    /* functions*/
    bool loadCloud();
    bool transformFrame(const yarp::sig::Matrix& toolPose = yarp::math::eye(4,4));
    int  computeFeats();

    /* helper functions */
    yarp::sig::Matrix   eigMat2yarpMat(const Eigen::MatrixXf eigMat);
    Eigen::MatrixXf     yarpMat2eigMat(const yarp::sig::Matrix yarpMat);

public:

    // RPC Accesible methods
    bool getFeats();
    bool setPose(const yarp::sig::Matrix& rotMat);
    bool setCanonicalPose(const int deg = 0);

    bool bins(const int binsN = 2);
    bool depth(const int depthN = 2);
    bool setName(const std::string& cloudname = "cloud_merged.ply");
    bool setVerbose(const std::string& verb);
    bool help_commands();

    // module control //
    bool						attach(yarp::os::RpcServer &source);
    bool						quit();

    // RF modules overrides
    bool						configure(yarp::os::ResourceFinder &rf);
    bool						interruptModule();
    bool						close();
    bool						updateModule();
    double						getPeriod();
};

#endif
