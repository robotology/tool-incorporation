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

#ifndef __TOOL3DSHOW_H__
#define __TOOL3DSHOW_H__

// Includes

#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/RateThread.h>

#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

// icub Libraries
#include <iCub/data3D/SurfaceMeshWithBoundingBox.h>
#include "iCub/YarpCloud/CloudUtils.h"

#include <tool3Dshow_IDLServer.h>
#include "visThread.h"

/**********************************************************
    PUBLIC METHODS
/**********************************************************/

/**********************************************************/
class ShowModule : public yarp::os::RFModule, public tool3Dshow_IDLServer
{
protected:
    /* module parameters */
    yarp::os::RpcServer handlerPort;  // port to handle incoming commands
    VisThread *visThrd;

    yarp::os::BufferedPort<iCub::data3D::SurfaceMeshWithBoundingBox> cloudsInPort; // Buffered port to receive clouds.

    std::string cloudpath; //path to folder with .ply files
    std::string cloudfile; //name of the .ply file to show
    bool closing;

public:

    // RPC Accesible methods
    bool accumClouds(bool accum);
    bool clearVis();
    bool showFileCloud(const std::string &cloudname);
    bool addNormals(double radSearch);
    bool addBoundingBox(bool minBB);
    //bool showCloud(const iCub::data3D::SurfaceMeshWithBoundingBox meshB);

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
