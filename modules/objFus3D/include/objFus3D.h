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

#ifndef __OBJFUS3D_H__
#define __OBJFUS3D_H__

// Includes
#include <iostream>
#include <string>

// YARP libs
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/all.h>
#include <yarp/sig/Image.h>

// icub Libraries
#include "iCub/YarpCloud/CloudUtils.h"
#include "objFus3D_IDLServer.h"
#include "visThread.h"

//PCL libs
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/surface/mls.h>
#include <pcl/registration/ia_ransac.h>

// OpenCV libs
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



/**********************************************************
    PUBLIC METHODS
/**********************************************************/

/**********************************************************/
class FusionModule : public yarp::os::RFModule, public objFus3D_IDLServer
{
protected:

    // module parameters
    VisThread *visThrd;

    // Ports
    yarp::os::BufferedPort<yarp::os::Bottle>                            coordsInPort;
    yarp::os::BufferedPort<yarp::os::Bottle>                            trackerInPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    imgInPort;      // Port to receive camera images
    yarp::os::BufferedPort<yarp::os::Bottle>                            cloudsInPort;   // Buffered port to receive clouds.

    yarp::os::BufferedPort<yarp::os::Bottle>                            coordsOutPort;  // Buffered port to send clouds.
    yarp::os::BufferedPort<yarp::os::Bottle>                            cloudsOutPort;  // Buffered port to send clouds.
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    cropOutPort;    // Port to send out template image

    // rpc ports
    yarp::os::RpcServer                 rpcInPort;              // port to handle incoming commands
    yarp::os::RpcClient         		rpcObjRecPort;          //rpc port to communicate with objectReconst module


    // config variables
    bool        verbose;
    bool        pfTracker;      //sets whether an external pfTracker is running (true) or tracking is performed automatically by segmentation module.
    std::string cloudpath;      //path to folder with .ply files
    std::string filename;       //name of file to save recosntructed model.

    // Workflow variables
    bool tracking; 
    bool saving;
    bool closing;
    bool paused;
    bool initAlignment;

    // Algorithms parameters
    double ds_res;
    double mls_rad;
    double mls_usRad;
    double mls_usStep;
    double icp_maxIt;
    double icp_maxCorr;
    double icp_ranORT;
    double icp_transEp;


    //bool trackerInit;
    int NO_FILENUM;
    int STATE;

    // Algorithm variables
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr      cloud_in;       // Last registered pointcloud    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr      cloud_merged;   // Validated merged pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr      cloud_raw;      // Merged pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr      cloud_aligned;  // Cloud in after alignment

    /* functions */
    //bool                read(yarp::os::ConnectionReader &connection);
    bool                startTracker();
    bool                getPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_read);
    bool                filterCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter);
    bool                downsampleCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ds, const double res = 0.001);    

    bool                alignPointClouds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_from, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_align, Eigen::Matrix4f& transfMat);
    int                removeNaNs(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_noNaN, std::vector <int> nanInds);
    void                computeLocalFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);
    void                computeSurfaceNormals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);

public:

    // Thrift commands
    bool                        restart();
    bool                        track();
    bool                        mls(double rad = 0.03, double usRad = 0.005, double usStep = 0.003);
    bool                        icp(int maxIt = 100, double maxCorr = 0.03, double ranORT = 0.03, double transEp= 1e-6);
    bool                        ds(double res = 0.002);
    bool                        save(const std::string &name);
    bool                        pause();
    bool                        verb();
    bool                        initAlign();

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
