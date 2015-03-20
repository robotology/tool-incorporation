#ifndef __PCLVISTHRD_H__
#define __PCLVISTHRD_H__

// Includes

#include <iostream>
#include <string>

#include <yarp/os/RateThread.h>
#include <yarp/os/Network.h>

#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#define NOT_INIT     0
#define INIT   1

class VisThread: public yarp::os::RateThread
{
protected:

    // INTERNAL VARIABLES:
    bool update;
    boost::mutex updateModelMutex;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    std::string id;

    bool initialized;
    bool addClouds;


public:
    // CONSTRUCTOR
    VisThread(int period, const std::string &_cloudname);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();
    
    // Update the displayed cloud
    void updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);

    // Switch between plotting clouds together or not
    void switchAddClouds();
};

#endif


