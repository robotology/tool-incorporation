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

#ifndef __CLOUDUTILS_H__
#define __CLOUDUTILS_H__

// Includes
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <ctime>
#include <string>

// YARP includes
//#include <yarp/os/all.h>
//#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

//PCL includes
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

// iCub includes
#include <iCub/data3D/SurfaceMeshWithBoundingBox.h>
#include <iCub/data3D/minBoundBox.h>
#include <iCub/data3D/RGBA.h>

namespace iCub {
    namespace YarpCloud {
        class CloudUtils;
     }
}

/**
 * @brief The iCub::YarpCloud::CloudUtils class
 */
class iCub::YarpCloud::CloudUtils {
public: 

    static bool        loadCloud(const std::string &cloudpath, const std::string &cloudname, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to);
    static void        savePointsPly(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string& savepath, const std::string& savename, const int addNum = -1);

    static void        mesh2cloud(const iCub::data3D::SurfaceMeshWithBoundingBox& cloudB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    static void        cloud2mesh(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, iCub::data3D::SurfaceMeshWithBoundingBox& meshB, const std::string &cloudname = "cloud");

    static yarp::sig::Matrix   eigMat2yarpMat(const Eigen::MatrixXf eigMat);
    static Eigen::MatrixXf     yarpMat2eigMat(const yarp::sig::Matrix yarpMat);

        
};
#endif //__CLOUDUTILS_H__

