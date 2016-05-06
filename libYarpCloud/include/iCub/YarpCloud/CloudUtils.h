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
#include <string>

// YARP includes
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

//PCL includes
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

// iCub includes
//#include <iCub/data3D/SurfaceMeshWithBoundingBox.h>
//#include <iCub/data3D/minBoundBox.h>
//#include <iCub/data3D/RGBA.h>

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
    /**
     * @brief loadCloud Loads a cloud from the given path and .ply, .pcd or .(c)off file to a PCL PointXYZRGB::Ptr type cloud_to.
     * @param cloudpath Path where the cloud file is found
     * @param cloudname file name and extension (.ply or .pcd) of the desired cloud to load.
     * @param cloud_to  Output variable containing the 3D pointcloud
     * @return Success in finding and loading the cloud from the file.
     */
    static bool        loadCloud(const std::string &cloudpath, const std::string &cloudname, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to);

    /**
     * @brief loadOFFFile Loads a cloud from .off or .coff file into a PCL PointXYZRGB::Ptr type cloud_to. For .off files, default color red is used.
     * @param cloudpath Path where the cloud file is found (with format)
     * @param cloud_to  Output variable containing the 3D pointcloud
     */
    static bool        loadOFFFile(const std::string &cloudpath, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to);


    /**
     * @brief savePointsPly Saves a PointXYZRGB Cloud into a .ply ascii file
     * @param cloud Boost Ptr to the cloud to be saved
     * @param savepath Path where the cloud should be saved
     * @param savename Desired name (without extension) for the .ply file
     * @param addNum In order to save multiple registrations of the same object, a number can be added after the name. -1 to not add any number.
     */
    static void        savePointsPly(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string& savepath, const std::string& savename, int &addNum);

    /**
     * @brief savePointsOff Saves a PointXYZRGB Cloud into a .off ascii file
     * @param cloud Boost Ptr to the cloud to be saved
     * @param savepath Path where the cloud should be saved
     * @param savename Desired name (without extension) for the .off file
     * @param addNum In order to save multiple registrations of the same object, a number can be added after the name. -1 to not add any number.
     */
    static void        savePointsOff(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string& savepath, const std::string& savename, int &addNum);


    /**
     * @brief bottle2cloud Converts a Bottle structured as a list of XYZ points into a PCL PointXYZ pointcloud
     * @param cloudB Input reference to the Bottle which contains a list of size 3 (XYZ) for each cloud point.
     * @param cloud Output boost pointer to the PCL PointXYZRGB pointcloud to be written
     */
    static void        bottle2cloud(const yarp::os::Bottle& cloudB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    /**
     * @brief cloud2bottle Converts a PCL PointXYZ(RGB) pointcloud into a YARP Bottle, consisting of a list of size 3 (XYZ) for each point.
     * @param cloud Input boost pointer to the PCL PointXYZ(RGB) pointcloud to be converted
     * @param cloudB Outupt reference to the Bottle which contains a list of size 3 (XYZ) for each cloud point to which the pointcloud will be converted.
     */
    static void        cloud2bottle(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, yarp::os::Bottle& cloudB);

     /**
     * @brief eigMat2yarpMat Converts a matrix from Eigen::MatrixXf format to yarp::sig::Matrix format.
     * @param eigMat input Eigen::MatrixXf.
     * @return Outputs yarp::sig::Matrix.
     */
    static yarp::sig::Matrix   eigMat2yarpMat(const Eigen::MatrixXf eigMat);

    /**
     * @brief yarpMat2eigMat Converts a matrix from yarp::sig::Matrix format to Eigen::MatrixXf format
     * @param yarpMat input yarp::sig::Matrix
     * @return Output Eigen::MatrixXf.
     */
    static Eigen::MatrixXf     yarpMat2eigMat(const yarp::sig::Matrix yarpMat);

        
};
#endif //__CLOUDUTILS_H__

