/*
 * VISUALIZER THREAD for 3D OBJECT DISPLAY
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Tanis Mar
 * email: tanis.mar@iit.it
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

#ifndef __PCLVISTHRD_H__
#define __PCLVISTHRD_H__

// Includes

#include <iostream>
#include <math.h> 
#include <string>
#include <sstream>

#include <yarp/os/RateThread.h>
#include <yarp/os/Network.h>

#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/surface/mls.h>


class VisThread: public yarp::os::RateThread
{
protected:
    // EXTERNAL VARIABLES: set on call
    std::string id;

    // INTERNAL VARIABLES:
    // Mutex cotnrol
    bool update;
    boost::mutex updateModelMutex;

    // Visualizer global variables
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_normalColors;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    // Flow control flags
    bool initialized;
    bool addClouds;
    bool clearing;
    bool updatingCloud;
    bool displayNormals;
    bool displayOMSEGI;    
    bool displayFilt;
    bool normalsComputed;
    bool displayBB;
    bool displaySphere;
    bool displayArrow;

    // Processing parameters
    int dBBstyle;
    double dNradSearch;
    bool dNcolored;
    float dEGIres;
    bool dEGIspheres;

    bool dFror;
    bool dFsor;
    bool dFmls;
    bool dFds;

    int sphColor[3];
    std::vector<double> sphCoords;

    int arrowColor[3];
    std::vector<double> arrowCoordsIni;
    std::vector<double> arrowCoordsEnd;
    int arrowNum;

    
    void updateVis();
    void plotNewCloud();
    void plotBB(int typeBB = 2);
    void plotSphere(const std::vector<double> &coords, const int color[]);
    void plotArrow(const std::vector<double> &coordsIni,const std::vector<double> &coordsEnd, const int color[]);
    void plotNormals(double rS = 0.01, bool normCol = false);
    void plotOMSEGI(double res = 0.02, bool plotHist = true);    
    void filterCloud(bool rorF = false, bool sorF = false, bool mlsF = false, bool dsF = false);

public:
    // CONSTRUCTOR
    VisThread(int period, const std::string &_cloudname);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();

    /**
     * @brief addNormals - Sets flow to allow normals to be computed and displayed on the visualizer.
     * @param rS - radiusSearch for consider nerighbors to compute surface normal.
     * @param normCol - determines whether the normals will be shown as vectors or colorwise.
     */
    void addNormals(double rS, bool normCol = false);         // Function called from main module to set parameters and unlock update


    /**
     * @brief addArrow- Displays an arrow with given coordinates and color
     * @param start - starting coordinates
     * @param end - ending coordinates
     * @param color - color (RGB)
     */
    void addArrow(const std::vector<double> &coordsIni, const std::vector<double> &coordsEnd, const std::vector<int> &color);

    /**
     * @brief addSphere- Displays a sphere at given coordinates and a given color
     * @param coords - starting coordinates
     * @param color - color (RGB)
     */
    void addSphere(const std::vector<double> &coords, const std::vector<int> &color);


    /**
     * @brief addOMSEGI - Sets flow to allow voxel normal histograms to be computed and displayed on the visualizer
     * @param res - resolution onf the voxels within which the EGIs are computed
     */
    void addOMSEGI(double res = 0.01, bool plotHist = true);         // Function called from main module to set parameters and unlock update


    /**
     * @brief addBoundingBox - Sets flow to allow bounding box to be computed and added on display update.
     * @param minBB - true for oriented (minimum) boinding box. False for axis aligned bounding box
     */
    void addBoundingBox(int typeBB);    // Function called from main module to set parameters and unlock update

    /**
     * @brief clearVisualizer - Clears all the figures (except the coordinate system) from the display.
     */
    void clearVisualizer();

    /**
     * @brief updateCloud - Updates the displayed cloud with the received one.
     * @param cloud_in - Input cloud to be displayed
     */
    void updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);

    /**
     * @brief filter - Function to apply and show different filtering processes to the displayed cloud.
     * @param ror - bool: Activates RadiusOutlierRemoval (rad = 0.05, minNeigh = 5).
     * @param sor - bool: Activates StatisticalOutlierRemoval (meanK = 20).
     * @param mls - bool: Activates MovingLeastSquares (rad = 0.02, order 2, usRad = 0.005, usStep = 0.003).
     * @param ds - bool: Activates Voxel Grid Downsampling (rad = 0.002).
     * @return true/false on showing the poitnclouddar =
     */
     void filter(bool ror = false, bool sor = false, bool mks = false, bool ds = false);


    /**
     * @brief accumulateClouds - Selects between displaying clouds together or not
     * @param accum - True to accumulate clouds on display. False to display them independently
     */
    void accumulateClouds(bool accum);


};

#endif


