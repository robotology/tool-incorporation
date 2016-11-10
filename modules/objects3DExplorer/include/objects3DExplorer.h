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

#ifndef __OBJECTS3DEXPLORER_H__
#define __OBJECTS3DEXPLORER_H__

//Includes
#include <stdio.h>
#include <time.h>
#include <string>
#include <algorithm>
#include <numeric>

//YARP libs
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>

#include "iCub/YarpCloud/CloudUtils.h" 

//PCL libs
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>


#include <cv.h>



/**********************************************************/
class Objects3DExplorer : public yarp::os::RFModule
{
protected:
    /* variables */ 
    // ports
    yarp::os::BufferedPort<yarp::os::Bottle>                            cloudsInPort;
    yarp::os::BufferedPort<yarp::os::Bottle>                            cloudsOutPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> >    imgInPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> >    imgOutPort;

    // rpc ports
    yarp::os::RpcServer                 rpcPort;
    yarp::os::RpcClient         		rpcObjRecPort;          //rpc port to communicate with objectReconst module
    yarp::os::RpcClient         		rpcVisualizerPort;      //rpc port to communicate with tool3Dshow module to display pointcloud
    yarp::os::RpcClient         		rpcFeatExtPort;         //rpc port to communicate with the 3D feature extraction module
    yarp::os::RpcClient         		rpcClassifierPort;         //rpc port to communicate with the 3D feature extraction module

    // Drivers
    yarp::dev::PolyDriver               driverG;
    yarp::dev::PolyDriver               driverL;
    yarp::dev::PolyDriver               driverR;
    yarp::dev::PolyDriver               driverHL;
    yarp::dev::PolyDriver               driverHR;

    yarp::dev::IGazeControl             *iGaze;
    yarp::dev::ICartesianControl        *iCartCtrlL;
    yarp::dev::ICartesianControl        *iCartCtrlR;
    yarp::dev::ICartesianControl        *iCartCtrl;
    yarp::dev::ICartesianControl        *otherHandCtrl;

    // config variables
    std::string                         hand;
    std::string                         camera;
    std::string                         robot;    
    std::string                         cloudsPathFrom;
    std::string                         cloudsPathTo;
    std::string                         saveName;
    bool                                saving;
    bool                                verbose;
    bool                                handFrame;

    // icp variables
    int                                 icp_maxIt;
    double                              icp_maxCorr;
    double                              icp_ranORT;
    double                              icp_transEp;

    // mls variables
    double                              mls_rad;
    double                              mls_usRad;
    double                              mls_usStep;

    // noise params
    double                              noise_mean;
    double                              noise_sigma;

    // module parameters
    bool                                cloudLoaded;
    bool                                poseFound;
    bool                                symFound;
    bool                                initAlignment;
    bool                                seg2D;
    bool                                closing;
    bool                                displayTooltip;
    int                                 numCloudsSaved;
    int                                 NO_FILENUM;

    struct                              Point2D {int u; int v;};
    struct                              Point3D {double x;double y; double z;};
    struct                              Plane3D {double a;double b; double c; double d;};
    int                                 purple[3], red[3], green[3], blue[3], orange[3];

    yarp::sig::Matrix                           toolPose;
    Point3D                                     tooltip, tooltipCanon;
    Point2D                                     tooltip2D, handFrame2D;
    int                                         bbsize, imgW, imgH;

    yarp::sig::Vector                           eigenValues;
    std::vector<Plane3D>                        eigenPlanes;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr      cloud_temp;     // Temporal pointcloud for validation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr      cloud_model;    // Validated merged pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr      cloud_pose;     // Cloud oriented on estimated Pose


    /* functions*/
    
    /* Actions */
    bool                turnHand(const int rotDegX = 0, const int rotDegY = 0, const bool followTool = false);
    bool                exploreTool(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rec_merged, const std::string &label, const bool flag2D = true, const bool flag3D = true);
    bool                lookAtTool();
    bool                lookAtHand();
    bool                lookAround();
    double              adaptDepth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double spatial_distance);

    /* Object info from Cloud */
    bool                loadCloud(const std::string &cloud_name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    bool                saveCloud(const std::string &cloud_name, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    bool                getPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rec, double segParam = -1.0, double handRad = 0.06);
    bool                sendPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    bool                findPoseAlign(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr poseCloud, yarp::sig::Matrix &pose, const int T = 5);
    bool                alignPointClouds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_from, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned, Eigen::Matrix4f& transfMat, double fitScore);
    bool                alignWithScale(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_from, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned, Eigen::Matrix4f& transfMat, int numsteps = 10, double stepsize = 0.02);
    bool                checkGrasp(const yarp::sig::Matrix &pose);

    bool                findTooltipCanon(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloud, Point3D &ttCanon);    
    bool                findSyms(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, yarp::sig::Matrix &pose, int K = 5, bool vis = true);
    bool                findPlanes(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<Plane3D> &mainplanes, std::vector< Eigen::Vector3f> &eigVec, Eigen::Vector3f &eigVal, Eigen::Vector3f &mc );
    bool                findTooltipSym(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const yarp::sig::Matrix &pose, Point3D& ttSym, double effW = 0.8);
    Plane3D             main2unitPlane(const Plane3D main);

    bool                placeTipOnPose(const Point3D &ttCanon, const yarp::sig::Matrix &pose, Point3D &tooltipTrans);
    bool                paramFromPose(const yarp::sig::Matrix &pose, double &ori, double &displ, double &tilt, double &shift);
    bool                poseFromParam(const double ori, const double displ, const double tilt, const double shift, yarp::sig::Matrix &pose);
    bool                setToolPose(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const yarp::sig::Matrix &pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInPose);


    bool                extractFeats();
    bool                getAffordances(yarp::os::Bottle &affBottle, bool allAffs = false);
    int                 getTPindex(const std::string &tool, const yarp::sig::Matrix &pose);
    bool                getAffProps(const yarp::sig::Matrix &affMatrix, yarp::os::Property &affProps);

    /* Learn - recognize tools from vision */

    bool                learn(const std::string &label, bool depthBB = true);
    bool                recognize(std::string &label, bool depthBB = true);

    /* Cloud Utils */
    bool                frame2Hand(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trans);
    bool                cloud2canonical(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_canon);
    bool                addNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double mean, double sigma);
    bool                addPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Point3D coords, bool shift = false);     // overload default color
    bool                addPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Point3D coords, int color[], bool shift = false);
    bool                showTooltip(const Point3D coords, int color[]);
    bool                showRefFrame(const Point3D center, const std::vector<Plane3D> &refPlanes);
    bool                showLine(const Point3D coordsIni, const Point3D coordsEnd, int color[]);
    bool                changeCloudColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    bool                changeCloudColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int color[]); // overload default color
    bool                scaleCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scaled, double scale = 1.0);
    bool                downsampleCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ds, double res = 0.001);
    bool                filterCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, double thr = 3.0);
    void                computeLocalFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);
    void                computeSurfaceNormals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);    
    int                 getSign(const double x);
    
    /* Configuration commands */ 
    bool                setVerbose(const std::string& verb);
    bool                showTipProj(const std::string& tipF);
    bool                setHandFrame(const std::string& hf);
    bool                setInitialAlignment(const std::string& fpfh);
    bool                setSeg(const std::string& seg);
    bool                setSaving(const std::string& sav);
    bool                changeSaveName(const std::string& fname);
       
public:

   // RF modules overrides    
    bool						configure(yarp::os::ResourceFinder &rf);
    bool						interruptModule();
    bool						close();
    bool						updateModule();
    double						getPeriod();

    bool                        respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
};


#endif

