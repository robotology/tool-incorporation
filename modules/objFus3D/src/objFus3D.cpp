/*
 * OBJECT 3D RECONSTRUCTION MODULE
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

#include "objFus3D.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::YarpCloud;

/************************************************************************/
//                          PUBLIC METHODS
/************************************************************************/

//                          THRIFT RPC CALLS
/************************************************************************/
bool FusionModule::accumClouds(bool accum)
{
    visThrd->accumulateClouds(accum);
    return true;
}

bool FusionModule::clearVis()
{
    visThrd->clearVisualizer();
    return true;
}



bool FusionModule::addNormals(double radSearch, bool normCol)
{
    visThrd->addNormals(radSearch, normCol);
    return true;
}

bool FusionModule::addFeats(double res, bool plotHist)
{
    visThrd->addOMSEGI(res,plotHist);
    return true;
}

bool FusionModule::addBoundingBox(int typeBB)
{
    visThrd->addBoundingBox(typeBB);
    return true;
}

bool FusionModule::quit()
{
    std::cout << "Quitting!" << std::endl;
    closing = true;
    return true;
}

/************************************************************************/
//                      RF MOUDLE Commands
/************************************************************************/
bool FusionModule::configure(yarp::os::ResourceFinder &rf)
{
    // Module RF parameters
    string name=rf.check("name",Value("objFus3D")).asString().c_str();
    string robot = rf.check("robot",Value("icub")).asString().c_str();
    verbose = rf.check("verbose", Value(true)).asBool();
    string cloudpath_file = rf.check("clouds",Value("cloudsPath.ini")).asString().c_str();
    rf.findFile(cloudpath_file.c_str());

    ResourceFinder cloudsRF;
    cloudsRF.setContext("objects3DModeler");
    cloudsRF.setDefaultConfigFile(cloudpath_file.c_str());
    cloudsRF.configure(0,NULL);

    // Set the path where new model will be saved
    string defSaveDir = "/saveModels";
    string cloudsSaveDir = rf.check("save")?rf.find("save").asString().c_str():defSaveDir;
    if (cloudsSaveDir[0]!='/')
        cloudsSaveDir="/"+cloudsSaveDir;
    cloudpath="."+cloudsSaveDir;

    yarp::os::mkdir_p(cloudpath.c_str());            // Create the save folder if it didnt exist

    printf("New clouds will be saved to: %s",cloudpath.c_str());

    //ports
    bool ret = true;
    ret = ret && coordsInPort.open("/"+name+"/coords:i");
    ret = ret && imgInPort.open("/"+name+"/img:i");
    ret = ret && cloudsInPort.open(("/"+name+"/clouds:i").c_str());                    // port to receive pointclouds from

    ret = ret && cloudsOutPort.open(("/"+name+"/coords:o").c_str());                   // port to send pointclouds to
    ret = ret && cloudsOutPort.open(("/"+name+"/clouds:o").c_str());                   // port to send pointclouds to
    ret = ret && cropOutPort.open(("/"+name+"/crop:o").c_str());                        // port to send croped image for template
    if (!ret){
        printf("\nProblems opening ports\n");
        return false;
    }

    // RPC ports
    bool retRPC = true;
    retRPC = rpcInPort.open(("/"+name+"/rpc:i").c_str());
    retRPC = retRPC && rpcObjRecPort.open(("/"+name+"/objrec:rpc").c_str());             // port to communicate with object reconstruction module
    if (!retRPC){
        printf("\nProblems opening RPC ports\n");
        return false;
    }

    //coordsInPort.setReader(*this);
    attach(rpcInPort);

    // Module rpc parameters
    closing = false;
    initAlignment =true;
    resolution =rf.check("resolution",Value(0.001)).asDouble();

    // Init variables
    STATE = 0;

    cloud_in = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
    //cloud_temp = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
    cloud_merged = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud


    //Threads
    visThrd = new VisThread(50, "Cloud");
    if (!visThrd->start())
    {
        delete visThrd;
        visThrd = 0;
        cout << "\nERROR!!! visThread wasn't instantiated!!\n";
        return false;
    }
    cout << "PCL visualizer Thread istantiated...\n";
    cout << endl << "Configuring done."<<endl;

    printf("Base path: %s \n \n",cloudpath.c_str());

    return true;
}

double FusionModule::getPeriod()
{
    return 0.5; //module periodicity (seconds)
}

bool FusionModule::interruptModule()
{
    closing = true;
    rpcInPort.interrupt();
    rpcObjRecPort.interrupt();
    cloudsInPort.interrupt();
    cloudsOutPort.interrupt();
    cout<<"Interrupting your module, for port cleanup"<<endl;
    return true;
}


bool FusionModule::close()
{
    cout<<"Calling close function\n";

    rpcInPort.close();
    rpcObjRecPort.close();
    cloudsInPort.close();
    cloudsOutPort.close();

    if (visThrd)
    {
        visThrd->stop();
        delete visThrd;
        visThrd =  0;
    }

    return true;
}

bool FusionModule::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/**********************************************************************/
bool FusionModule::updateModule()
{

    // STATE = init
    if (STATE == 0)
    {
        // Set template to track object
        // XXX this could also be initialized to a motionCUT BoundingBox center, or from the segmetator module.
        startTracker();

        // call obj3Drec with "seg" and the seed to obtain the first cloud
        // XXX Bilateral filter can/should be added to obj3Drec to improve cloud quality while keeping edges.
        getPointCloud(cloud_raw);

        // perform filtering on the 3D cloud to further reduce noise
        filterCloud(cloud_raw, cloud_merged);       // This cloud will be the initial model.        
        visThrd->updateCloud(cloud_merged);
        // cin << "Press any key to continue... " < endl;

        // Downsamlpe / smooth cloud
        downsampleCloud(cloud_merged, cloud_merged, resolution);

        // Update viewer
        visThrd->updateCloud(cloud_merged);
        // cin << "Press any key to continue... " < endl;
    }

    // STATE = FUSION
    if (STATE == 1)
    {
        // Get new cloud from obj3Drec from tracking coords
        getPointCloud(cloud_raw);

        // XXX Eventually, backrpoject 3D image and get all blobs which fall to some extent within the backrpojected blob.
        // XXX Proper 2D tracker-segmentation could be done simultanoeusly to improve both.
        // XXX Moreover, 3D backprojection could be used to further improve estimated blob and predict next

        // filter received cloud
        filterCloud(cloud_raw, cloud_in);
        //

        // Align and merge new cloud to model.
        //  - Use multi-scale ICP for merging.
        //  - Smooth/average result
        //  - Again, filter resulting cloud
        //  - This will be the updated model

        Eigen::Matrix4f transfMatrix;

        // XXX add an option to do merging throuhg TSDF when GPU is active. (http://docs.pointclouds.org/trunk/classpcl_1_1gpu_1_1kinfu_l_s_1_1_tsdf_volume.html)
        if(!alignPointClouds(cloud_in,cloud_merged,cloud_aligned,transfMatrix)) // Align
            return true;        // If alignment didnt work, skip merging
        *cloud_merged += *cloud_aligned;                                          // Merge
        downsampleCloud(cloud_merged,cloud_merged,resolution);                  // Smooth and downsample.

        // Update viewer
        visThrd->updateCloud(cloud_merged);
        // cin << "Press any key to continue... " < endl;

        // Estimate new object pose as the inverse of the transformation used for alignment
        // Rotate updated model to new object pose
        pcl::transformPointCloud(*cloud_merged, *cloud_merged, transfMatrix);   // ??? check if transfMatrix is correct, or needs to be the opposite.
        // Update viewer
        visThrd->updateCloud(cloud_merged);
        // cin << "Press any key to continue... " < endl;
    }
    return !closing;
}

/************************************************************************/
//                      Private Functions
/************************************************************************/

/************************************************************************/
bool FusionModule::startTracker()
{
    // read image
    ImageOf<PixelRgb> *imgIn = imgInPort.read();  // read an image
    cv::Mat imgMatIn((IplImage*) imgIn->getIplImage());

    // Get initial bounding box of object:
    printf("Click first top left and then bottom right from the object !!\n");
    cv::Rect BBox;
    cv::Point tl, br;
    bool boxOK = false;
    while (!boxOK)
    {
        printf("Click on top left!\n");
        Bottle *point1 = coordsInPort.read(true);
        tl.x =  point1->get(0).asDouble();
        tl.y =  point1->get(1).asDouble();
        printf("Point read at %d, %d!!\n", tl.x, tl.y);

        printf("Click on bottom right!\n");
        Bottle *point2 = coordsInPort.read(true);
        br.x =  point2->get(0).asDouble();
        br.y =  point2->get(1).asDouble();
        printf("Point read at %d, %d!!\n", br.x, br.y);
        BBox = cv::Rect(tl,br);
        if (BBox.area() > 20) {
            printf("valid coordinates, cropping image!\n");
            boxOK = true;
        } else {
            printf("Coordinates not valid, click again!\n");
        }
    }

    if(verbose==1){printf("Prep out mat !!\n");}

    ImageOf<PixelRgb> &templateOut  = cropOutPort.prepare();
    templateOut.resize(BBox.width, BBox.height);
    cv::Mat tempOut((IplImage*)templateOut.getIplImage(),false);
    imgMatIn(BBox).copyTo(tempOut);
    //cv::GaussianBlur(img(BBox), imOut, cv::Size(1,1), 1, 1);

    double t0 = Time::now();
    while(Time::now()-t0 < 1) {  //send the template for one second
        printf("Writing Template!\n");
        cropOutPort.write();
        Time::delay(0.1);
    }

    return true;
}

/************************************************************************/
bool FusionModule::getPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_read)
{    
    cloud_read->points.clear();
    cloud_read->clear();   // clear receiving cloud

    // Retrieves and relays the 2D coordinates of the object tracked by the tracker
    cv::Point coords2D;
    // Bottle &out  = coordsOutPort.prepare();
    if(verbose==1){printf("Getting 2D coords of tracked object!!\n");}
    Bottle *trackCoords = coordsInPort.read(true);
    coords2D.x =  trackCoords->get(0).asInt();
    coords2D.y =  trackCoords->get(1).asInt();
    if (verbose){printf("Point in 2D read from tracker: %.2i, %.2i!!\n", coords2D.x, coords2D.y);}

    // requests 3D reconstruction to objectReconst module    
    Bottle cmdOR, replyOR;
    cmdOR.clear();	replyOR.clear();
    cmdOR.addString("seg");
    cmdOR.addInt(coords2D.x);
    cmdOR.addInt(coords2D.y);
    rpcObjRecPort.write(cmdOR,replyOR);

    // read the cloud from the objectReconst output port
    Bottle *cloudBottle = cloudsInPort.read(true);
    if (cloudBottle!=NULL){
        if (verbose){	cout << "Bottle of size " << cloudBottle->size() << " read from port \n"	<<endl;}
        CloudUtils::bottle2cloud(*cloudBottle,cloud_read);
    } else{
        if (verbose){	printf("Couldn't read returned cloud \n");	}
        return -1;
    }

    cmdOR.clear();	replyOR.clear();
    cmdOR.addString("clear");
    rpcObjRecPort.write(cmdOR,replyOR);

    if (verbose){ cout << " Cloud of size " << cloud_read->points.size() << " obtained from 3D reconstruction" << endl;}

    return true;
}


/************************************************************************/
bool FusionModule::filterCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter)
{
    // Apply some filtering to clean the cloud
    // Process the cloud by removing distant points ...
    const float depth_limit = 1.0;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_orig);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, depth_limit);
    pass.filter(*cloud_filter);

     // ... and removing outliers
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror; // -- by neighbours within radius
    ror.setInputCloud(cloud_filter);
    ror.setRadiusSearch(0.05);
    ror.setMinNeighborsInRadius(5);
    ror.filter(*cloud_filter);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; // -- by statistical values
    sor.setStddevMulThresh(1.0);
    sor.setInputCloud(cloud_filter);
    sor.setMeanK(20);
    sor.filter(*cloud_filter);

    // Perform Moving least squares to smooth surfaces
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
    mls.setInputCloud (cloud_filter);
    mls.setSearchRadius (0.01);
    mls.setPolynomialFit (true);
    mls.setPolynomialOrder (2);
    mls.setUpsamplingMethod (pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius (0.005);
    mls.setUpsamplingStepSize (0.003);
    mls.process (*cloud_filter);

    if (verbose){ cout << " Size of filtered cloud " << cloud_filter->points.size() << " obtained from 3D reconstruction" << endl;}

    return true;
}



/************************************************************************/
bool FusionModule::downsampleCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ds, const double res)
{
    pcl::PointCloud<int> sampled_indices;

    pcl::UniformSampling<pcl::PointXYZRGB> us;
    us.setInputCloud(cloud_orig);
    us.setRadiusSearch(res);
    us.compute(sampled_indices);
    pcl::copyPointCloud (*cloud_orig, sampled_indices.points, *cloud_ds);
    if (verbose){std::cout << "Model total points: " << cloud_orig->size () << "; Downsampled to: " << cloud_ds->size () << std::endl;}
    return true;
}

/************************************************************************/
bool FusionModule::alignPointClouds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned, Eigen::Matrix4f& transfMat)
{
    Eigen::Matrix4f initial_T;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_IA (new pcl::PointCloud<pcl::PointXYZRGB> ());
    if (initAlignment) // Use FPFH features for initial alignment (much slower, but benefitial when clouds are initially far away)
    {
        // XXX Compute keypoints and compute features only on them to increase speed (check http://www.pointclouds.org/documentation/tutorials/correspondence_grouping.php)

        printf("Applying FPFH alignment... \n");
        if (verbose){printf("Defining features... \n");}
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_source (new pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_target (new pcl::PointCloud<pcl::FPFHSignature33>);

        // Feature computation
        if (verbose){printf("Computing features... \n");}
        computeLocalFeatures(cloud_source, features_source);
        computeLocalFeatures(cloud_target, features_target);

        // Perform initial alignment
        if (verbose){printf("Setting Initial alignment parameters \n");}
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia;
        sac_ia.setMinSampleDistance (0.05f);
        sac_ia.setMaxCorrespondenceDistance (0.02);
        sac_ia.setMaximumIterations (500);

        if (verbose){printf("Adding source cloud\n");}
        sac_ia.setInputTarget(cloud_target);
        sac_ia.setTargetFeatures (features_target);

        if (verbose){printf("Adding target cloud\n");}
        sac_ia.setInputSource(cloud_source);
        sac_ia.setSourceFeatures(features_source);

        if (verbose){printf("Aligning clouds\n");}
        sac_ia.align(*cloud_IA);

        if (!sac_ia.hasConverged())
            return -1;

        cout << "FPFH has converged:" << sac_ia.hasConverged() << " score: " << sac_ia.getFitnessScore() << endl;

        if (verbose){printf("Getting alineation matrix\n");}
        initial_T = sac_ia.getFinalTransformation();


    }

    //  Apply ICP registration
    printf("Applying ICP alignment... \n");

    if (verbose){printf("Setting ICP parameters \n");}
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaximumIterations(100);
    icp.setMaxCorrespondenceDistance (0.005);
    icp.setRANSACOutlierRejectionThreshold (0.01);  // Apply RANSAC too
    icp.setTransformationEpsilon (1e-6);

    //pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    //for (int scale = 1; scale <=3 ; scale++){

    //ICP algorithm
    if (initAlignment){
        icp.setInputSource(cloud_IA);
    }else{
        icp.setInputSource(cloud_source);
    }
    icp.setInputTarget(cloud_target);
    icp.align(*cloud_aligned);

    if (!icp.hasConverged())
        return -1;

    cout << icp.getFinalTransformation() << endl;
    transfMat = icp.getFinalTransformation() * initial_T;

    //}
}



void FusionModule::computeLocalFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features)
{
    // XXX here we could compute the keypoints first and compute features only on them to increase speed.
    pcl::search::KdTree<pcl::PointXYZRGB> ::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    computeSurfaceNormals(cloud, normals);

    printf("Computing Local Features\n");
    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud(cloud);
    fpfh_est.setInputNormals(normals);
    fpfh_est.setSearchMethod(tree);
    fpfh_est.setRadiusSearch(0.02f);
    fpfh_est.compute(*features);
}


void FusionModule::computeSurfaceNormals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    printf("Computing Surface Normals\n");
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;

    norm_est.setInputCloud(cloud);
    norm_est.setSearchMethod(tree);
    norm_est.setRadiusSearch(0.02f);
    norm_est.compute(*normals);
}



/************************************************************************/
/*
bool FusionModule::getContour(vector<cv::Point> contour)
{
    // Get segmented region from external segmentation module
    Bottle cmdSeg, replySeg;
    cmdSeg.addString("get_component_around");
    cmdSeg.addInt(click.x);
    cmdSeg.addInt(click.y);

    if (portSeg.write(cmdSeg,replySeg))
    {

        Bottle* pixelList=replySeg.get(0).asList();
        cout << "Read " << pixelList->size() << " points from segmentation algorithm" <<endl;
        cv::Mat binImg = cv::Mat(imgDispInMat.rows, imgDispInMat.cols, CV_8U, 0.0);
        for (int i=0; i<pixelList->size(); i++)
        {
            Bottle* point=pixelList->get(i).asList();
            int x = point->get(0).asDouble();
            int y = point->get(1).asDouble();
            binImg.at<uchar>(y,x) = 255;
        }

        // Get the contours of the segmented region.
        vector<vector<cv::Point> > contoursSeg;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(binImg, contoursSeg, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        contour = contoursSeg[0];
        //cout << "Contours extracted."  <<endl;
        return true;
    }
    return false;
}
*/

/*
bool FusionModule::read(ConnectionReader &connection)
{
    Bottle data; data.read(connection);
    if (data.size()>=2)
    {
        LockGuard lg(mutex);
        cv::Point click(data.get(0).asInt(),data.get(1).asInt());
        clickList.push_back(click);
    }

    return true;
}*/


/************************************************************************/
/************************************************************************/
int main(int argc, char * argv[])
{

    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    FusionModule module;
    ResourceFinder rf;
    rf.setDefaultContext("objects3DModeler");
    rf.setDefaultConfigFile("cloudsPath.ini");
    rf.setVerbose(true);
    rf.configure(argc, argv);


    cout<<"Configure module..."<<endl;
    module.configure(rf);
    cout<<"Start module..."<<endl;
    module.runModule();

    cout<<"Main returning..."<<endl;
    return 0;
}


