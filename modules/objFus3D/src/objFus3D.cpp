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
using namespace yarp::math;
using namespace iCub::YarpCloud;

/************************************************************************/
//                          PUBLIC METHODS
/************************************************************************/

//                          THRIFT RPC CALLS
/************************************************************************/


// Restart -> clear everything and go to STATE = 0
bool FusionModule::restart()
{
    // clear all clouds
    cloud_in->points.clear();          cloud_in->clear();       // Clear cloud in
    cloud_raw->points.clear();         cloud_raw->clear();      // Clear cloud raw
    cloud_aligned->points.clear();     cloud_aligned->clear();  // Clear cloud aligned
    cloud_merged->points.clear();      cloud_merged->clear();   // Clear cloud merged
    // clear visualizer
    visThrd->clearVisualizer();
    paused = false;
    tracking = false;


    // return to initial state
    STATE = 0;
    // trackerInit = false;
    return true;
}

// pause -> pause reconstruction/merging, but leave everything as it is.
bool FusionModule::track()
{
    cout << "calling startTracker" <<endl;   
    if (!startTracker())
    { 
        tracking = false;
        cout << "Failed to initialize tracking" <<endl;    
        return false; 
    }
    tracking = true;
    cout << "Tracking initialized successfully" <<endl;
    return true;         

}


// mls -> sets parameters for minimum least squares filtering
bool FusionModule::mls(double rad, double usRad, double usStep)
{
    mls_rad = rad;
    mls_usRad = usRad;
    mls_usStep = usStep;
    cout << " mls Parameters set to " <<  rad << ", " << usRad << ", " << usStep << endl;
    return true;
}
// icp -> sets parameters for iterative closest point aligning algorithm
bool FusionModule::icp(int maxIt, double maxCorr, double ranORT, double transEp)
{
    icp_maxIt = maxIt ;
    icp_maxCorr = maxCorr;
    icp_ranORT = ranORT;
    icp_transEp = transEp;
    cout << " icp Parameters set to " <<  maxIt << ", " << maxCorr << ", " <<ranORT<< ", " << transEp<< endl;
    return true;    
}
// ds -> sets parameters for downsampling
bool FusionModule::ds(double res)
{
    ds_res = res;
    return true;
}

// pause -> pause reconstruction/merging, but leave everything as it is.
bool FusionModule::save(const string &name)
{
    saving  =  true;
    filename  = name;

    cout << "Saving cloud at  " << (cloudpath+filename).c_str() << endl;
    return true;
}

// pause -> pause reconstruction/merging, but leave everything as it is.
bool FusionModule::pause()
{
    paused = !paused;
    cout << "Pausing is " << paused << endl;
    return true;
}

// setVerbose -> sets verbose ON or OFF
bool FusionModule::verb()
{
    verbose = !verbose;
    cout << "Verbose set to "<< verbose << endl;
    return true;
}

// setVerbose -> sets initAlginment ON or OFF
bool FusionModule::initAlign()
{
    initAlignment = !initAlignment;
    cout << "Initial Alignment set to "<< initAlignment << endl;
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
    pfTracker = rf.check("pfTracker", Value(false)).asBool();
    filename = rf.check("filename",Value("model")).asString().c_str();
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
    ret = ret && trackerInPort.open("/"+name+"/track:i");
    ret = ret && imgInPort.open("/"+name+"/img:i");
    ret = ret && cloudsInPort.open(("/"+name+"/clouds:i").c_str());                    // port to receive pointclouds from

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

    // Init variables
    // Defines:
    STATE = 0;
    NO_FILENUM = -1;

    // Module rpc parameters
    verbose = true ; // XXX    
    closing = false;
    saving = false;
    paused = false;
    tracking = false;
    initAlignment = false;
    tsdfON = true; //XXX

    //Algorithms parameters by default
    mls_rad = 0.02;
    mls_usRad = 0.005;
    mls_usStep = 0.003;
    icp_maxIt = 100;
    icp_maxCorr = 0.05;
    icp_ranORT = 0.05;
    icp_transEp = 1e-6;
    ds_res = rf.check("ds_resolution",Value(0.003)).asDouble();

    // Initilaize clouds
    cloud_raw = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());                  // Point cloud
    cloud_in = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());                   // Point cloud
    cloud_merged = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());               // Point cloud
    cloud_aligned = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());              // Point cloud
    cloud_normals = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());  // Point cloud
    cloud_normals->clear();
    transfMatAccum = Eigen::Matrix4f::Identity();


// XXX Check here why tsdf functions appear as not defined and give compilation errors!!!
    // it might be because they are directly defined and declared on the hpp, and hence it does not find the definition atl linking time.
    tsdf = cpu_tsdf::TSDFVolumeOctree::Ptr (new cpu_tsdf::TSDFVolumeOctree ());
    // Set initial values for tsdf
    if (tsdfON){
        tsdf->setGridSize (0.5, 0.5, 0.5); // 10m x 10m x 10m
        tsdf->setResolution (256, 256, 256); // Smallest cell size = 0.5m / 256 = about 2 mm
        tsdf->setIntegrateColor(true); // Set to true if you want the TSDF to store color
        Eigen::Affine3d tsdf_center; // Optionally offset the center
        tsdf->setGlobalTransform (tsdf_center);
        tsdf->reset (); // Initialize it to be empty
    }

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

    coordsInPort.interrupt();
    trackerInPort.interrupt();
    cloudsInPort.interrupt();
    imgInPort.interrupt();

    cloudsOutPort.interrupt();
    cropOutPort.interrupt();

    cout<<"Interrupting module."<<endl;
    return true;
}


bool FusionModule::close()
{
    cout<<"Calling close function\n";

    rpcInPort.close();
    rpcObjRecPort.close();

    cloudsInPort.close();
    coordsInPort.close();
    trackerInPort.close();
    imgInPort.close();
    cloudsOutPort.close();
    cropOutPort.close();

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
    cout << " =======================================================" << endl;
    cout << "Running on STATE: " << STATE <<endl;

    if (STATE == 0){
        // Set template to track object
        // XXX this could also be initialized to a motionCUT BoundingBox center, or from the segmetator module.
        if (tracking){
            STATE = 1;
            return true;
        }        
    }
        
    // STATE = init
    if (STATE == 1)  
    {    

        if (paused)
            return true;

        // call obj3Drec with "seg" and the seed to obtain the first cloud
        cout << "Retriveing model cloud for initialization" << endl;
        if (!getPointCloud(cloud_raw)){
            cout << "Cloud couldn't be retrieved, skipping" << endl;
            return true;
        }

        visThrd->updateCloud(cloud_raw,"raw", 0);

        // perform filtering on the 3D cloud to further reduce noise
        cout << "Filtering pointcloud" << endl;
        filterCloud(cloud_raw, cloud_merged);       // This cloud will be the initial model.        

        // Downsamlpe / smooth cloud
        cout << "Downsampling pointcloud" << endl;
        downsampleCloud(cloud_merged, cloud_merged, ds_res);


        if (tsdfON){
            Eigen::Affine3d cameraPose;
            transf2pose(transfMatAccum,cameraPose);
            tsdf->integrateCloud(*cloud_merged, *cloud_normals, cameraPose); // Integrate the cloud
        }

        // Update viewer
        visThrd->updateCloud(cloud_merged,"merged" ,1);

        STATE = 2;
    }

    // STATE = FUSION
    if (STATE == 2)
    {
        if (paused)
            return true;

        // Get new cloud from obj3Drec from tracking coords
        cout << "STATE 2: Retriveing new cloud for merging" << endl;
        if (!getPointCloud(cloud_raw)){
            cout << "Cloud couldn't be retrieved, skipping" << endl;
            return true;
        }
        cout << "STATE 2: Retrieved cloud of size: " << cloud_raw->points.size() << endl << endl;
        visThrd->updateCloud(cloud_raw,"raw",  0);

        // XXX Eventually, backrpoject 3D image and get all blobs which fall to some extent within the backrpojected blob.
        // XXX Proper 2D tracker-segmentation could be done simultanoeusly to improve both.
        // XXX Moreover, 3D backprojection could be used to further improve estimated blob and predict next

        // filter received cloud
        cout << "STATE 2: Filtering pointcloud" << endl;
        filterCloud(cloud_raw, cloud_in);       // This cloud will be the initial model. 
        cout << "STATE 2: Filtered to cloud of size: " << cloud_in->points.size() << endl << endl;

        // Downsamlpe / smooth cloud
        cout << "STATE 2: Downsampling pointcloud" << endl;
        downsampleCloud(cloud_in,cloud_in,ds_res );      // Smooth and downsample.
        cout << "STATE 2: Downsampled cloud of size: " << cloud_in->points.size() << endl <<endl;



        cout << "STATE 2: Aligning pointcloud" << endl;
        cout << "-- Source cloud of size: " << cloud_in->points.size() << endl;
        cout << "-- Merged model cloud has size: " << cloud_merged->points.size() << endl;
        Eigen::Matrix4f transfMatrix;
        if(!alignPointClouds(cloud_in,cloud_merged,cloud_aligned,transfMatrix)){ // Align
            cout << "xxx Pointcloud not aligned" << endl;
            return true;        // If alignment didnt converge, skip merging
        }
        cout << "== Pointcloud aligned to cloud of size " << cloud_aligned->points.size()  << endl;
        transfMatAccum = transfMatrix * transfMatAccum;
        //Eigen::Affine3d cameraPose = eigenTransform(transfMatAccum);
        Eigen::Affine3d cameraPose;
        transf2pose(transfMatAccum,cameraPose);

        if (!tsdfON){
            cout << "STATE 2: Merging pointcloud" << endl;
            *cloud_merged += *cloud_aligned;                                          // Merge
            cout << "== Pointclouds Merged to cloud of size " << cloud_merged->points.size()  << endl;

            cout << "STATE 2: Downsampling merged pointcloud" << endl;
            downsampleCloud(cloud_merged,cloud_merged,ds_res);    // Smooth and downsample.
            cout << "== Downsampled to cloud of size " << endl;

            // Estimate new object pose as the inverse of the transformation used for alignment
            // Rotate updated model to new object pose
            cout << "STATE 2: Rotating merged pointcloud to new pose" << endl;
            Eigen::Matrix4f targetToSource = transfMatrix.inverse();            //
            pcl::transformPointCloud(*cloud_merged, *cloud_merged, targetToSource);   // targetToSource is correct, checked

            cout << "== Pointcloud transformed to model pose " << cloud_merged->points.size()  << endl;
        } else {
            // XXX add an option to do merging throuhg TSDF when GPU is active. (http://docs.pointclouds.org/trunk/classpcl_1_1gpu_1_1kinfu_l_s_1_1_tsdf_volume.html)
            // Set initial pose to 4x4 identity, and multiply each T (from ICP) by all the previous ones (iteratively) to get the pose at each cloud wrt the first one.
            // Probably we dont need to invert if we keep assume that the world is fixed at the pose of the first cloud.
            cout << "Performinf TSDF-based integration" << endl;
            tsdf->integrateCloud(*cloud_in, *cloud_normals, cameraPose); // Integrate the cloud

            // Now what do you want to do with it?
            float distance; pcl::PointXYZ query_point (1.0, 2.0, -1.0);
            tsdf->getFxn (query_point, distance); // distance is normalized by the truncation limit -- goes from -1 to 1
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_raytraced = tsdf->renderColoredView(cameraPose); // Render it
            cloud_merged->clear();
            pcl::copyPointCloud(*cloud_raytraced, *cloud_merged);
        }

        // Update viewer
        //visThrd->updateCloud(cloud_merged,"merged", 1);

        if (saving){
            CloudUtils::savePointsPly(cloud_merged, cloudpath, filename, NO_FILENUM);
            saving = false;
        }
    }
    return !closing;
}

/************************************************************************/
//                      Private Functions
/************************************************************************/

/************************************************************************/
bool FusionModule::startTracker()
{
    if (pfTracker)
    {

        printf("Initalizing tracker \n");
        // read image
        ImageOf<PixelRgb> *imgIn = imgInPort.read();  // read an image
        cv::Mat imgMatIn = cv::cvarrToMat((IplImage*)imgIn->getIplImage());

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
        cv::Mat tempOut=cv::cvarrToMat((IplImage*)templateOut.getIplImage());
        imgMatIn(BBox).copyTo(tempOut);
        //cv::GaussianBlur(img(BBox), imOut, cv::Size(1,1), 1, 1);

        double t0 = Time::now();
        while(Time::now()-t0 < 1) {  //send the template for one second
            printf("Writing Template!\n");
            cropOutPort.write();
            Time::delay(0.1);
        }
    }

    return true;
}

/************************************************************************/
bool FusionModule::getPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_read)
{    
    cloud_read->points.clear(); // clear receiving cloud
    cloud_read->clear();

    // Retrieves and relays the 2D coordinates of the object tracked by the tracker
    cv::Point coords2D;
    Bottle *trackCoords = trackerInPort.read(true);
    coords2D.x =  trackCoords->get(0).asInt();
    coords2D.y =  trackCoords->get(1).asInt();
    if (verbose){printf("Point in 2D read from tracker: %.2i, %.2i!!\n", coords2D.x, coords2D.y);}

    // requests 3D reconstruction to objectReconst module    
    Bottle cmdOR, replyOR;
    cmdOR.clear();	replyOR.clear();
    cmdOR.addString("setFormat");               // Set command to ask obj3Drec not to save files
    cmdOR.addString("none");
    rpcObjRecPort.write(cmdOR,replyOR);

    cmdOR.clear();	replyOR.clear();
    cmdOR.addString("clear");                   // Send command to clear disparity visualizer.
    rpcObjRecPort.write(cmdOR,replyOR);

    cmdOR.clear();	replyOR.clear();
    cmdOR.addString("seg");                     // Set command to ask obj3Drec get cloud contour from segmentation algorithm.
    cmdOR.addInt(coords2D.x);
    cmdOR.addInt(coords2D.y);
    rpcObjRecPort.write(cmdOR,replyOR);

    // read the cloud from the objectReconst output port
    Bottle *cloudBottle = cloudsInPort.read(true);
    if (cloudBottle!=NULL){
        if (verbose){	cout << "Received bottle of size " << cloudBottle->size() << endl;	}
        if (cloudBottle->size()==0) {
            if (verbose){	printf("Received empty bottle \n");	}
            return false;
        }
        if (verbose){	cout << "Bottle of size " << cloudBottle->size() << " read from port \n"	<<endl;}
        CloudUtils::bottle2cloud(*cloudBottle,cloud_read);
    } else{
        if (verbose){	printf("Couldn't read returned cloud \n");	}
        return false;
    }

    return true;
}


/************************************************************************/
bool FusionModule::filterCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filter)
{
    // Apply some filtering to clean the cloud
    // First of all, remove possible NaNs
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_orig, *cloud_orig, indices);

    // Process the cloud by removing distant points ...
    cout << "--Original cloud to filter of size " << cloud_orig->points.size() << "." << endl;
    const float depth_limit = 1.0;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_orig);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, depth_limit);
    pass.filter(*cloud_filter);
    cout << "--Size after passthrough: " << cloud_filter->points.size() << "." << endl;

     // ... and removing outliers

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror; // -- by neighbours within radius
    ror.setInputCloud(cloud_filter);
    ror.setRadiusSearch(0.05);
    ror.setMinNeighborsInRadius(5);
    ror.filter(*cloud_filter);
    cout << "--Size after rad out rem: " << cloud_filter->points.size() << "." << endl;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; // -- by statistical values
    sor.setStddevMulThresh(3.0);
    sor.setInputCloud(cloud_filter);
    sor.setMeanK(10);
    sor.filter(*cloud_filter);
    cout << "--Size after Stat outrem: " << cloud_filter->points.size() << "." << endl;

    // Perform Moving least squares to smooth surfaces
    // Init object (second point type is for the normals, even if unused)

    // XXX Dirty hack to do it with XYZ clouds. Check better ways (less conversions) to change XYZ <-> XYZRGB
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoColor(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoColorFilter(new pcl::PointCloud<pcl::PointXYZ>); 
    copyPointCloud(*cloud_filter, *cloudNoColor); 

    //pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    //pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
    //    mls.setInputCloud (cloud_filter);
    mls.setInputCloud(cloudNoColor);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(mls_rad);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);
    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(mls_usRad);
    mls.setUpsamplingStepSize(mls_usStep);
    mls.process(*cloudNoColorFilter);
    copyPointCloud(*cloudNoColorFilter, *cloud_filter); 
    //    mls.process (*cloud_filter);
    cout << "--Size after Mov leastsq: " << cloud_filter->points.size() << "." << endl;

//    if (verbose){ cout << " Cloud of size " << cloud_filter->points.size() << "after filtering." << endl;}

    return true;
}



/************************************************************************/
bool FusionModule::downsampleCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ds, double res)
{
    pcl::PointCloud<int> sampled_indices;
    //if (verbose){cout << "Model total points: " << cloud_orig->size () << endl;}
    cout << "== Size before downsampling: " << cloud_orig->size () << endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fil(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud_orig);
    vg.setLeafSize(res, res, res);
    vg.filter (*cloud_fil);
    cloud_ds->points.clear();
    cloud_ds->clear();
    copyPointCloud(*cloud_fil, *cloud_ds);

    //if (verbose){cout << " Downsampled to: " << cloud_ds->size () << endl;}
    cout << "== Size after  downsampling: " << cloud_ds->size () << endl;
    return true;
}

/************************************************************************/
bool FusionModule::alignPointClouds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_align, Eigen::Matrix4f& transfMat)
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
            return false;

        cout << "FPFH has converged:" << sac_ia.hasConverged() << " score: " << sac_ia.getFitnessScore() << endl;

        if (verbose){printf("Getting alineation matrix\n");}
        initial_T = sac_ia.getFinalTransformation();


    }

    //  Apply ICP registration
    printf("\n Starting ICP alignment procedure... \n");

    if (verbose){printf("Setting ICP parameters \n");}
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setMaximumIterations(icp_maxIt);
    icp.setMaxCorrespondenceDistance(icp_maxCorr);
    icp.setRANSACOutlierRejectionThreshold(icp_ranORT);  // Apply RANSAC too
    icp.setTransformationEpsilon(icp_transEp);

/*
    // Do multiscale ICP to get inital good guess and increase speed and resolution
    double icpRes = ds_res*4;   // Start with 1 cm resolution
    bool init_it = false;
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity ();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src_ds (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt_ds (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_align_ds (new pcl::PointCloud<pcl::PointXYZRGB> ());
    for (int scale = 3; scale >=1 ; scale--){
        icp.setMaxCorrespondenceDistance(icp_maxCorr*scale);
        downsampleCloud(cloud_source, cloud_src_ds, icpRes );    // Smooth and downsample.
        downsampleCloud(cloud_target, cloud_tgt_ds, icpRes );    // Smooth and downsample.

        icp.setInputSource(cloud_src_ds);
        icp.setInputTarget(cloud_tgt_ds);
        cout << "Aligning at resolution " << icpRes << endl;

        if (init_it){
            icp.align(*cloud_align_ds, guess);
        }else{
            icp.align(*cloud_align_ds);}

        if (!icp.hasConverged()){
            printf("DS Clouds not aligned \n");
            return false;
        }
        cout << "Clouds at res "<< icpRes << " aligned!" <<endl;

        guess = icp.getFinalTransformation();
        icpRes = icpRes/2;
        init_it = true;
    }

    // Do final ICP based on guess
    printf("Setting source cloud as input... \n");
    icp.setInputSource(cloud_source);
    icp.setInputTarget(cloud_target);
    printf("Aligning... \n");
    icp.align(*cloud_align, guess);

    if (!icp.hasConverged()){
        printf("Clouds not aligned \n");
        return false;
    }
    printf("Clouds Aligned! \n");

    transfMat = icp.getFinalTransformation();
    cout << transfMat << endl;
*/
    //ICP algorithm
    // Carefully clean NaNs, as otherwise they make the alignment crash horribly.
    std::vector <int> nanInd;

    removeNaNs(cloud_source,cloud_source, nanInd);
    cout << "Found " << nanInd.size() << " NaNs on source cloud." <<endl;
    removeNaNs(cloud_target,cloud_target, nanInd);
    cout << "Found " << nanInd.size() << " NaNs on target cloud." <<endl;

    if (initAlignment){
        printf("Setting initAlgined cloud as input... \n");
        icp.setInputSource(cloud_IA);
    }else{
        printf("Setting source cloud as input... \n");
        icp.setInputSource(cloud_source);
    }
    icp.setInputTarget(cloud_target);
    printf("Aligning... \n");
    icp.align(*cloud_align);
    printf("Alignment attept done\n");
    if (!icp.hasConverged()){
        printf("Clouds not aligned \n");
        return false;
    }
    printf("Clouds Aligned! \n");

    if (initAlignment){
        transfMat = icp.getFinalTransformation() * initial_T;
    }else{
        transfMat = icp.getFinalTransformation();
    }


    // Check if the rotation in any angle is too big > 30, assuming the object is rotated slowly, it would mean the aligning is incorrect.
    Matrix transfMatYARP = CloudUtils::eigMat2yarpMat(transfMat);
    Vector rotVec = dcm2rpy(transfMatYARP);  // from rot Matrix to roll pitch yaw
    Vector rotVecDeg = rotVec*(180.0/M_PI);


    printf("Alignment rotated new cloud: \n %s  degrees \n",rotVecDeg.toString().c_str());
    for (int d=0;d<rotVecDeg.size();d++){
        if (rotVecDeg[d]>30.0 ){//M_PI/6.0){
            cout << "Alignment required too much rotation. Skipping" << endl;
            return false;
        }
    }
}


int FusionModule::removeNaNs(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_noNaN, vector <int> nanInds)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudC (new pcl::PointCloud<pcl::PointXYZRGB> ()); // Copy input cloud to allow having input and output the same cloud
    pcl::copyPointCloud(*cloud, *cloudC);
    cloud_noNaN->points.clear();
    cloud_noNaN->clear();
    nanInds.clear();

    int sizeC;
    sizeC = cloud->size();

    int nanCount = 0;
    for (int i = 0 ;i < sizeC; i++){
        if (!pcl_isfinite (cloudC->points[i].x) || !pcl_isfinite (cloudC->points[i].y) || !pcl_isfinite (cloudC->points[i].z)){
            nanCount ++;
            nanInds.push_back(i);
        }else {
            cloud_noNaN->push_back(cloudC->points[i]);
        }
    }
    return nanCount;
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

void FusionModule::transf2pose (const Eigen::Matrix4f trans, Eigen::Affine3d pose){

    pose(3,0) = 0;pose(3,1) = 0; pose(3,2) = 0; pose(3,3) = 1; // Make last row 0 0 0 1
    for (int y = 0; y < 3; y++)
    {
        for (int x = 0; x < 4; x++)     {
            pose(y,x) = static_cast<double>(trans(y,x));
        }
    }

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


