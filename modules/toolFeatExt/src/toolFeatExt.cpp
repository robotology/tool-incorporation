/*
 * TOOL 3D FEATURE EXTRACTOR
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

#include <toolFeatExt.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::YarpCloud;

// This module is mainly concerned with getting OMS-EGI 3D features from a point cloud. Therefore, it can:
// - load cloud from model (loadModel) and orient it with matrix (setPose-setCanonicalPose)
// - read a cloud (on updateModule) and set it as the model.
// - compute features on command and send them out (getFeat), and set paraemters
// - Optionally,
//      - compute automatically all features for all available models. (getAllToolFeats).
//      - get multiple samples from a single tool-pose with slight variations (getSamples).



/************************* RF overwrites ********************************/
/************************************************************************/
bool ToolFeatExt::configure(ResourceFinder &rf)
{
    // add and initialize the port to send out the features via thrift.
    string name = rf.check("name",Value("toolFeatExt")).asString().c_str();
    string robot = rf.check("robot",Value("icub")).asString().c_str();
    string cloudpath_file = rf.check("from",Value("cloudsPath.ini")).asString().c_str();
    rf.findFile(cloudpath_file.c_str());

    ResourceFinder cloudsRF;
    cloudsRF.setContext("tool3DModeler");
    cloudsRF.setDefaultConfigFile(cloudpath_file.c_str());
    cloudsRF.configure(0,NULL);

    // Set the path that contains previously saved pointclouds
    if (rf.check("clouds_path")){
        cloudpath = rf.find("clouds_path").asString().c_str();
    }else{
        string defPathFrom = "/share/ICUBcontrib/contexts/objects3DModeler/sampleClouds/";
        string localModelsPath    = rf.check("local_path")?rf.find("local_path").asString().c_str():defPathFrom;     //cloudsRF.find("clouds_path").asString();
        string icubContribEnvPath = yarp::os::getenv("ICUBcontrib_DIR");
        cloudpath  = icubContribEnvPath + localModelsPath;
    }
    cout << "Cloud files PATH: " << cloudpath << endl;


    verbose = rf.check("verbose",Value(true)).asBool();
    maxDepth = rf.check("maxDepth",Value(2)).asInt();
    binsPerDim = rf.check("depth",Value(2)).asInt();

    loadModelsFromFile(rf);


    //open ports
    bool ret = true;
    ret = ret && cloudsInPort.open(("/"+name+"/clouds:i").c_str());    // port to receive pointclouds from
    ret =  ret && feat3DoutPort.open("/"+name+"/feats3D:o");			// Port which outputs the vector containing all the extracted features

    // XXX eventually might be ok to remove this port, and all the functions to send out clouds, as that is now handled by obj3Dexp
    ret = ret && cloudsOutPort.open(("/"+name+"/clouds:o").c_str());    // port to receive pointclouds from
    if (!ret){
        printf("Problems opening ports\n");
        return false;
    }

    // open rpc ports
    bool retRPC = true;
    retRPC =  retRPC && rpcInPort.open("/"+name+"/rpc:i");
    if (!retRPC){
        printf("Problems opening ports\n");
        return false;
    }
    attach(rpcInPort);

    /* Module rpc parameters */
    closing = false;

    /*Init variables*/
    cloudLoaded = false;
    cloudTransformed = false;
    cloudname = "cloud_merged.ply";
    cloud_orig = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    rotMat = eye(4,4);

    cout << endl << "Configuring done."<<endl;

    return true;
}


double ToolFeatExt::getPeriod()
{
    return 0.2; //module periodicity (seconds)
}

bool ToolFeatExt::updateModule()
{
    // read the cloud as bottle
    Bottle *cloudBottle=cloudsInPort.read(false);
    if (cloudBottle!=NULL){
        cout<< "Received Cloud Bottle of size " << cloudBottle->size() << endl;
        cloud->points.clear();
        cloud->clear();
        CloudUtils::bottle2cloud(*cloudBottle,cloud);
        cout<< "Cloud of size: " << cloud->points.size() << endl;
        sendCloud(cloud);
        cloudTransformed = true;
        cloudLoaded =true;
    }
    return !closing;
}

bool ToolFeatExt::interruptModule()
{
    closing = true;
    rpcInPort.interrupt();
    feat3DoutPort.interrupt();
    cout<<"Interrupting your module, for port cleanup"<<endl;
    return true;
}


bool ToolFeatExt::close()
{
    cout<<"Calling close function\n";
    rpcInPort.close();
    feat3DoutPort.close();
    return true;
}

/**************************** THRIFT CONTROL*********************************/
bool ToolFeatExt::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

/**********************************************************
                    PUBLIC METHODS
/**********************************************************/

// RCP Accesible via trhift.

/**********************************************************/
bool ToolFeatExt::getFeats()
{   // computes 3D oriented -normalized voxel wise EGI - tool featues.
    int ok = computeOMSEGI();
    if (ok>=0) {        
        return true;
    } else {
        fprintf(stdout,"3D Features not computed correctly. \n");
        return false;
    }
}


/**********************************************************/
bool ToolFeatExt::getSamples(const int n, const double deg)
{
    if (!cloudLoaded){
        if (!loadToolModel())
        {
            fprintf(stdout,"Couldn't load cloud");
            return -1;
        }
    }

    float maxVar = 5;      //Define the maximum variation, on degrees, wrt the given orientation angle

    Rand randG; // YARP random generator
    Vector degVarVec = randG.vector(n); // Generate a vector of slight variations to the main orientation angle
    for (int s=0;s<n;++s)
    {
        if(!setCanonicalPose(deg + degVarVec[s]*maxVar*2-maxVar))   // Transform model to a close but different position
        {
            fprintf(stdout,"Error transforming frame to canonical position . \n");
        }

        if (!computeOMSEGI()) {
            fprintf(stdout,"Error computing features. \n");
        }
    }
    fprintf(stdout,"Features extracted from tool pose samples. \n");\
    return true;
}

/**********************************************************/
bool ToolFeatExt::getAllToolFeats(const string& setup)
{
    int iniTool = 0;
    if (setup ==  "sim")
        iniTool = 1;    // in sim model 0 is the cube, so skip it

    for (int toolI = iniTool ; toolI <models.size(); toolI++)
    {
        string meshName = models[toolI].get(2).asString();
        string::size_type idx;
        idx = meshName.rfind('.');
        string cloudName = meshName.substr(0,idx);  //remove format
        cloudName = setup + "/"+ cloudName;
        cout << "cloud model: " << cloudName << endl;

        if (!loadModel(cloudName)){
            cout << "Couldn't load model for tool " << cloudName << endl;
            return false;
        }

        if(setup=="real"){
            computeOMSEGI();         // Get first the canonical position features
            // Transform tools to orientations 90, 0  and -90 , in that order.  And for each, displacements -1, 0
            for ( int ori = 90; ori > -100; ori = ori - 90){            // This is a loop for {90, 0, -90}
                for (int disp=-1 ; disp<1 ; disp ++){                   // This is a loop for {-1,0}
                    setCanonicalPose(ori, disp);
                    //getSamples(const int n, const double deg)
                    computeOMSEGI();                                     // Compute features for each desired pose
                }
            }

        }else if (setup=="sim"){
            computeOMSEGI();         // Get first the canonical position features
            // Transform tools to orientations -90, 0  and 90 , in that order. And for each, displacements -2, 0, 2
            for ( int ori = -90; ori < 100; ori +=  90){                  // This is a loop for {-90, 0, 90}
                for (int disp=-2 ; disp<3 ; disp += 2){                   // This is a loop for {-2,0, 2}
                    setCanonicalPose(ori, disp);
                    computeOMSEGI();                                       // Compute features for each desired pose
                }
            }

        }else {
            cout << "Please select 'real' or 'sim' tools" << endl;
            return false;
        }
    }

    return true;
}

/**********************************************************/
bool ToolFeatExt::setPose(const Matrix& toolPose)
{   // Rotates the tool model according to the given rotation matrix to extract pose dependent features.
    int ok =  transform2pose(toolPose);
    if (ok>=0) {
        fprintf(stdout,"Cloud rotated correctly \n");
        return true;
    } else {
        fprintf(stdout,"Cloud could not be rotated correctly. \n");
        return false;
    }
}

/**********************************************************/
bool ToolFeatExt::setCanonicalPose(const double deg, const int disp)
{   // Rotates the tool model 'deg' degrees around the hand -Y axis
    // Positive angles turn the end effector "inwards" wrt the iCub, while negative ones rotate it "outwards" (for tool on the right hand).
    float rad = deg*M_PI/180.0; // converse deg into rads

    Vector oy(4);   // define the rotation over the Y axis (that is the one that we consider for tool orientation -left,front,right -
    oy[0]=0.0; oy[1]=-1.0; oy[2]=0.0; oy[3]= rad; // XXX This is the RIGHT WAY, becasue the tool is along the -Y axis!!
    //oy[0]=0.0; oy[1]=1.0; oy[2]=0.0; oy[3]= rad;

    Matrix toolPose = axis2dcm(oy); // from axis/angle to rotation matrix notation
    toolPose(1,3) = -disp /100.0;   // This accounts for the traslation of 'disp' in the -Y axis in the hand coord system along the extended thumb).

    int ok =  transform2pose(toolPose);
    if (ok>=0) {
        fprintf(stdout,"Cloud rotated correctly \n");
        return true;
    } else {
        fprintf(stdout,"Cloud could not be rotated correctly. \n");
        return false;
    }
}

/**********************************************************/
bool ToolFeatExt::setBinNum(const int binsN)
{
    binsPerDim = binsN;
    return true;
}

/**********************************************************/
bool ToolFeatExt::setDepth(const int depthN)
{
    maxDepth = depthN;
    return true;
}

/**********************************************************/
bool ToolFeatExt::loadModel(const string& name)
{   //Changes the name of the .ply file to display. Default 'cloud_merged.ply'"
    cloudname = name;
    return loadToolModel();
}

/**********************************************************/
bool ToolFeatExt::setVerbose(const string& verb)
{
    if (verb == "ON"){
        verbose = true;
        fprintf(stdout,"Verbose is : %s\n", verb.c_str());
        return true;
    } else if (verb == "OFF"){
        verbose = false;
        fprintf(stdout,"Verbose is : %s\n", verb.c_str());
        return true;
    }
    fprintf(stdout,"Verbose can only be set to ON or OFF. \n");
    return false;
}

/**********************************************************/
bool ToolFeatExt::quit()
{
    closing = true;
    return true;
}


/**********************************************************
                    PRIVATE METHODS
/**********************************************************/


/************************************************************************/
bool ToolFeatExt::loadModelsFromFile(ResourceFinder &rf)
{
    // Read the Models configurations
    Bottle temp;
    string modelName = "obj";

    cout << "Loading models to buffer" << endl;
    bool noMoreModels = false;
    int n =0;
    while (!noMoreModels){      // read until there are no more objects.
        stringstream s;
        s.str("");
        s << modelName << n;
        string objNameNum = s.str();
        temp = rf.findGroup("objects").findGroup(objNameNum);

        if(!temp.isNull()) {
            cout << "model " << n << ", id: " << objNameNum;
            cout << ", model: " << temp.get(2).asString() << endl;
            models.push_back(temp);
            temp.clear();
            n++;
        }else {
            noMoreModels = true;
        }
    }
    int numberObjs = n;
    cout << "Loaded " << numberObjs << " objects. " << endl;
    return true;
}

/************************************************************************/
bool ToolFeatExt::loadToolModel()
{
    cout << endl <<" +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ " << endl;
    cout << "Loading tool model cloud from file" << cloudpath.c_str() << cloudname.c_str() << endl;

    // (Re-) initialize cloud transformations
    cloud_orig->points.clear();
    cloud_orig->clear();
    cloudTransformed = false;
    rotMat = eye(4,4);

    if (CloudUtils::loadCloud(cloudpath, cloudname, cloud_orig))
    {
        cout << "Loaded tool model of size: " << cloud_orig->points.size () << endl;
        cloudLoaded = true;

        sendCloud(cloud_orig);
        return true;
    }
    return false;
}

/************************************************************************/
// toolPose is represented by the transformation matrix of the explored object wrt the canonical position.
// It shall be computed by a previous fucntion/module as the transformation matrix obtained when
//   registratering the single view to the canonical mode.
bool ToolFeatExt::transform2pose(const Matrix &toolPose)
{   // transform2pose() rotates the full canonical 3D model to the pose on which it has been found to be
    //    in order to obtain the pose - dependent 3D features.

    //Load cloud if ain't yet loaded
    if (!cloudLoaded){
        if (!loadToolModel())
        {
            printf("Couldn't load cloud");
            return false;
        }
    }

    if (verbose){	cout << "Transform with Matrix " << endl << toolPose.toString() <<endl;	}

    // format YARP Matrix yo Eigen Matrix
    Eigen::Matrix4f TM = CloudUtils::yarpMat2eigMat(toolPose);

    // Execute the transformation
    pcl::transformPointCloud(*cloud_orig , *cloud, TM);

    if (verbose){	printf("Transformation done \n");	}

    rotMat = toolPose;
    cloudTransformed = true;

    sendCloud(cloud);

    return true;
}

/************************************************************************/
int ToolFeatExt::computeOMSEGI()
    {
    cout << endl <<" +++++++++++++++++++++++++++++ FEATURE EXTRACTION ++++++++++++++++++++++++++++++++++++ " << endl;

    if (!cloudLoaded){
        if (!loadToolModel())
        {
            printf("Couldn't load cloud");
            return -1;
        }
    }
    cout << "Extracting features from: " << cloudname.c_str() << endl;

    if (!cloudTransformed){
        transform2pose();
    }
    
    cout << "Computing Features to maximum depth = " << maxDepth <<  "." << endl;
    
    /* ===========================================================================*/
    // Get fixed bounding box to avoid octree automatically figure it out (what might change with resolution)
    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();

    pcl::PointXYZRGB min_point_AABB;
    pcl::PointXYZRGB max_point_AABB;
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    max_point_AABB.y = 0; // Limit the bounding box to the bottom of the hand, so only the "usable" part of the tool gets represented.
    double BBlengthX = fabs(max_point_AABB.x - min_point_AABB.x);
    double BBlengthY = fabs(max_point_AABB.y - min_point_AABB.y);
    double BBlengthZ = fabs(max_point_AABB.z - min_point_AABB.z);

    double maxSize = max(max(BBlengthX,BBlengthY), BBlengthZ);
    cout << "Lenght of the larger size is = " << maxSize <<  "." << endl;

    /* =========================================================================== */
    // Divide the bounding box in subregions using octree, and compute the normal histogram (EGI) in each of those subregions at different levels.

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Create instances of necessary classes

    // octree
    float minVoxSize = (maxSize+0.01)/2; // Min voxel size (max resolution) 1 cm
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(minVoxSize);

    // Normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Cloud of Normals from whole pointcloud
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());

    // voxelCloud and voxelCloudNormals to contain the poitns on each voxel
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::Normal>::Ptr voxelCloudNormals(new pcl::PointCloud<pcl::Normal> ());

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Initialize octree
    //create octree structure
    octree.setInputCloud(cloud);
    //update bounding box automatically
    octree.defineBoundingBox(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z,
                             max_point_AABB.x, max_point_AABB.y, max_point_AABB.z);
    //add points in the tree
    octree.addPointsFromInputCloud();


    // Compute all normals from the orignial cloud, and THEN subdivide into voxels (to avoid voxel boundary problems arising when computing normals voxel-wise)
    // Clear output cloud
    cloud_normals->points.clear();

    // Pass the input cloud to the normal estimation class
    ne.setInputCloud (cloud);

    // Use all neighbors in a sphere of radius of size of the voxel length
    ne.setRadiusSearch (0.05);//(minVoxSize/2);

    // Compute the normals
    ne.compute (*cloud_normals);


    // Initialize histogram variables
    float rangeMin = -1;
    float rangeMax = 1;
    float rangeValid = rangeMax - rangeMin;
    float sizeBin = rangeValid/binsPerDim;

    std::vector < std::vector < double > > featureVectorOccVox;     // Vector containing histograms for occupied voxels
    ToolFeat3DwithOrient featureVectorAllVox;     // Vector containing histograms for all voxels, also the empty ones
    featureVectorAllVox.toolname = cloudname;
    featureVectorAllVox.orientation = rotMat;
    //std::vector < std::vector < float > > featureVectorAllVox;     // Vector containing histograms for all voxels, also the empty ones
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Find the minimum resolution octree (depth =1) to start increase it from there

    double cBB_min_x, cBB_min_y, cBB_min_z, cBB_max_x, cBB_max_y, cBB_max_z;
    octree.getBoundingBox(cBB_min_x, cBB_min_y, cBB_min_z, cBB_max_x, cBB_max_y, cBB_max_z);
    double cBBsize = fabs(cBB_max_x - cBB_min_x);           // size of the cubic bounding box encompasing the whole pointcloud

    if(verbose){
        cout << "AA BB: (" << min_point_AABB.x << "," << min_point_AABB.y << "," << min_point_AABB.z << "), (" << max_point_AABB.x << "," <<max_point_AABB.y << "," << max_point_AABB.z << ")" << endl;
        cout << "Octree BB: (" << cBB_min_x << "," << cBB_min_y << "," <<cBB_min_z << "), (" << cBB_max_x << "," <<cBB_max_y << "," <<cBB_max_z << ")" << endl;
    }

    // re-initialize tree with new resolution
    minVoxSize = cBBsize/2;     //Update to minimum resolution
    octree.deleteTree();
    octree.setResolution(minVoxSize);
    octree.defineBoundingBox(cBB_min_x, cBB_min_y, cBB_min_z,
                             cBB_max_x, cBB_max_y, cBB_max_z);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Perform feature extraction iteratively on increasingly smaller voxels up to maxDepth level.
    for (int depth = 0; depth<=maxDepth; ++depth)
    {
        if(verbose){cout << endl << "====================== Depth:" << depth << " ========================== " << endl;}


        if (depth ==0 ){ // At depth 0, compute the normal histogram for the whole object.
            /* =================================================================================*/
            // Compute the EGI (sphere normal histogram). Normalize and add it to the feature vector.

            float cloudHist[binsPerDim][binsPerDim][binsPerDim];     // Histogram 3D matrix per voxel

            // Initialize histogram for new voxel
            for (int i = 0; i < binsPerDim; ++i)
                for (int j = 0; j < binsPerDim; ++j)
                    for (int k = 0; k < binsPerDim; ++k)
                       cloudHist[i][j][k] = 0.0f;

            // Compute normal histogram looping through normals in voxel.
            unsigned int okNormals = 0;
            unsigned int nanCount= 0;
            for (size_t i = 0; i < cloud_normals->points.size (); ++i)
            {
                int xbin, ybin, zbin;
                float xn, yn, zn;

                if ((pcl::isFinite(cloud_normals->points[i]))){
                    xn = cloud_normals->points[i].normal_x;
                    yn = cloud_normals->points[i].normal_y;
                    zn = cloud_normals->points[i].normal_z;
                    //std::cout << "Normal  (" << xn << "," << yn << "," << zn << ")" << std::endl;

                    xbin = floor((xn - rangeMin) /sizeBin); // floor because 0-indexed array
                    ybin = floor((yn - rangeMin) /sizeBin);
                    zbin = floor((zn - rangeMin) /sizeBin);
                    //std::cout << "lies on bin (" << xbin << "," << ybin << "," << zbin << ")" << std::endl;

                    cloudHist[xbin][ybin][zbin]= cloudHist[xbin][ybin][zbin] + 1.0f;
                    okNormals ++;
                }else{
                    nanCount++;
                    //std::cout << "Normal is NaN." << std::endl;

                }
            }   //- close loop over normal points in voxel
            std::cout << "." ;
            std::cout << "Cloud has " << okNormals << " valid points and " << nanCount << " NaNs." << std::endl;
            std::vector< double> featVecHist;                                   // Vector containing a serialized version of the 3D histogram
            //Normalize by the total number of points in voxel so that each histogram has sum 1, independent of the number of points in the voxel.
            for (int i = 0; i < binsPerDim; ++i){
                for (int j = 0; j < binsPerDim; ++j){
                    for (int k = 0; k < binsPerDim; ++k){
                        if (okNormals){              // Avoid dividing by 0.
                            cloudHist[i][j][k] = cloudHist[i][j][k] / (float) okNormals;}

                        // Push into feature vector per histogram
                        featVecHist.push_back(cloudHist[i][j][k]);
                    }
                }
            }

            featureVectorAllVox.toolFeats.push_back(featVecHist);       // copy histogram to sendout data
        }
        else
        {
            featureVectorOccVox.clear();                                // Clear the vector which saves all populated histograms at depth level

            int voxPerSide = pow(2,depth);
            bool voxelOccupancy[voxPerSide][voxPerSide][voxPerSide];    // Keeps record of the position of the occupied voxels at each depth.
            for (int i = 0; i < voxPerSide; ++i)                        // initialize all positions to false
                for (int j = 0; j < voxPerSide; ++j)
                    for (int k = 0; k < voxPerSide; ++k)
                       voxelOccupancy[i][j][k] = false;

            // Instantiate normal histogram at actual depth
            int numLeaves = octree.getLeafCount();                              // Get the number of occupied voxels (leaves) at actual depth
            int histSize = pow(binsPerDim,3);
            float voxelHist[numLeaves][binsPerDim][binsPerDim][binsPerDim];     // Histogram 3D matrix per occupied voxel
            float voxelsHistMat[voxPerSide][voxPerSide][voxPerSide][histSize]; // Matrix containing the normal histogram at each voxel at actual depth.

            //std::vector< double> featVecHist;                                   // Vector containing a serialized version of the 3D histogram

            if(verbose){
                cout << "BB divided into " << voxPerSide << " voxels per side of size "<< minVoxSize << ", " << numLeaves << " occupied." << endl;
                cout << "Looping through the  " << numLeaves << " occupied voxels." << endl;
            }

            // Extracts all the points at leaf level from the octree
            int octreePoints = 0;
            int voxelI = 0;
            int nanCount = 0;
            pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::LeafNodeIterator tree_it;
            pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::LeafNodeIterator tree_it_end = octree.leaf_end();

            // Iterate through lower level leafs (voxels).
            for (tree_it = octree.leaf_begin(depth); tree_it!=tree_it_end; ++tree_it)
            {
                //std::cout << "Finding points within voxel " << voxelI << std::endl;

                // Clear clouds and vectors from previous voxel.
                voxelCloud->points.clear();
                voxelCloudNormals->points.clear();
                //featVecHist.clear();

                // Find Voxel Bounds ...
                Eigen::Vector3f voxel_min, voxel_max;
                octree.getVoxelBounds(tree_it, voxel_min, voxel_max);

                //... and center
                pcl::PointXYZRGB voxelCenter;
                voxelCenter.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
                voxelCenter.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
                voxelCenter.z = (voxel_min.z() + voxel_max.z()) / 2.0f;

                // Mark octree leaves as occupied voxels.
                int occx, occy, occz;                                  // voxel indices
                occx = floor((voxelCenter.x - cBB_min_x) /minVoxSize); // floor because 0-indexed array
                occy = floor((voxelCenter.y - cBB_min_y) /minVoxSize);
                occz = floor((voxelCenter.z - cBB_min_z) /minVoxSize);
                //std::cout << "Voxel (" << occx << "," << occy << "," << occz << ") is occupied" << std::endl;
                voxelOccupancy[occx][occy][occz]= true;

                //std::cout << "Voxel center at ("
                //            << voxelCenter.x << " "
                //            << voxelCenter.y << " "
                //            << voxelCenter.z << ")" << std::endl;

                // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                // Neighbors within voxel search
                // It assigns the search point to the corresponding leaf node voxel and returns a vector of point indices.
                // These indices relate to points which fall within the same voxel.
                std::vector<int> voxelPointsIdx;
                if (octree.voxelSearch (voxelCenter, voxelPointsIdx))
                {
                    for (size_t i = 0; i < voxelPointsIdx.size (); ++i)
                    {
                        // Save Points within voxel as cloud
                        voxelCloud->points.push_back(cloud->points[voxelPointsIdx[i]]);
                        voxelCloudNormals->points.push_back(cloud_normals->points[voxelPointsIdx[i]]);

                        octreePoints++;
                    } // for point in voxel
                }
                //std::cout << "Voxel has " << voxelCloud->points.size() << " points. " << std::endl;


                /* =================================================================================*/
                // On each voxel compute the EGI (sphere normal histogram). Normalize and send it out as feature vector.

                // Initialize histogram for new voxel
                for (int i = 0; i < binsPerDim; ++i)
                    for (int j = 0; j < binsPerDim; ++j)
                        for (int k = 0; k < binsPerDim; ++k)
                           voxelHist[voxelI][i][j][k] = 0.0f;

                // Compute normal histogram looping through normals in voxel.
                unsigned int okNormalsVox = 0;
                unsigned int nanCountVox = 0;
                for (size_t i = 0; i < voxelCloudNormals->points.size (); ++i)
                {
                    int xbin, ybin, zbin;
                    float xn, yn, zn;

                    if ((pcl::isFinite(voxelCloudNormals->points[i]))){
                        // Get normal orientation in each dimension
                        xn = voxelCloudNormals->points[i].normal_x;
                        yn = voxelCloudNormals->points[i].normal_y;
                        zn = voxelCloudNormals->points[i].normal_z;
                        //std::cout << "Normal  (" << xn << "," << yn << "," << zn << ")" << std::endl;

                        // Get the bin of the normal orientation in each direction.
                        xbin = floor((xn - rangeMin) /sizeBin); // floor because 0-indexed array
                        ybin = floor((yn - rangeMin) /sizeBin);
                        zbin = floor((zn - rangeMin) /sizeBin);
                        //std::cout << "lies on bin (" << xbin << "," << ybin << "," << zbin << ")" << std::endl;

                        // Increase count of the bin where the normal is assigned
                        voxelHist[voxelI][xbin][ybin][zbin]= voxelHist[voxelI][xbin][ybin][zbin] + 1.0f;
                        okNormalsVox ++;
                    }else{
                        nanCount++;
                        nanCountVox++;
                        //std::cout << "Normal is NaN." << std::endl;

                    }
                }   //- close loop over normal points in voxel
                std::cout << "." ;
                std::cout << "Voxel (" << occx << "," << occy << "," << occz << ")" << "has " << okNormalsVox << " valid points and " << nanCountVox << " NaNs." << std::endl;

                //Normalize by the total number of points in voxel so that each histogram has sum 1, independent of the number of points in the voxel.
                int n =0;
                for (int i = 0; i < binsPerDim; ++i){
                    for (int j = 0; j < binsPerDim; ++j){
                        for (int k = 0; k < binsPerDim; ++k){
                            if (okNormalsVox){              // Avoid dividing by 0.
                                voxelHist[voxelI][i][j][k] = voxelHist[voxelI][i][j][k] / (float) okNormalsVox;
                                voxelsHistMat[occx][occy][occz][n] = voxelHist[voxelI][i][j][k];
                                n++;
                            }

                            // Push into feature vector per histogram
                            //featVecHist.push_back(voxelHist[voxelI][i][j][k]);
                        }
                    }
                }                

                // Keep all histograms form occupied voxels
                // featureVectorOccVox.push_back(featVecHist);

                // XXX Need a 3D matrix structure of histograms so I can retrieve them basedon its voxel index
                // instead of based on a line index which I dont know if its matching. Something like:
                //voxelsHistMat[occx][occy][occz] = featVecHist;

                // Voxel histograms coincide with voxels set as occupied (obviously), otherwise they are filled with all 0s.



                voxelI++;

                // Print out angular histogram
                /*
                cout << "Angular histogram: " << std::endl << "{";
                for (int i = 0; i < binsPerDim; ++i){
                    cout <<  "[" ;
                    for (int j = 0; j < binsPerDim; ++j){
                        for (int k = 0; k < binsPerDim; ++k)
                           cout << voxelHist[voxelI][i][j][k] << "," ;
                        cout <<  std::endl;
                    }
                    cout <<  "]" ;
                }
                cout <<  "}" <<std::endl;
                // */

            } // close for voxel

            // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            // Print out security checks

            // Check that all methods end with the same number of points, to avoid missing or repetition.
            if(verbose){
                cout << "Point Cloud has " << cloud->points.size() << " points." << endl;
                cout << "Normals Cloud has " << cloud->points.size() << " points, of which "<< nanCount << " are NaNs" << endl;
                cout << "Octree found " << octreePoints << " points." << endl;
            }

            // Print out voxel occupancy and stream out histograms
            // /*
            if(verbose){
                cout << "Voxel Occupancy: " << std::endl << "{";
                for (int i = 0; i < voxPerSide; ++i){
                    cout <<  "[" ;
                    for (int j = 0; j < voxPerSide/2; ++j){    //Limit bounding box to the upper part of the tool only, to remove handle from feature list.

                        for (int k = 0; k < voxPerSide; ++k){
                           cout << voxelOccupancy[i][j][k] << "," ;
                        }
                        cout <<  std::endl;
                    }
                    cout <<  "]" ;
                }
                cout <<  "}" <<std::endl;
            }
            //*/

            // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            // Fill the complete feature vector, including histograms for empty voxels.
            int occVoxI = 0;
            std::vector< double > histVec;
            for (int i = 0; i < voxPerSide; ++i){
                for (int j = 0; j < voxPerSide/2; ++j){   //Limit bounding box to the upper part of the tool only, to remove handle from feature list.
                    for (int k = 0; k < voxPerSide; ++k){
                        if (voxelOccupancy[i][j][k]){        // if the voxel is occupied
                            histVec.clear();
                            for (int n = 0; n <histSize; ++n){
                                histVec.push_back(voxelsHistMat[i][j][k][n]);}
                            //histVec = featureVectorOccVox[occVoxI];
                            featureVectorAllVox.toolFeats.push_back(histVec); // copy histogram
                            occVoxI++;
                        }else{                               // if the voxel is empty fill with 0s.
                            std::vector< double >  zeroVector(pow(binsPerDim,3),0.0f);
                            featureVectorAllVox.toolFeats.push_back(zeroVector); // fill with the same amount of zeros
                        }
                    }
                }
            }

            if(verbose){cout <<  "Vector has a size of " << featureVectorAllVox.toolFeats.size() << " x " << histSize << " at depth "<< depth <<endl;}

            // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            // Re-initialize octree with double resolution (half minVoxSize) to analyse next level
            minVoxSize = minVoxSize/2;
            octree.deleteTree();
            octree.setResolution(minVoxSize);       // re-initialize tree with new resolution
            octree.defineBoundingBox(cBB_min_x, cBB_min_y, cBB_min_z,
                                     cBB_max_x, cBB_max_y, cBB_max_z);
            octree.setInputCloud(cloud);
            octree.addPointsFromInputCloud();
        }
    }
    if(verbose){
        // print out end feature vector of vectors
        cout << "Feature vector contains: " << endl << featureVectorAllVox.toString()<< endl;
        cout << endl << "====================== Feature Extraction Done ========================== " << endl;
    }

    // Put featureVectorAllVox in a bottle and send it out.
    feat3DoutPort.write(featureVectorAllVox);

    return true;
}


/***************** Helper Functions *************************************/

bool ToolFeatExt::sendCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
    Bottle &cloudBottle = cloudsOutPort.prepare();
    cloudBottle.clear();

    CloudUtils::cloud2bottle(cloud_in, cloudBottle);
    cout << "Sending cloud of size " << cloud_in->size() << endl;
    cloudsOutPort.write();
    return true;
}

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

    ToolFeatExt module;
    ResourceFinder rf;
    rf.setDefaultContext("AffordancesProject");
    rf.setDefaultConfigFile("realTools.ini");
    rf.setVerbose(true);
    rf.configure(argc, argv);


    cout<<"Configure module..."<<endl;
    module.configure(rf);
    cout<<"Start module..."<<endl;
    module.runModule();

    cout<<"Main returning..."<<endl;
    return 0;
}

