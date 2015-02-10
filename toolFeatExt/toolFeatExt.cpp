
#include <iostream>
#include <math.h>
#include <vector>
#include <ctime>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/Vocab.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>

using namespace std;
using namespace yarp::os;

class ToolFeatExt:public RFModule
{
    RpcServer handlerPort;  // port to handle messages
    string path;            // path to folder with .ply or .pcd files
    string fname;           // name of the .ply or .pcd cloud file

    bool verbose;
    int maxDepth;
    int binsPerDim;

    bool closing;


/************************************************************************/
int computeFeats()
	{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());// Point cloud
    DIR *dir;    
    if ((dir = opendir (path.c_str())) != NULL) {
        printf ("Loading file: %s\n", fname.c_str());

        string::size_type idx;
        idx = fname.rfind('.');
        if(idx != std::string::npos)
        {
            if(strcmp(fname.substr(idx+1).c_str(),"ply"))
            {
                if (pcl::io::loadPLYFile (path+fname, *cloud) < 0)	{
                    PCL_ERROR("Error loading cloud %s.\n", fname.c_str());
                    return -1;
                }
            }else if(strcmp(fname.substr(idx+1).c_str(),"pcd"))
            {
                if (pcl::io::loadPCDFile (path+fname, *cloud) < 0)	{
                    PCL_ERROR("Error loading cloud %s.\n", fname.c_str());
                    return -1;
                }
            }else {
                printf ("Please select .pcd or .ply file.\n");
            }
        }
    } else {
        /* could not open directory */
        perror ("can't load data files"); 	
        return -1;
    }


    /* ===========================================================================*/
    // Get fixed bounding box to avoid octree automatically figure it out (what might change with resolution)
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();

    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);

    /* =========================================================================== */
    // Divide the bounding box in subregions using octree, and compute the normal histogram (EGI) in each of those subregions at different levels.

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Create instances of necessary classes

    // octree
    float minVoxSize = 0.01; // Min voxel size (max resolution) 1 cm
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(minVoxSize);

    // Normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Cloud of Normals from whole pointcloud
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());

    // voxelCloud and voxelCloudNormals to contain the poitns on each voxel
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::Normal>::Ptr voxelCloudNormals(new pcl::PointCloud<pcl::Normal> ());

    // Viewer class for visualization
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

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
    ne.setRadiusSearch (minVoxSize/2);

    // Compute the normals
    ne.compute (*cloud_normals);


    // Initialize histogram variables
    //float pi = 3.14159;

    float rangeMin = -1;    // XXX make this automatic.
    float rangeMax = 1;     //          ´´
    float rangeValid = rangeMax - rangeMin;
    float sizeBin = rangeValid/binsPerDim;

    std::vector < std::vector < float > > featureVectorOccVox;     // Vector containing histograms for occupied voxels
    std::vector < std::vector < float > > featureVectorAllVox;     // Vector containing histograms for all voxels, also the empty ones
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
    for (int depth = 1; depth<=maxDepth; ++depth)
    {
        if(verbose){cout << endl << "====================== Depth:" << depth << " ========================== " << endl;}

        featureVectorOccVox.clear();                                // Clear the vector saving all populated histograms at depth level

        int voxPerSide = pow(2,depth);
        bool voxelOccupancy[voxPerSide][voxPerSide][voxPerSide];    // Keeps record of the position of the occupied voxels at each depth.
        for (int i = 0; i < voxPerSide; ++i)                        // initialize all positions to false
            for (int j = 0; j < voxPerSide; ++j)
                for (int k = 0; k < voxPerSide; ++k)
                   voxelOccupancy[i][j][k] = false;



        // Instantiate normal histogram at actual depth
        int numLeaves = octree.getLeafCount();      // Get the number of occupied voxels (leaves) at actual depth
        float voxelHist[numLeaves][binsPerDim][binsPerDim][binsPerDim]; // Histogram 3D matrix per voxel
        std::vector<float> featVecHist;                                 // Vector containing a seriazed version of the 3D histogram

        if(verbose){
            cout << "BB divided into " << voxPerSide << " voxels per side of size "<< minVoxSize << "," << numLeaves << " occupied." << endl;
            cout << "Looping through the  " << numLeaves << " occupied voxels." << endl;
        }

        // Extracts all the points at leaf level from the octree
        int octreePoints = 0;
        int voxelI = 0;
        int nanCount = 0;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::LeafNodeIterator tree_it;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::LeafNodeIterator tree_it_end = octree.leaf_end();

        // Iterate through lower level leafs (voxels).
        for (tree_it = octree.leaf_begin(depth); tree_it!=tree_it_end; ++tree_it)
        {
            //std::cout << "Finding points within voxel " << voxelI << std::endl;
            //std::cout << "Finding points within voxel " << voxelI << std::endl;

            // Clear clouds and vectors from previous voxel.
            voxelCloud->points.clear();
            voxelCloudNormals->points.clear();
            featVecHist.clear();

            // Find Voxel Bounds ...
            Eigen::Vector3f voxel_min, voxel_max;
            octree.getVoxelBounds(tree_it, voxel_min, voxel_max);

            //... and center
            pcl::PointXYZ voxelCenter;
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
            for (size_t i = 0; i < voxelCloudNormals->points.size (); ++i)
            {
                int xbin, ybin, zbin;
                float xn, yn, zn;

                if ((pcl::isFinite(voxelCloudNormals->points[i]))){
                    xn = voxelCloudNormals->points[i].normal_x;
                    yn = voxelCloudNormals->points[i].normal_y;
                    zn = voxelCloudNormals->points[i].normal_z;
                    //std::cout << "Normal  (" << xn << "," << yn << "," << zn << ")" << std::endl;

                    xbin = floor((xn - rangeMin) /sizeBin); // floor because 0-indexed array
                    ybin = floor((yn - rangeMin) /sizeBin);
                    zbin = floor((zn - rangeMin) /sizeBin);
                    //std::cout << "lies on bin (" << xbin << "," << ybin << "," << zbin << ")" << std::endl;

                    voxelHist[voxelI][xbin][ybin][zbin]= voxelHist[voxelI][xbin][ybin][zbin] + 1.0f;
                    okNormalsVox ++;
                }else{
                    nanCount++;
                    //std::cout << "Normal is NaN." << std::endl;

                }
            }   //- close loop over normal points in voxel
            std::cout << "." ;

            //Normalize by the total number of point in voxel so that each histogram has sum 1, independent of the number of points in the voxel.
            for (int i = 0; i < binsPerDim; ++i){
                for (int j = 0; j < binsPerDim; ++j){
                    for (int k = 0; k < binsPerDim; ++k){
                       voxelHist[voxelI][i][j][k] = voxelHist[voxelI][i][j][k] / (float) okNormalsVox;

                       // Push into feature vector per histogram
                       featVecHist.push_back(voxelHist[voxelI][i][j][k]);
                    }
                }
            }

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

            // Keep all histograms form occupied voxels
            featureVectorOccVox.push_back(featVecHist);

            voxelI++;

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
        /*
        cout << "Voxel Occupancy: " << std::endl << "{";
        for (int i = 0; i < voxPerSide; ++i){
            cout <<  "[" ;
            for (int j = 0; j < voxPerSide; ++j){
                for (int k = 0; k < voxPerSide; ++k){
                   cout << voxelOccupancy[i][j][k] << "," ;
                }
                cout <<  std::endl;
            }
            cout <<  "]" ;
        }
        cout <<  "}" <<std::endl;
        //*/

        // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // Fill the complete feature vector, including histograms for empty voxels.
        int occVoxI = 0;
        for (int i = 0; i < voxPerSide; ++i){
            for (int j = 0; j < voxPerSide; ++j){
                for (int k = 0; k < voxPerSide; ++k){
                    if (voxelOccupancy[i][j][k]){        // if the voxel is occupied
                        std::vector<float> histVec;
                        histVec = featureVectorOccVox[occVoxI];
                        featureVectorAllVox.push_back(histVec); // copy histogram
                        occVoxI++;
                    }else{
                        std::vector < float >  zeroVector(pow(binsPerDim,3),0.0f);
                        featureVectorAllVox.push_back(zeroVector); // fill with the same amount of zeros
                    }
                }
            }
        }

        if(verbose){cout <<  "Vector has a size of " << featureVectorAllVox.size() << " x " << featVecHist.size() << " at depth "<< depth <<endl;}

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
    if(verbose){cout << endl << "======================Feature Extraction Done========================== " << endl;}

    // XXX put featureVectorAllVox in a bottle and send it out.

    return 0;
}


public:

    double getPeriod()
    {
        return 1; //module periodicity (seconds)
    }

    bool updateModule()
    {
        return true;
    }


    /************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
	/* This method is called when a command string is sent via RPC */
    	reply.clear();  // Clear reply bottle

	/* Get command string */
	string receivedCmd = command.get(0).asString().c_str();
	int responseCode;   //Will contain Vocab-encoded response

    if (receivedCmd == "getFeat") {
        int ok = computeFeats();
	    if (ok>=0) { 
		    responseCode = Vocab::encode("ack");
		    reply.addVocab(responseCode);
            return true;
	    } else {
            fprintf(stdout,"3D Features not computed correctly. \n");
	        responseCode = Vocab::encode("nack");
	        reply.addVocab(responseCode);
	        return false;
	    }


	} else if (receivedCmd == "name") {
            if (command.size() == 2){
                fname = command.get(1).asString();
                responseCode = Vocab::encode("ack");
                reply.addVocab(responseCode);
                return true;
            } else {
                fprintf(stdout,"Please provide the .pcd file name. \n");
	            responseCode = Vocab::encode("nack");
	            reply.addVocab(responseCode);
    	        return false;                
            }

    } else if (receivedCmd == "verbose") {

            string verb;
            if (command.size() == 2){
                verb = command.get(1).asString();
                if (verb == "ON"){
                    verbose = true;
                    fprintf(stdout,"Verbose is : %s\n", verb.c_str());
                    return true;
                } else if (verb == "OFF"){
                    verbose = false;
                    fprintf(stdout,"Verbose is : %s\n", verb.c_str());
                    return true;
                }
            } else {
                fprintf(stdout,"Please set verbose ON or OFF. \n");
                responseCode = Vocab::encode("nack");
                reply.addVocab(responseCode);
                return false;
            }

    } else if (receivedCmd == "bins") {
            if (command.size() == 2){
                binsPerDim = command.get(1).asInt();
                responseCode = Vocab::encode("ack");
                reply.addVocab(responseCode);
                return true;
            } else {
                fprintf(stdout,"Couldn't set the numbe of bins. \n");
                responseCode = Vocab::encode("nack");
                reply.addVocab(responseCode);
                return false;
            }

    } else if (receivedCmd == "depth") {
            if (command.size() == 2){
                maxDepth = command.get(1).asInt();
                responseCode = Vocab::encode("ack");
                reply.addVocab(responseCode);
                return true;
            } else {
                fprintf(stdout,"Please provide the .pcd file name. \n");
                responseCode = Vocab::encode("nack");
                reply.addVocab(responseCode);
                return false;
            }

	}else if (receivedCmd == "help"){
		reply.addVocab(Vocab::encode("many"));		
		reply.addString("Available commands are:");
		reply.addString("name (string) - Changes the name of the .ply file to display. Default 'cloud_merged.ply'");
        reply.addString("getFeat - computes 3D oriented -normalized voxel wise EGI - tool featues.");
        reply.addString("bins (int) - sets the number of bins per angular dimension (yaw-pitch-roll) used to compute the normal histogram. Total number of bins per voxel = bins^3. (Default bins = 4)");
        reply.addString("depth (int)- sets the number of iterative times that the bounding box will be subdivided into octants. Total number of voxels = sum(8^(1:depth)). (Default depth = 2, 72 vox)");
        reply.addString("verbose ON/OFF - Sets active the printouts of the program, for debugging or visualization.");
		reply.addString("help - produces this help.");
		reply.addString("quit - closes the module.");
		
		responseCode = Vocab::encode("ack");
		reply.addVocab(responseCode);
		return true;

	} else if (receivedCmd == "quit") {
		responseCode = Vocab::encode("ack");
		reply.addVocab(responseCode);
		closing = true;
		return true;
        }
    reply.addString("Invalid command, type [help] for a list of accepted commands.");
    
    return true;	
    }

    /************************************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
    string name=rf.check("name",Value("toolFeatExt")).asString().c_str();
    string robot = rf.find("robot").asString();
    if (strcmp(robot.c_str(),"icub"))
        path = rf.find("clouds_path").asString();
    else
        path = rf.find("clouds_path_sim").asString();
    printf("Path: %s",path.c_str());

    maxDepth = rf.find("maxDepth").asInt();
    binsPerDim = rf.find("maxDepth").asInt();

	handlerPort.open("/"+name+"/rpc:i");
        attach(handlerPort);

	/* Module rpc parameters */
        closing = false;

	/*Init variables*/
	fname = "cloud_merged.ply";

	cout << endl << "Configuring done."<<endl;

    return true;
    }

    /************************************************************************/    
    bool interruptModule()
    {
        closing = true;
        handlerPort.interrupt();
        cout<<"Interrupting your module, for port cleanup"<<endl;
        return true;
    }

    
    bool close()
    {
        cout<<"Calling close function\n";
        handlerPort.close();
        return true;
    }
};

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
    rf.setDefaultContext("toolModeler");
    rf.setDefaultConfigFile("toolFeatExt.ini");
    rf.setVerbose(true);
    rf.configure(argc, argv);


    cout<<"Configure module..."<<endl;
    module.configure(rf);
    cout<<"Start module..."<<endl;
    module.runModule();

    cout<<"Main returning..."<<endl;
    return 0;
}



