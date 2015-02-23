
#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/Vocab.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

#include "alphanum.hpp"

#ifdef _WIN32
    #include "dirent.h"
#endif



typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;
using namespace yarp::os;

class MergeModule:public RFModule
{
    RpcServer handlerPort;  // port to handle messages
    string path;            // path to folder with .ply files
    string saveName;        // name of the file to save the merged cloud
    bool saving;
    bool closing;
    bool initF;

    int filesScanned;


/************************************************************************/
int MergePointclouds()
	{

	PointCloudT::Ptr cloud_in 	(new PointCloudT); // Original point cloud
	PointCloudT::Ptr cloud_icp	(new PointCloudT); // ICP output point cloud
	PointCloudT::Ptr cloud_filtered	(new PointCloudT); // downsampled point cloud
	
    
    int iterations = 30;
    float distance = 0.003;
	// The Iterative Closest Point algorithm
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(iterations);
    icp.setMaxCorrespondenceDistance (distance);
    icp.setRANSACOutlierRejectionThreshold (distance);
    icp.setTransformationEpsilon (1e-6);
	
	
	DIR *dir;
	struct dirent *ent;
	bool flag=false;
	int filesFound = 0;
    string fname;                   // name of the file with the cloud to load.
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; //filter to remove outliers
	sor.setStddevMulThresh (1.0);

	//loop through all ply-files in the data folder and apply ICP
	if ((dir = opendir (path.c_str())) != NULL) {
        /* get all the files within directory and check if there is any previous merge to start from */

		vector<string> fileNames;
		while ((ent = readdir(dir)) != NULL) {
			int len = strlen (ent->d_name);
			if ( ent->d_type == DT_REG)
			{
                if( strcmp(ent->d_name, saveName.c_str())==0){
					initF = false;		// Mark when there is an existing merged cloud
					cout << "Previous merge found, scanning only new .ply files "  << endl;
				} else {
					if (len >= 4) {
					    if (strcmp (".ply", &(ent->d_name[len - 4])) == 0) {
						printf ("Found valid .ply file %s\n", ent->d_name);
						fileNames.push_back(ent->d_name);
						filesFound++;
					    }else{
					    	printf ("Found non valid file %s\n", ent->d_name);
					    }
					}					
				}					
			}			
		}

        // now sort the vector by alphanumeric name with the algorithm
		std::sort(fileNames.begin(), fileNames.end(), doj::alphanum_less<std::string>());
		// and print the vector to cout
		std::copy(fileNames.begin(), fileNames.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
	  	
		/* Compute the fist cloud*/
		if (initF){					
			fname = fileNames[0];		//load first .ply file of the list to cloud_icp
			cout << "Initialize cloud with: " << fname << endl;
			filesScanned++;
        }else {                         //If there was a previous merge, use it to initialize cloud.
            fname = saveName;
			cout << "Initialize cloud with: " << fname << endl;		
		}
		
		printf ("Loading file: %s\n", fname.c_str());
		if (pcl::io::loadPLYFile (path+fname, *cloud_icp) < 0)	{
			PCL_ERROR("Error loading cloud %s.\n", fname.c_str());
			return -1;
		}

		//filtering					
		sor.setInputCloud (cloud_icp);
		sor.setMeanK(cloud_icp->size()/2);
		sor.filter (*cloud_icp);

		/*load all other files to cloud_in and apply ICP */
		for(int i = filesScanned;  i< fileNames.size() ; i++){
			fname = fileNames[i];
			printf ("Loading file: %s\n", fname.c_str());
			if (pcl::io::loadPLYFile (path+fname, *cloud_in) < 0)	{
				PCL_ERROR("Error loading cloud %s.\n", fname.c_str());
				return -1;
			}

			//filtering
			sor.setInputCloud (cloud_in);
			sor.setMeanK(cloud_in->size()/2);
			sor.filter (*cloud_in);			

			//ICP algorithm
			printf("Applying ICP...");
			icp.setInputSource(cloud_in);
			icp.setInputTarget(cloud_icp);
			icp.align(*cloud_in);
	
			if (icp.hasConverged()) {
				printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());		
			} else {
				PCL_ERROR ("\nICP has not converged.\n");
				return -1;
			}

			*cloud_icp+=*cloud_in;
			filesScanned++;
		}		
  		closedir (dir);
	} else {
  		/* could not open directory */
  		perror ("can't load data files"); 	
		return EXIT_FAILURE;
	}

	//all files are processed, ICP is done	
	printf("ICP finished. %i files processed.\n", filesScanned);
	printf("Merged point cloud size is %lu\n",cloud_icp->size());

	// save data
	printf("Saving data to file...\n");        
    savePointsPly(cloud_icp, saveName);

	// downsampling with voxel grid
	printf("Downsampling point clouds...\n");	
  	pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  	grid.setInputCloud (cloud_icp);
  	grid.setLeafSize (0.005f, 0.005f, 0.005f);
  	grid.filter (*cloud_filtered);

	printf("Downsampled cloud size is %lu\n",cloud_filtered->size());


	//compute surface normals
	printf("Computing surface normals...\n");

	// Create the normal estimation class, and pass the input dataset to it
  	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  	ne.setInputCloud (cloud_filtered);

  	// Create an empty kdtree representation, and pass it to the normal estimation object.
  	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  	ne.setSearchMethod (tree);

  	// Output datasets
  	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  	// Use all neighbors in a sphere of radius 1cm
  	ne.setRadiusSearch (0.01);

  	// Compute the features
  	ne.compute (*cloud_normals);

	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
	printf("cloud size: %lu; normals size: %lu\n", cloud_filtered->points.size(), cloud_normals->points.size());

	//merging = false;
    
    return 0;
}

/************************************************************************/
void savePointsPly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const string& name)
{
    stringstream s;
    s.str("");
    s<< path + name.c_str();
    string filename=s.str();
    string filenameNumb=filename+".ply";
    ofstream plyfile;
    plyfile.open(filenameNumb.c_str());
    plyfile << "ply\n";
    plyfile << "format ascii 1.0\n";
    plyfile << "element vertex " << cloud->width <<"\n";
    plyfile << "property float x\n";
    plyfile << "property float y\n";
    plyfile << "property float z\n";
    plyfile << "property uchar diffuse_red\n";
    plyfile << "property uchar diffuse_green\n";
    plyfile << "property uchar diffuse_blue\n";
    plyfile << "end_header\n";

    for (unsigned int i=0; i<cloud->width; i++){
        plyfile << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " " << (int)cloud->at(i).r << " " << (int)cloud->at(i).g << " " << (int)cloud->at(i).b << "\n";
    }
    plyfile.close();
    fprintf(stdout, "Writing finished\n");
}


public:

    double getPeriod()
    {
        return 1; //module periodicity (seconds)
    }

    bool updateModule()
    {
        return !closing;
    }


    /************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
	/* This method is called when a command string is sent via RPC */
    	reply.clear();  // Clear reply bottle

	/* Get command string */
	string receivedCmd = command.get(0).asString().c_str();
	int responseCode;   //Will contain Vocab-encoded response

	if (receivedCmd == "merge") {        
	    int ok = MergePointclouds();
	    if (ok>=0) { 
		    responseCode = Vocab::encode("ack");
	    } else {
            fprintf(stdout,"Couldn't merge pointclouds. \n");
	        responseCode = Vocab::encode("nack");
	        reply.addVocab(responseCode);
	        return false;
	    }
	    reply.addVocab(responseCode);
	    return true;

    } else if (receivedCmd == "setName") {
        if (command.size() == 2){
            saveName = command.get(1).asString();
            responseCode = Vocab::encode("ack");
            reply.addVocab(responseCode);
            return true;
        } else {
            fprintf(stdout,"Please provide the desired file name. \n");
            reply.addString("[nack] Please provide the desired file name.");
            return false;
        }

    } else if (receivedCmd == "save") {
        if (command.size() == 2){
            string saveF = command.get(1).asString();
            if (saveF == "ON"){
                saving = true;
                fprintf(stdout,"Saving flag is : %s\n", saveF.c_str());
                return true;
            } else if (saveF == "OFF"){
                saving = false;
                fprintf(stdout,"Saving flag is : %s\n", saveF.c_str());
                return true;
            }
            fprintf(stdout,"Saving flag can only be set to ON or OFF. \n");
            reply.addString("[nack] Set save flag to ON or OFF. \n");
            return false;
        }

	}else if (receivedCmd == "help"){
		reply.addVocab(Vocab::encode("many"));		
		reply.addString("Available commands are:");
        reply.addString("merge - Merges all pointclouds on the path folder, or the new ones if a previous merge already exists. Name shall be specified if it's not cloud_merge.ply.");
        reply.addString("setName (string) - Changes the name of the file where the merged cloud will be saved. Default 'cloud_merged.ply'");
        reply.addString("save ON/OFF - Sets ON or OFF saving the resulting merged point cloud. (Default true).");
		//reply.addString("verbose ON/OFF - Sets active the printouts of the program, for debugging or visualization.");
		reply.addString("help - produces this help.");
		reply.addString("quit - closes the module.");
		
		responseCode = Vocab::encode("ack");
		reply.addVocab(responseCode);
		return true;

	} else if (receivedCmd == "quit") {
		closing = true;	
		responseCode = Vocab::encode("ack");
		reply.addVocab(responseCode);
		return true;
        }
    reply.addString("Invalid command, type [help] for a list of accepted commands.");
    
    return true;	
    }

    /************************************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
    string name=rf.check("name",Value("mergeClouds")).asString().c_str();
    string robot = rf.find("robot").asString();
    if (strcmp(robot.c_str(),"icub")==0)
        path = rf.find("clouds_path").asString();
    else
        path = rf.find("clouds_path_sim").asString();

	printf("Path: %s",path.c_str());		
	handlerPort.open("/"+name+"/rpc:i");
        attach(handlerPort);

	/* Module rpc parameters */
	saving = true;
    closing = false;

	/*Init variables*/
    saveName = "cloud_merged.ply";
	initF = true;	
	filesScanned = 0;

	cout << endl << "Configuring done." << endl;

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

    MergeModule module;
    ResourceFinder rf;
    rf.setDefaultContext("toolModeler");
    rf.setDefaultConfigFile("cloudPath.ini");
    rf.setVerbose(true);
    rf.configure(argc, argv);


    cout<<"Configure module..."<<endl;
    module.configure(rf);
    cout<<"Start module..."<<endl;
    module.runModule();

    cout<<"Main returning..."<<endl;
    return 0;
}



