
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
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>


class toolFeatExt:public RFModule
{
    RpcServer handlerPort; //a port to handle messages
    string path;  //path to folder with .ply or .pcl files // XXX update this to allow ply files too
    string fname; // name of the cloud file
    bool closing;


    /*  XXX MODIFIED TILL HERE   <- mode this to the positoin of conversion */

/************************************************************************/
int computeFeats()
	{

    PointCloudT::Ptr cloud_in 	(new PointCloudT); // Point cloud	
    DIR *dir;
    if ((dir = opendir (path.c_str())) != NULL) {
        printf ("Loading file: %s\n", fname.c_str());
        if (pcl::io::loadPLYFile (path+fname, *cloud_in) < 0)	{
	        PCL_ERROR("Error loading cloud %s.\n", fname.c_str());
	        return -1;
        }

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));	
        viewer->setBackgroundColor (0, 0, 0);
        printf("Visualizing point clouds...\n");				
        Visualize(viewer, cloud_in);
    } else {
        /* could not open directory */
        perror ("can't load data files"); 	
        return EXIT_FAILURE;
    }		
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

	if (receivedCmd == "show") {        
	    int ok = showPointcloud();
	    if (ok>=0) { 
		    responseCode = Vocab::encode("ack");
		    reply.addVocab(responseCode);
            return true;
	    } else {
	        fprintf(stdout,"Couldnt show pointcloud. \n");
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
                fprintf(stdout,"PLease provide the .ply file name. \n");
	            responseCode = Vocab::encode("nack");
	            reply.addVocab(responseCode);
    	        return false;                
            }                

	}else if (receivedCmd == "help"){
		reply.addVocab(Vocab::encode("many"));		
		reply.addString("Available commands are:");
		reply.addString("name (string) - Changes the name of the .ply file to display. Default 'cloud_merged.ply'");
		reply.addString("show - Opens visualizer and displays the desired pointcloud.");
		//reply.addString("verbose ON/OFF - Sets active the printouts of the program, for debugging or visualization.");
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
    string name=rf.check("name",Value("tool3Dshow")).asString().c_str();
	path = rf.find("clouds_path").asString();        
	printf("Path: %s",path.c_str());		
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

    ShowModule module;
    ResourceFinder rf;
    rf.setDefaultContext("toolModeler");
    rf.setDefaultConfigFile("configMPC.ini");
    rf.setVerbose(true);
    rf.configure(argc, argv);


    cout<<"Configure module..."<<endl;
    module.configure(rf);
    cout<<"Start module..."<<endl;
    module.runModule();

    cout<<"Main returning..."<<endl;
    return 0;
}



