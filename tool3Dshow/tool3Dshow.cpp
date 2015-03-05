
#include <iostream>
#include <string>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/Vocab.h>

#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iCub/ctrl/math.h>
#include <iCub/data3D/SurfaceMeshWithBoundingBox.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>


using namespace std;
using namespace yarp::os;

class ShowModule:public RFModule
{
    RpcServer handlerPort; //a port to handle messages

    BufferedPort<iCub::data3D::SurfaceMeshWithBoundingBox> cloudsInPort; // Buffered port to receive clouds.

    string path; //path to folder with .ply files
    string fname; //name of the .ply file to show
    bool closing;



/************************************************************************/
void Visualize(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
    string id = "Cloud";

    // Set camera position and orientation
    viewer->setBackgroundColor (0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer->addCoordinateSystem (0.05);    
    //viewer->initCameraParameters();
    //viewer->setSize(1000, 1000); // Visualiser window size

    bool colorCloud = false;
    // check if the loaded file contains color information or not
    for  (int p = 1; p < cloud->points.size(); ++p)
    {
        uint32_t rgb = *reinterpret_cast<int*>(&cloud->points[p].rgb);
        if (rgb != 0)
            colorCloud = true;
    }

    // Add cloud color if it has not
    if (!colorCloud)    {
        // Define R,G,B colors for the point cloud
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler(cloud, 255, 255, 255); //white
        viewer->addPointCloud (cloud, cloud_color_handler, id);
    }else{
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloud_color_handler (cloud);
        viewer->addPointCloud (cloud, cloud_color_handler, id);
    }
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);

    // Display the visualiser
	while (!viewer->wasStopped ()) 
	{
        if (closing)
        {
            viewer->close();
            break;
        }
        viewer->spinOnce ();
        //viewer->spinOnce (100);
        //boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    printf ("Closing visualizer: \n");
    viewer->close();
    viewer->removePointCloud(id);
    closing = true;
}

/************************************************************************/
void mesh2cloud(const iCub::data3D::SurfaceMeshWithBoundingBox& cloudB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{   // Converts mesh from a bottle into pcl pointcloud.
    for (size_t i = 0; i<cloudB.mesh.points.size(); ++i)
    {
        pcl::PointXYZRGB pointrgb;
        pointrgb.x=cloudB.mesh.points.at(i).x;
        pointrgb.y=cloudB.mesh.points.at(i).y;
        pointrgb.z=cloudB.mesh.points.at(i).z;
        if (i<cloudB.mesh.rgbColour.size())
        {
            int32_t rgb= cloudB.mesh.rgbColour.at(i).rgba;
            pointrgb.rgba=rgb;
            pointrgb.r = (rgb >> 16) & 0x0000ff;
            pointrgb.g = (rgb >> 8)  & 0x0000ff;
            pointrgb.b = (rgb)       & 0x0000ff;
        }
        else
            pointrgb.rgb=0;

        cloud->push_back(pointrgb);
    }
    printf("Mesh fromatted as Point Cloud \n");
}

/************************************************************************/
int showFileCloud()
	{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>); // Point cloud
    DIR *dir;

    cout << "Attempting to load " << (path + fname).c_str() << "... "<< endl;
    if ((dir = opendir (path.c_str())) != NULL) {
        string::size_type idx;
        idx = fname.rfind('.');
        if(idx != std::string::npos)
        {
            string ext = fname.substr(idx+1);
            cout << "Found file with extension " << ext << endl;
            if(strcmp(ext.c_str(),"ply")==0)
            {
                printf ("Loading .ply file: %s\n", (path + fname).c_str());
                if (pcl::io::loadPLYFile (path+fname, *cloud_in) < 0)	{
                    PCL_ERROR("Error loading cloud %s.\n", fname.c_str());
                    return -1;
                }
            }else if(strcmp(ext.c_str(),"pcd")==0)
            {
                printf ("Loading .pcd file: %s\n",(path + fname).c_str());
                if (pcl::io::loadPCDFile (path+fname, *cloud_in) < 0)	{
                    PCL_ERROR("Error loading cloud %s.\n", fname.c_str());
                    return -1;
                }
            }else {
                printf ("Please select .pcd or .ply file.\n");
                return -1;
            }
        }
    } else {
        /* could not open directory */
        perror ("can't open directory");
        return -1;
    }

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    printf("Visualizing point clouds...\n");
    Visualize(viewer, cloud_in);

    return 0;
}


public:

    double getPeriod()
    {
        return 0.2; //module periodicity (seconds)
    }

    bool updateModule()
    {
        // read the mesh
        iCub::data3D::SurfaceMeshWithBoundingBox *cloudMesh = cloudsInPort.read(false);	//keeps on reading until it receives a cloud bottle
        if (cloudMesh!=NULL){
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
            printf("Cloud read from port \n");
            mesh2cloud(*cloudMesh,cloud);

            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
            printf("Visualizing point clouds...\n");
            Visualize(viewer, cloud);

        }
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

	if (receivedCmd == "show") {        
        int ok = showFileCloud();
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
                fprintf(stdout,"Please provide the .ply or .pcd file name. \n");
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
    string robot = rf.find("robot").asString();
    if (strcmp(robot.c_str(),"icub")==0)
        path = rf.find("clouds_path").asString();
    else
        path = rf.find("clouds_path_sim").asString();

	printf("Path: %s",path.c_str());		
	handlerPort.open("/"+name+"/rpc:i");
        attach(handlerPort);

    cloudsInPort.open("/"+name+"/mesh:i");

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
        cloudsInPort.interrupt();
        cout<<"Interrupting your module, for port cleanup"<<endl;
        return true;
    }

    
    bool close()
    {
        cout<<"Calling close function\n";
        cloudsInPort.close();
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



