/*
 * TOOL 3D DISPLAY MODULE
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

#include "tool3Dshow.h"


using namespace std;
using namespace yarp::os;

// XXX Add auto-connect to /mesh:o to avoid having to restart it all the time.


/*************************************************************************/
//                  PRIVATE METHODS
/************************************************************************/
void ShowModule::Visualize(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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
void ShowModule::mesh2cloud(const iCub::data3D::SurfaceMeshWithBoundingBox& cloudB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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
//                          PUBLIC METHODS
/************************************************************************/

//                          THRIFT RPC CALLS
/************************************************************************/
bool ShowModule::showFileCloud(const string& cloudname)
	{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>); // Point cloud
    DIR *dir;

    cout << "Attempting to load " << (path + cloudname).c_str() << "... "<< endl;
    if ((dir = opendir (path.c_str())) != NULL) {
        string::size_type idx;
        idx = cloudname.rfind('.');
        if(idx != std::string::npos)
        {
            string ext = cloudname.substr(idx+1);
            cout << "Found file with extension " << ext << endl;
            if(strcmp(ext.c_str(),"ply")==0)
            {
                printf ("Loading .ply file: %s\n", (path + cloudname).c_str());
                if (pcl::io::loadPLYFile (path+cloudname, *cloud_in) < 0)	{
                    PCL_ERROR("Error loading cloud %s.\n", cloudname.c_str());
                    return false;
                }
            }else if(strcmp(ext.c_str(),"pcd")==0)
            {
                printf ("Loading .pcd file: %s\n",(path + cloudname).c_str());
                if (pcl::io::loadPCDFile (path+cloudname, *cloud_in) < 0)	{
                    PCL_ERROR("Error loading cloud %s.\n", cloudname.c_str());
                    return false;
                }
            }else {
                printf ("Please select .pcd or .ply file.\n");
                return false;
            }
        }
    } else {
        /* could not open directory */
        perror ("can't open directory");
        return false;
    }
    printf ("Cloud loaded successfully from file: %s\n",cloudname.c_str());
    fname = cloudname;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    printf("Visualizing point clouds...\n");
    Visualize(viewer, cloud_in);

    return true;
}

string ShowModule::help_commands()
{
        string reply;
        reply += " Available commands are: ----";
        reply += " ShowFileCloud (filename) - Opens visualizer and displays the pointcloud on the given file.----";
        reply += " help_commands - produces this help.----";
        reply += " quit - closes the module.----";

        return reply;
}


bool ShowModule::quit()
{
    std::cout << "Quitting!" << std::endl;
    closing = true;
    return true;
}
    /**********************************************************/
/*
    bool ToolFeatExt::setName(const string& cloudname)
    {   //Changes the name of the .ply file to display. Default 'cloud_merged.ply'"
        fname = cloudname;
        //cloudLoaded = false;

        if (!loadCloud())
        {
            fprintf(stdout,"Couldn't load cloud.\n");
            return false;
        }

        return true;
    }

    */

    /**********************************************************/
/*
    bool Tool3Dshow::setVerbose(const string& verb)
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
*/



    /************************************************************************/
    //                      RF MOUDLE Commands
    /************************************************************************/
    bool ShowModule::configure(yarp::os::ResourceFinder &rf)
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

    // Module rpc parameters
    closing = false;

    //Init variables
    fname = "cloud_merged.ply";

    // Autoconnect to mesh:o port to avoid having to connect on every restart
    Network::connect("/toolExplorer/mesh:o", cloudsInPort.getName()); // From toolExplorer
    Network::connect("/toolFeatExt/mesh:o", cloudsInPort.getName()); // From toolExplorer
    Network::connect("/objectReconstr/mesh:o", cloudsInPort.getName()); // From toolExplorer

	cout << endl << "Configuring done."<<endl;

    return true;
    }

    double ShowModule::getPeriod()
    {
        return 0.5; //module periodicity (seconds)
    }

    bool ShowModule::updateModule()
    {
        // read the mesh
        iCub::data3D::SurfaceMeshWithBoundingBox *cloudMesh = cloudsInPort.read(false);	//keeps on reading until it receives a cloud bottle
        if (cloudMesh!=NULL){
            cout<< "Received Cloud... " << endl;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
            printf("Cloud read from port \n");
            mesh2cloud(*cloudMesh,cloud);

            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
            printf("Visualizing point clouds...\n");
            Visualize(viewer, cloud);
        }
        return !closing;
    }

    bool ShowModule::interruptModule()
    {
        closing = true;
        handlerPort.interrupt();
        cloudsInPort.interrupt();
        cout<<"Interrupting your module, for port cleanup"<<endl;
        return true;
    }

    
    bool ShowModule::close()
    {
        cout<<"Calling close function\n";
        cloudsInPort.close();
        handlerPort.close();
        return true;
    }

    bool ShowModule::attach(RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }



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



