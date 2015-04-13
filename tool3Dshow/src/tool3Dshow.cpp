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

/*************************************************************************/
//                  PRIVATE METHODS
/************************************************************************/

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
bool ShowModule::accumClouds(bool accum)
{
    visThrd->accumulateClouds(accum);
    return true;
}

bool ShowModule::clearVis()
{
    visThrd->clearVisualizer();
    return true;
}


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
    cout << "Cloud of size " << cloud_in->points.size() << " loaded successfully from file: " << cloudname.c_str() << endl;
    fname = cloudname;

    visThrd->updateCloud(cloud_in);

    return true;
}

bool ShowModule::addNormals(double radSearch)
{
    visThrd->addNormals(radSearch);
    return true;
}


bool ShowModule::addBoundingBox(bool minBB)
{
    visThrd->addBoundingBox(minBB);
    return true;
}

string ShowModule::help_commands()
{
        string reply;
        reply += " Available commands are: ----";
        reply += " showFileCloud (str filename) - Opens visualizer and displays the pointcloud on the given file.----";
        reply += " addNormals (double radSearch) - adds Normals to the displayed cloud radSearch is used to find neighboring points.----";
        reply += " addBoundingBox (bool minBB) - adds the bounding box to the displayed cloud. If minBB is true, it will be the minimum BB, otherwise the axis-aligned one.----";
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

    /************************************************************************/
    //                      RF MOUDLE Commands
    /************************************************************************/
    bool ShowModule::configure(yarp::os::ResourceFinder &rf)
    {
    //Ports
    string name=rf.check("name",Value("tool3Dshow")).asString().c_str();
    string robot = rf.check("robot",Value("icub")).asString().c_str();
    cout << "robot: "<< robot.c_str() << endl;
    if (strcmp(robot.c_str(),"icub")==0)
        path = rf.find("clouds_path").asString();
    else
        path = rf.find("clouds_path_sim").asString();


	handlerPort.open("/"+name+"/rpc:i");
        attach(handlerPort);

    cloudsInPort.open("/"+name+"/mesh:i");

    // Module rpc parameters
    closing = false;

    //Init variables
    fname = "cloud_merged.ply";

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

    printf("Base path: %s \n \n",path.c_str());

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

            visThrd->updateCloud(cloud);

            //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
            //printf("Visualizing point clouds...\n");
            //Visualize(viewer, cloud);
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

        if (visThrd)
        {
            visThrd->stop();
            delete visThrd;
            visThrd =  0;
        }

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



