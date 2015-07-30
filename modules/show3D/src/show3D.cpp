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

#include "show3D.h"

using namespace std;
using namespace yarp::os;
using namespace iCub::YarpCloud;

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

    if (!CloudUtils::loadCloud(cloudpath, cloudname, cloud_in)){
        printf ("Could not load the cloud");
        return false;
    }

    cout << "Cloud of size " << cloud_in->points.size() << " loaded successfully from file: " << cloudname.c_str() << endl;
    cloudfile = cloudname;

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
    string name=rf.check("name",Value("show3D")).asString().c_str();
    string robot = rf.check("robot",Value("icub")).asString().c_str();
    cout << "robot: "<< robot.c_str() << endl;

    // XXX update this so it can get either a path specified by cloudsPath.ini, or as in 'objects3DExplorer', the installation one where sample clouds go.
    if (strcmp(robot.c_str(),"icub")==0)
        cloudpath = rf.find("clouds_path").asString();
    else
        cloudpath = "/home/icub/icubTanis/objects3DModeler/data/data50tools/modelData"; // rf.find("clouds_path_sim").asString();


	handlerPort.open("/"+name+"/rpc:i");
        attach(handlerPort);

    cloudsInPort.open("/"+name+"/clouds:i");

    // Module rpc parameters
    closing = false;

    //Init variables
    cloudfile = "cloud.ply";

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

    double ShowModule::getPeriod()
    {
        return 0.5; //module periodicity (seconds)
    }

    bool ShowModule::updateModule()
    {
        // read the cloudcas bottle
        Bottle *cloudBottle=cloudsInPort.read(false);
        if (cloudBottle!=NULL){
            cout<< "Received Cloud... " << endl;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
            CloudUtils::bottle2cloud(*cloudBottle,cloud);

            visThrd->updateCloud(cloud);

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
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    ShowModule module;
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



