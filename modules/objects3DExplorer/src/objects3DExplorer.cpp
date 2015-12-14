/* 
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Tanis Mar
 * email:  tanis.mar@iit.it
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

#include "objects3DExplorer.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

using namespace iCub::YarpCloud;
  
/**********************************************************
                    PUBLIC METHODS
/**********************************************************/

bool Objects3DExplorer::configure(ResourceFinder &rf)
{
    string name = rf.check("name",Value("objects3DExplorer")).asString().c_str();
    robot = rf.check("robot",Value("icub")).asString().c_str();
    string cloudpath_file = rf.check("from",Value("cloudsPath.ini")).asString().c_str();
    rf.findFile(cloudpath_file.c_str());

    ResourceFinder cloudsRF;
    cloudsRF.setContext("objects3DModeler");
    cloudsRF.setDefaultConfigFile(cloudpath_file.c_str());
    cloudsRF.configure(0,NULL);

    // Set the path that contains previously saved pointclouds
    if (rf.check("clouds_path")){
        cloudsPathFrom = rf.find("clouds_path").asString().c_str();
    }else{
        string defPathFrom = "/share/ICUBcontrib/contexts/objects3DModeler/sampleClouds/";
        string localModelsPath    = rf.check("local_path")?rf.find("local_path").asString().c_str():defPathFrom;     //cloudsRF.find("clouds_path").asString();
        string icubContribEnvPath = yarp::os::getenv("ICUBcontrib_DIR");
        cloudsPathFrom  = icubContribEnvPath + localModelsPath;
    }

    // Set the path where new pointclouds will be saved
    string defSaveDir = "/saveModels";
    string cloudsSaveDir = rf.check("save")?rf.find("save").asString().c_str():defSaveDir;
    if (cloudsSaveDir[0]!='/')
        cloudsSaveDir="/"+cloudsSaveDir;
    cloudsPathTo="."+cloudsSaveDir;

    yarp::os::mkdir_p(cloudsPathTo.c_str());            // Create the save folder if it didnt exist

    printf("Base path to read clouds from: %s",cloudsPathFrom.c_str());
    printf("Path to save new clouds to: %s",cloudsPathTo.c_str());

    cloudName = rf.check("modelName", Value("cloud")).asString();
    hand = rf.check("hand", Value("right")).asString();
    eye = rf.check("camera", Value("left")).asString();
    verbose = rf.check("verbose", Value(true)).asBool();
    toolExploration = rf.check("toolExploration", Value(true)).asBool();

    //ports
    bool ret = true;
    ret = ret && cloudsInPort.open(("/"+name+"/clouds:i").c_str());                    // port to receive pointclouds from
    ret = ret && cloudsOutPort.open(("/"+name+"/clouds:o").c_str());                   // port to send pointclouds to
    if (!ret){
        printf("\nProblems opening ports\n");
        return false;
    }

    // RPC ports
    bool retRPC = true;
    retRPC = rpcPort.open(("/"+name+"/rpc:i").c_str());
    retRPC = retRPC && rpcObjRecPort.open(("/"+name+"/objrec:rpc").c_str());             // port to communicate with object reconstruction module
    retRPC = retRPC && rpcFeatExtPort.open(("/"+name+"/featExt:rpc").c_str());           // port to command the pointcloud feature extraction module
    retRPC = retRPC && rpcVisualizerPort.open(("/"+name+"/visualizer:rpc").c_str());     // port to command the visualizer module
    if (!retRPC){
        printf("\nProblems opening RPC ports\n");
        return false;
    }

    attach(rpcPort);

    printf("\n Opening controllers...\n");

    //Cartesian controllers
    Property optionG("(device gazecontrollerclient)");
    optionG.put("remote","/iKinGazeCtrl");
    optionG.put("local",("/"+name+"/gaze_ctrl").c_str());

    Property optionL("(device cartesiancontrollerclient)");
    optionL.put("remote",("/"+robot+"/cartesianController/left_arm").c_str());
    optionL.put("local",("/"+name+"/cart_ctrl/left_arm").c_str());

    Property optionR("(device cartesiancontrollerclient)");
    optionR.put("remote",("/"+robot+"/cartesianController/right_arm").c_str());
    optionR.put("local",("/"+name+"/cart_ctrl/right_arm").c_str());

    Property optionHL("(device remote_controlboard)");
    optionHL.put("remote",("/"+robot+"/left_arm").c_str());
    optionHL.put("local",("/"+name+"/hand_ctrl/left_arm").c_str());

    Property optionHR("(device remote_controlboard)");
    optionHR.put("remote",("/"+robot+"/right_arm").c_str());
    optionHR.put("local",("/"+name+"/hand_ctrl/right_arm").c_str());

    if (!driverG.open(optionG))
        return false;

    if (!driverL.open(optionL))
    {
        driverG.close();
        return false;
    }

    if (!driverR.open(optionR))
    {
        driverG.close();
        driverL.close();
        return false;
    }

    if (!driverHL.open(optionHL))
    {
        driverG.close();
        driverL.close();
        driverR.close();
        return false;
    }

    if (!driverHR.open(optionHR))
    {
        driverG.close();
        driverL.close();
        driverR.close();
        driverHL.close();
        return false;
    }

    driverG.view(iGaze);
    driverL.view(iCartCtrlL);
    driverR.view(iCartCtrlR);


    iGaze->setSaccadesMode(false);
    if (robot == "icubSim"){
            iGaze->setNeckTrajTime(1.5);
            iGaze->setEyesTrajTime(0.5);
            iGaze->blockEyes(0.0);
    }else{
            iGaze->setNeckTrajTime(1.5);
            iGaze->setEyesTrajTime(0.5);
            iGaze->blockEyes(0.0);
    }

    printf("\nInitializing variables... \n");

    closing = false;
    initAlignment = false;
    numCloudsSaved = 0;
    NO_FILENUM = -1;

    cloud_in = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
    cloud_temp = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
    cloud_merged = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud

    cout << endl << "Configuring done." << endl;

    return true;
}

/************************************************************************/
bool Objects3DExplorer::interruptModule()
{
    closing = true;

    iGaze->stopControl();
    iCartCtrlL->stopControl();
    iCartCtrlR->stopControl();

    IVelocityControl *ivel;
    if (hand=="left")
        driverHL.view(ivel);
    else
        driverHR.view(ivel);
    ivel->stop(4);

    cloudsInPort.interrupt();
    cloudsOutPort.interrupt();

    rpcPort.interrupt();
    rpcObjRecPort.interrupt();
    rpcVisualizerPort.interrupt();
    rpcFeatExtPort.interrupt();

    return true;
}

/************************************************************************/
bool Objects3DExplorer::close()
{
    cloudsInPort.close();
    cloudsOutPort.close();

    rpcPort.close();
    rpcObjRecPort.close();
    rpcVisualizerPort.close();
    rpcFeatExtPort.close();


    driverG.close();
    driverL.close();
    driverR.close();
    driverHL.close();
    driverHR.close();
    return true;
}

/************************************************************************/
double Objects3DExplorer::getPeriod()
{
    return 0.02;
}

/************************************************************************/
bool Objects3DExplorer::updateModule()
{
    return !closing;
}

/************************************************************************/
bool Objects3DExplorer::respond(const Bottle &command, Bottle &reply)
{
	/* This method is called when a command string is sent via RPC */
    reply.clear();  // Clear reply bottle

	/* Get command string */
	string receivedCmd = command.get(0).asString().c_str();
	int responseCode;   //Will contain Vocab-encoded response

    if (receivedCmd == "exploreAuto"){
        // Moves the tool in different direction to obtain different points of view and extracts corresponding partial pointclouds.
        bool ok = exploreAutomatic();
        if (ok){
            reply.addString(" [ack] Exploration successfully finished.");
            return true;
        } else {
            fprintf(stdout,"Couldnt obtain 3D model successfully. \n");
            reply.addString("[nack] Couldnt obtain 3D model successfully.");
            return false;
        }

    }else if (receivedCmd == "exploreInt"){
        // Moves the tool in different direction to obtain different points of view and extracts corresponding partial pointclouds.
        bool ok = exploreInteractive();
        if (ok){
            reply.addString(" [ack] Interactive exploration successfully finished.");
            return true;
        } else {
            fprintf(stdout,"There was an error during the interactive exploration. \n");
            reply.addString("[nack] There was an error during the interactive exploration. ");
            return false;
        }

    }else if (receivedCmd == "turnHand"){
		// Turn the hand 'int' degrees, or go to 0 if no parameter was given.
		int rotDegX = 0;
		int rotDegY = 0;
		if (command.size() == 2){
			rotDegY = command.get(1).asInt();
		} else if  (command.size() == 3){
			rotDegX = command.get(1).asInt();
			rotDegY = command.get(2).asInt();
		}			

        bool ok = turnHand(rotDegX, rotDegY);
        if (ok){
            reply.addString(" [ack] Hand successfully turned.");
            return true;
        } else {
            fprintf(stdout,"Couldnt go to the desired position. \n");
            reply.addString("[nack] Couldnt go to the desired position." );
  		    return false;
		}

	}else if (receivedCmd == "get3D"){
        // segment object and get the pointcloud using objectReconstrucor module save it in file or array        
        bool ok = getPointCloud();
        if (ok) {            
            showPointCloud(cloud_in);
            CloudUtils::savePointsPly(cloud_in, cloudsPathTo, cloudName, numCloudsSaved);
            reply.addString(" [ack] 3D registration successfully completed.");
            return true;
        } else {
		    fprintf(stdout,"Couldnt reconstruct pointcloud. \n");
            reply.addString("[nack] Couldnt reconstruct pointcloud. ");
		    return false;
        }

    }else if (receivedCmd == "findGrasp"){
        // changes the name with which files will be saved by the object-reconstruction module
        string modelname;
        if (command.size() >= 2){
            modelname = command.get(1).asString();
        }else{
            modelname = cloudName;
            return false;
        }
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB> ());
        CloudUtils::loadCloud(cloudsPathFrom, modelname,cloud_out);
        bool ok = findToolPose(cloud_out,toolPose);

        if (ok){
            reply.addString(" [ack] Grasp pose successfully retrieved ");
            return true;
        }else {
            fprintf(stdout,"Grasp pose could not be obtained. \n");
            reply.addString("[nack] Grasp pose could not be obtained \n");
            return false;
        }

    }else if (receivedCmd == "normalize"){
        // activates the normalization of the pointcloud to the hand reference frame.
        bool ok = setNormalization(command.get(1).asString());
        if (ok){
            reply.addString(" [ack] Normalization state successfully set to ");
            reply.addString(command.get(1).asString());
            return true;}
        else {
            fprintf(stdout,"Normalization has to be set to ON or OFF. \n");
            reply.addString("[nack] Normalization has to be set to ON or OFF. ");
            return false;
        }


    }else if (receivedCmd == "FPFH"){
        // activates the normalization of the pointcloud to the hand reference frame.
        bool ok = setInitialAlignment(command.get(1).asString());
        if (ok){
            reply.addString(" [ack] Use of Point Features for initial alignment successfully set to ");
            reply.addString(command.get(1).asString());
            return true;}
        else {
            fprintf(stdout,"FPFH based Initial Alignment has to be set to ON or OFF. \n");
            reply.addString("[nack] FPFH based Initial Alignment has to be set to ON or OFF. ");
            return false;
        }

    }else if (receivedCmd == "modelname"){
        // changes the name with which files will be saved by the object-reconstruction module
        string modelname;
        if (command.size() >= 2){
            modelname = command.get(1).asString();
        }else{
            fprintf(stdout,"Please provide a name. \n");
            return false;
        }
        bool ok = changeModelName(modelname);
        if (ok){
            reply.addString(" [ack] Model name successfully changed to ");
            reply.addString(command.get(1).asString());
            return true;
        }else {
            fprintf(stdout,"Couldnt change the name. \n");
            reply.addString("[nack] Couldnt change the name. ");
            return false;
        }

	}else if (receivedCmd == "verbose"){
		bool ok = setVerbose(command.get(1).asString());
        if (ok){
            reply.addString(" [ack] Verbose successfully set to ");
            reply.addString(command.get(1).asString());
            return true;
        }
		else {
		    fprintf(stdout,"Verbose can only be set to ON or OFF. \n");
            reply.addString("[nack] Verbose can only be set to ON or OFF.");
		    return false;
		}

    }else if (receivedCmd == "loadFromFile"){
        string cloud_file_name = command.get(1).asString();
        cout << "Attempting to load " << (cloudsPathFrom + cloud_file_name).c_str() << "... "<< endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_file (new pcl::PointCloud<pcl::PointXYZRGB> ());

        // load cloud to be displayed
        if (CloudUtils::loadCloud(cloudsPathFrom, cloud_file_name, cloud_file))  {
            cout << "cloud of size "<< cloud_file->points.size() << " points loaded from "<< cloud_file_name.c_str() << endl;
        } else{
            std::cout << "Error loading point cloud " << cloud_file_name.c_str() << endl << endl;
            return false;
        }

        // Display the merged cloud
        showPointCloud(cloud_file);
        reply.addString(" [ack] Cloud successfully displayed");
        return true;


    }else if (receivedCmd == "mergeFromFiles"){
        string cloud_from_name = command.get(1).asString();
        string cloud_to_name = command.get(2).asString();

        cout << "Attempting to load " << (cloudsPathFrom + cloud_from_name).c_str() << "... "<< endl;
        cout << "Attempting to load " << (cloudsPathFrom + cloud_to_name).c_str() << "... "<< endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_from (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to (new pcl::PointCloud<pcl::PointXYZRGB> ());

        // load cloud to be aligned
        if (CloudUtils::loadCloud(cloudsPathFrom, cloud_from_name, cloud_from))  {
            cout << "cloud of size "<< cloud_from->points.size() << " points loaded from "<< cloud_from_name.c_str() << endl;
        } else{
            std::cout << "Error loading point cloud " << cloud_from_name.c_str() << endl << endl;
            return false;
        }

        // load base cloud to align to
        if (CloudUtils::loadCloud(cloudsPathFrom, cloud_to_name, cloud_to))  {
            cout << "cloud of size "<< cloud_to->points.size() << " points loaded from" <<cloud_from_name.c_str() << endl;
        } else{
            std::cout << "Error loading point cloud " << cloud_to_name.c_str() << endl << endl;
            return false;
        }

        // Merge the clouds
        Eigen::Matrix4f alignMatrix;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());
        alignPointClouds(cloud_from, cloud_to, cloud_aligned, alignMatrix);
        *cloud_to+=*cloud_aligned;                            // write aligned registration first to a temporal merged hand
        cout << "Alignment Matrix is "<< endl << alignMatrix << endl;
        // Display the merged cloud
        showPointCloud(cloud_to);

        CloudUtils::savePointsPly(cloud_to, cloudsPathTo,cloudName,numCloudsSaved);

        reply.addString(" [ack] Clouds successfully merged ");
        return true;

	}else if (receivedCmd == "help"){
		reply.addVocab(Vocab::encode("many"));
		responseCode = Vocab::encode("ack");
		reply.addString("Available commands are:");
        reply.addString("exploreAuto - automatically gets 3D pointcloud from different perspectives and merges them in a single model.");
        reply.addString("exploreInt - interactively explores the tool and asks for confirmation on each registration until a proper 3D model is built.");
        reply.addString("turnHand  (int)X (int)Y- moves arm to home position and rotates hand 'int' X and Y degrees around the X and Y axis  (0,0 by default).");
		reply.addString("get3D - segment object and get the pointcloud using objectReconstrucor module.");            
        reply.addString("findGrasp (string) - Find the actual grasp by comparing the actual registration to the given model of the tool.");
		reply.addString("modelname (string) - Changes the name with which the pointclouds will be saved.");
        reply.addString("nomalize (ON/OFF) - Activates/deactivates normalization of the cloud to the hand coordinate frame.");
        reply.addString("FPFH (ON/OFF) - Activates/deactivates fast local features (FPFH) based Initial alignment for registration."); // XXX do FPFH + ICP instead of one or the other
        reply.addString("verbose (ON/OFF) - Sets ON/OFF printouts of the program, for debugging or visualization.");
        reply.addString("help - produces this help.");
		reply.addString("quit - closes the module.");

        reply.addString("test XXX mergeFromFiles (sting) (string) - merges two poinctlouds loaded from the given .ply files.");

		reply.addVocab(responseCode);
		return true;

	} else if (receivedCmd == "quit"){
		responseCode = Vocab::encode("ack");
		reply.addVocab(responseCode);
		closing = true;
		return true;
	}
    
    reply.addString("Invalid command, type [help] for a list of accepted commands.");
    
    return true;

}


/**********************************************************
                    PUBLIC METHODS
/**********************************************************/

/************************************************************************/

// XXX Integrate object3Dexplorer into tool3DModeler, and actually name it object3Dmodeler.
// For that, modify both explorations with a 'bool tool' parameter, to choose between on hand or on table exploration
// That implies whether the arm will be moved around, and more importantly, whether the clouds are normalized to the root or the hand frames.

bool Objects3DExplorer::exploreAutomatic()
{
    // Explore the tool from different angles and save pointclouds
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud

    // Register the first pointcloud to initialize
    bool initDone = false;
    cloud_merged->clear();
    string  mergedName = cloudName + "_merged";
    turnHand(0,0);
    while (!initDone)
    {
        // Register and display the cloud
        getPointCloud();
        showPointCloud(cloud_in);
        *cloud_merged = *cloud_in;  //Initialize cloud merged
        CloudUtils::savePointsPly(cloud_merged, cloudsPathTo, mergedName, NO_FILENUM);
        initDone = true;
        printf("Base cloud initialized \n");
    }

    // Perform Automatic Exploration
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());
    for (int degX = 60; degX>=-75; degX -= 15)
    {
        turnHand(degX,0);
        //lookAround();
        getPointCloud();
        showPointCloud(cloud_in);
        printf("Exploration from  rot X= %i, Y= %i done. \n", degX, 0 );
        CloudUtils::savePointsPly(cloud_in, cloudsPathTo, cloudName, numCloudsSaved);

        // Aling last reconstructred cloud and merge it with the existing cloud_merged
        Eigen::Matrix4f alignMatrix;
        cloud_aligned->clear();
        alignPointClouds(cloud_in, cloud_merged, cloud_aligned, alignMatrix);
        *cloud_merged += *cloud_aligned;

        // Display the merged cloud
        showPointCloud(cloud_merged);
        CloudUtils::savePointsPly(cloud_merged, cloudsPathTo, mergedName, NO_FILENUM);

        Time::delay(0.5);
    }
    for (int degY = 0; degY<=60; degY += 15)
    {
        turnHand(-60,degY);
        //lookAround();
        getPointCloud();
        showPointCloud(cloud_in);
        CloudUtils::savePointsPly(cloud_in,cloudsPathTo, cloudName, numCloudsSaved);
        printf("Exploration from  rot X= %i, Y= %i done. \n", -60, degY );

        // Aling last reconstructred cloud and merge it with the existing cloud_merged
        Eigen::Matrix4f alignMatrix;
        cloud_aligned->clear();
        alignPointClouds(cloud_in, cloud_merged, cloud_aligned, alignMatrix);
        *cloud_merged += *cloud_aligned;

        // Display the merged cloud
        showPointCloud(cloud_merged);
        CloudUtils::savePointsPly(cloud_merged, cloudsPathTo, mergedName, NO_FILENUM);

        Time::delay(0.5);
    }

    printf("Exploration finished, returning control. \n");
    return true;
}

/************************************************************************/
bool Objects3DExplorer::exploreInteractive()
{   // Explore the tool and incrementally check registration and add merge pointclouds if correct

    // set angles for exploration limits
    int minRotX = -75; int maxRotX = 60;
    int minRotY = 0;   int maxRotY = 60;
    int angleStep = 15;

    int posN_X = 1 + (maxRotX-minRotX)/angleStep;
    int posN_Y = 1 + (maxRotY-minRotY)/angleStep;

    Vector positionsX(posN_X);       // Vector with the possible angles on X
    Vector positionsY(posN_Y);       // Vector with the possible angles on Y
    bool visitedPos[posN_X][posN_Y]; // Matrix to check which positions have been visited

    // Fill in the vectors with the possible values
    for (int iX = 0; iX<=posN_X; iX++ )
        positionsX[iX] = minRotX + iX*angleStep;

    for (int iY = 0; iY<=posN_Y; iY++ )
        positionsY[iY] = minRotY + iY*angleStep;

    // Register the first pointcloud to initialize
    bool initDone = false;
    cloud_merged->clear();
    string  mergedName = cloudName + "_merged";
    while (!initDone)
    {
        turnHand(0, 0);

        // Register and display the cloud
        getPointCloud();
        showPointCloud(cloud_in);

        printf("Is the registered cloud clean? (y/n)? \n");
        string answerInit;
        cin >> answerInit;
        if ((answerInit == "y")||(answerInit == "Y"))
        {
            *cloud_merged = *cloud_in;  //Initialize cloud merged
            *cloud_temp = *cloud_in;    //Initialize auxiliary cloud on which temporal merges will be shown before confirmation
            CloudUtils::savePointsPly(cloud_merged, cloudsPathTo, mergedName,NO_FILENUM );
            initDone = true;
            printf("Base cloud initialized \n");
        }else {
            printf(" Try again \n");
        }
    }

    // Perform interactive exploration
    yarp::math::Rand randG; // YARP random generator
    int Xind = 0; int Yind = 0;
    bool explorationDone = false;
    while (!explorationDone)
    {
        Xind = round(randG.scalar()*posN_X);
        Yind = round(randG.scalar()*posN_Y);

        turnHand(positionsX[Xind], positionsY[Yind]);

        // Register and display the cloud
        getPointCloud();
        showPointCloud(cloud_in);

        // If it is ok, do the merge and display again.
        printf("Is the registered cloud clean? (y/n)? \n >> ");
        string answerReg;
        cin >> answerReg;
        if ((answerReg == "y")||(answerReg == "Y"))
        {
            printf("\n Saving partial registration for later use \n ");
            CloudUtils::savePointsPly(cloud_in, cloudsPathTo, cloudName, numCloudsSaved);

            // If the cloud is clean, merge the last recorded cloud_in with the existing cloud_merged and save on cloud_temp
            Eigen::Matrix4f alignMatrix;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());
            alignPointClouds(cloud_in, cloud_merged, cloud_aligned, alignMatrix);
            *cloud_temp += *cloud_aligned;                            // write aligned registration first to a temporal merged

            // Display the merged cloud
            showPointCloud(cloud_temp);

            // If merge is good, update model
            // XXX offer three options
            // - Yes, its is good, save (y)
            // - No, it sucks, go for another registration (n)
            // - Retry merging with different parameters (r)
            // XXX this last option implies making some merging parameters tweakable ( use of FPFH / ICP, iterations, min dist, etc).
            printf("Is the merged cloud clean? (y/n)? \n >> ");
            string answerMerge;
            cin >> answerMerge;
            if ((answerMerge == "y")||(answerMerge == "Y"))
            {
                cloud_merged->clear();
                *cloud_merged = *cloud_temp;
                printf(" The model has been updated \n");

                // Ask if more registrations need to be made, and if so, loop again. Otherwise, break.
                printf(" Should I take another registration? (y/n) \n");
                string answerModel;
                cin >> answerModel;
                if ((answerModel == "y")||(answerModel == "Y"))
                {
                    printf(" Model not finished, continuing with exploration \n");
                } else {
                    CloudUtils::savePointsPly(cloud_merged, cloudsPathTo, mergedName,NO_FILENUM );
                    printf(" Final model saved as %s, finishing exploration \n", mergedName.c_str());
                    explorationDone = true;
                }
            } else {
                printf("\n Ignoring merge, continue with exploration \n");
                *cloud_temp = *cloud_merged;
            }
        } else {
            printf("\n Unproper registration, try again \n");
        }
        printf("Exploration from  rot X= %f, Y= %f done. \n",positionsX[Xind] , positionsY[Yind] );
    }

    printf("Exploration finished, returning control. \n");
    return true;
}

/************************************************************************/
bool Objects3DExplorer::findToolPose(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloud, Matrix toolPose)
{
     // Get a registration
     getPointCloud();    // Registration get and normalized to hand-reference frame. saved as 'cloud_in'

     // Align it to the canonical model
     Eigen::Matrix4f alignMatrix;
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());
     alignPointClouds(cloud_in, modelCloud, cloud_aligned, alignMatrix);

     // return the alineation matrix as toolPose YARP Matrix
     toolPose = CloudUtils::eigMat2yarpMat(alignMatrix);

     return true;
}

/************************************************************************/
bool Objects3DExplorer::turnHand(const int rotDegX, const int rotDegY)
{
    if (hand=="left")
        iCartCtrl=iCartCtrlL;
    else if (hand=="right")
        iCartCtrl=iCartCtrlR;
    else
        return false;

    if ((rotDegY > 70 ) || (rotDegY < -70) || (rotDegX > 90 ) || (rotDegX < -90) )	{
        printf("Rotation out of operational limits. \n");
        return false;
    }

    int context_arm,context_gaze;
    //iGaze->restoreContext(0);

    iCartCtrl->storeContext(&context_arm);
    iGaze->storeContext(&context_gaze);

    // intialize position and orientation matrices
    Matrix Rh(4,4);
    Rh(0,0)=-1.0;         Rh(2,1)=-1.0;         Rh(1,2)=-1.0;         Rh(3,3)=+1.0;
    //Vector r(4,0.0);
    Vector xd(3,0.0);
    Vector offset(3,0.0);;

    // set base position
    xd[0]=-0.30;
    xd[1]=(hand=="left")?-0.1:0.1;					// move sligthly out of center towards the side of the used hand
    xd[2]= 0.1;

    offset[0]=0;
    offset[1]=(hand=="left")?-0.05-(0.01*(rotDegY/10+rotDegX/3)):0.05 + (0.01*(rotDegY/10+rotDegX/3));	// look slightly towards the side where the tool is rotated
    offset[2]= 0.15 - 0.01*abs(rotDegX)/5;

    // Rotate the hand to observe the tool from different positions
    Vector ox(4), oy(4);
    ox[0]=1.0; ox[1]=0.0; ox[2]=0.0; ox[3]=(M_PI/180.0)*(hand=="left"?-rotDegX:rotDegX); // rotation over X axis
    oy[0]=0.0; oy[1]=1.0; oy[2]=0.0; oy[3]=(M_PI/180.0)*(hand=="left"?-rotDegY:rotDegY); // rotation over Y axis

    Matrix Ry=axis2dcm(oy);    // from axis/angle to rotation matrix notation
    Matrix Rx=axis2dcm(ox);
    Matrix R=Rh*Rx*Ry;         // compose the two rotations keeping the order
    Vector od=dcm2axis(R);     // from rotation matrix back to the axis/angle notation


    if (verbose){	printf("Orientation vector matrix is:\n %s \n", od.toString().c_str());	}

    // move!
    //iGaze->setTrackingMode(true);
    iGaze->lookAtFixationPoint(xd+offset);
    iCartCtrl->goToPoseSync(xd,od,1.0);
    iCartCtrl->waitMotionDone(0.1);

    iCartCtrl->restoreContext(context_arm);
    iCartCtrl->deleteContext(context_arm);

    iGaze->restoreContext(context_gaze);
    iGaze->deleteContext(context_gaze);

    return true;
}

/************************************************************************/
bool Objects3DExplorer::lookAround()
{
    Vector fp,fp_aux(3,0.0);
    iGaze->getFixationPoint(fp);
    printf("Looking at %.2f, %.2f, %.2f \n", fp[0], fp[1], fp[2] );
    for (int g = 1; g<6; g++)
    {
        fp_aux[0] = fp[0] + Rand::scalar(-0.02,0.02);
        fp_aux[1] = fp[1] + Rand::scalar(-0.04,0.04);
        fp_aux[2] = fp[2] + Rand::scalar(-0.02,0.02);
        printf("Around Looking at %.2f, %.2f, %.2f \n", fp_aux[0], fp_aux[1], fp_aux[2] );
        iGaze->lookAtFixationPoint(fp_aux);
        iGaze->waitMotionDone(0.05);
    }
    if (verbose){	printf("Looking around done \n");	}

    return true;
}

/************************************************************************/
bool Objects3DExplorer::getPointCloud()
{
    cloud_in->points.clear();
    cloud_in->clear();   // clear receiving cloud    

    // requests 3D reconstruction to objectReconst module
    Bottle cmdOR, replyOR;
    cmdOR.clear();	replyOR.clear();
    cmdOR.addString("seg");
    rpcObjRecPort.write(cmdOR,replyOR);
    
    // read coordinates from yarpview
    if (verbose){printf("Please click on seed point from the Disparity or Segmentation images. \n");}

    // read the cloud from the objectReconst output port
    Bottle *cloudBottle = cloudsInPort.read(true);
    if (cloudBottle!=NULL){
        if (verbose){	cout << "Bottle of size " << cloudBottle->size() << " read from port \n"	<<endl;}
        CloudUtils::bottle2cloud(*cloudBottle,cloud_in);
    } else{
        if (verbose){	printf("Couldnt read returned cloud \n");	}
        return -1;
    }
    
    cmdOR.clear();	replyOR.clear();
    cmdOR.addString("clear");
    rpcObjRecPort.write(cmdOR,replyOR);

    // Apply some filtering to clean the cloud
    // Process the cloud by removing distant points ...
    const float depth_limit = 0.5;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_in);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, depth_limit);
    pass.filter (*cloud_in);

     // ... and removing outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; //filter to remove outliers
    sor.setStddevMulThresh (1.0);
    sor.setInputCloud (cloud_in);
    sor.setMeanK(cloud_in->size()/2);
    sor.filter (*cloud_in);

    // Transform the cloud's frame so that the bouding box is aligned with the hand coordinate frame
    if (toolExploration) {
        printf("Normalizing cloud to hand reference frame \n");
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudNorm (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
        normFrame2Hand(cloud_in, cloud_in);
        printf("Cloud normalized to hand reference frame \n");
    }

    if (verbose){ cout << " Cloud of size " << cloud_in->points.size() << " obtained from 3D reconstruction" << endl;}
    return true;
}

/************************************************************************/
bool Objects3DExplorer::normFrame2Hand(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trans)
{   // Normalizes the frame of the point cloud from the robot frame (as acquired) to the hand frame.

    if (hand=="left")
        iCartCtrl=iCartCtrlL;
    else if (hand=="right")
        iCartCtrl=iCartCtrlR;
    else
        return false;

    // Transform (translate-rotate) the pointcloud by inverting the hand pose
    Vector H2Rpos, H2Ror;
    iCartCtrl->getPose(H2Rpos,H2Ror);
    Matrix H2R = axis2dcm(H2Ror);   // from axis/angle to rotation matrix notation

    // Include translation
    H2R(0,3)= H2Rpos[0];
    H2R(1,3)= H2Rpos[1];
    H2R(2,3)= H2Rpos[2];
    if (verbose){ printf("Hand to robot transformatoin matrix (H2R):\n %s \n", H2R.toString().c_str());}

    Matrix R2H = SE3inv(H2R);    //inverse the affine transformation matrix from robot to hand
    if (verbose){printf("Robot to Hand transformatoin matrix (R2H):\n %s \n", R2H.toString().c_str());}

    // Put Transformation matrix into Eigen Format
    Eigen::Matrix4f TM = Eigen::Matrix4f::Identity();
    TM(0,0) = R2H(0,0);     TM(0,1) = R2H(0,1);     TM(0,2) = R2H(0,2);     TM(0,3) = R2H(0,3);
    TM(1,0) = R2H(1,0);     TM(1,1) = R2H(1,1);     TM(1,2) = R2H(1,2);     TM(1,3) = R2H(1,3);
    TM(2,0) = R2H(2,0);     TM(2,1) = R2H(2,1);     TM(2,2) = R2H(2,2);     TM(2,3) = R2H(2,3);
    TM(3,0) = R2H(3,0);     TM(3,1) = R2H(3,1);     TM(3,2) = R2H(3,2);     TM(3,3) = R2H(3,3);
    //cout << TM.matrix() << endl;

    // Executing the transformation
    pcl::transformPointCloud(*cloud_orig, *cloud_trans, TM);

    if (verbose){	printf("Transformation done \n");	}

    return true;
}

/************************************************************************/
bool Objects3DExplorer::alignPointClouds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_from, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned, Eigen::Matrix4f& transfMat)
{
    if (initAlignment) // Use FPFH features for initial alignment
    {
        printf("Applying FPFH alignment... \n");
        if (verbose){printf("Defining features... \n");}
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr features (new pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr features_target (new pcl::PointCloud<pcl::FPFHSignature33>);

        // Feature computation
        if (verbose){printf("Computing features... \n");}
        computeLocalFeatures(cloud_from, features);
        computeLocalFeatures(cloud_to, features_target);

        // Perform initial alignment
        if (verbose){printf("Setting Initial alignment parameters \n");}
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia;
        sac_ia.setMinSampleDistance (0.05f);
        sac_ia.setMaxCorrespondenceDistance (0.01f*0.01f);
        sac_ia.setMaximumIterations (500);

        if (verbose){printf("Adding source cloud\n");}
        sac_ia.setInputTarget(cloud_to);
        sac_ia.setTargetFeatures (features_target);

        if (verbose){printf("Adding target cloud\n");}
        sac_ia.setInputSource(cloud_from);
        sac_ia.setSourceFeatures (features);

        if (verbose){printf("Aligning clouds\n");}
        sac_ia.align(*cloud_aligned);

        cout << "FPFH has converged:" << sac_ia.hasConverged() << " score: " << sac_ia.getFitnessScore() << endl;

        if (verbose){printf("Getting alineation matrix\n");}
        transfMat = sac_ia.getFinalTransformation();


    } else {          //  Apply good old ICP registration

        printf("Applying ICP alignment... \n");
        // The Iterative Closest Point algorithm
        if (verbose){printf("Setting ICP parameters \n");}
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        //int iterations = 50;
        //float distance = 0.003;   // 3 mm accepted as max distance between models
        //icp.setMaximumIterations(iterations);
        //icp.setMaxCorrespondenceDistance (distance);
        //icp.setRANSACOutlierRejectionThreshold (distance);  // Apply RANSAC too
        //icp.setTransformationEpsilon (1e-6);

        //ICP algorithm
        if (verbose){printf("Adding source cloud \n");}
        icp.setInputSource(cloud_from);
        if (verbose){printf("Adding target cloud \n");}
        icp.setInputTarget(cloud_to);
        if (verbose){printf("Aligning clouds\n");}
        icp.align(*cloud_aligned);
        //pcl::PointCloud<pcl::PointXYZRGB> cloud_test;
        //icp.align(cloud_test);

        cout << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;

        if (verbose){printf("Getting alineation matrix\n");}
        cout << icp.getFinalTransformation() << endl;
        if (verbose){printf("Clouds aligned! \n");}
        transfMat = icp.getFinalTransformation();
    }
    return true;
}

void Objects3DExplorer::computeLocalFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features)
{
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


void Objects3DExplorer::computeSurfaceNormals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
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
bool Objects3DExplorer::showPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    Bottle &cloudBottleOut = cloudsOutPort.prepare();
    cloudBottleOut.clear();
    
    CloudUtils::cloud2bottle(cloud, cloudBottleOut);    
       
    if (verbose){printf("Sending out cloud. \n");}
    cloudsOutPort.write();
    return true;

}

bool Objects3DExplorer::showPointCloudFromFile(const string& fname)
{

    Bottle cmdVis, replyVis;
    cmdVis.clear();	replyVis.clear();
    cmdVis.addString("showFileCloud");
    cmdVis.addString(fname);
    rpcVisualizerPort.write(cmdVis,replyVis);

    return true;
}

/************************************************************************/
bool Objects3DExplorer::extractFeatures() // XXX Change so that cloud can be sent directly via thrift.
{
    // Sends an RPC command to the toolFeatExt module to extract the 3D features of the merged point cloud/
    Bottle cmdFext, replyFext;
    cmdFext.clear();	replyFext.clear();
    cmdFext.addString("getFeat");

    rpcFeatExtPort.write(cmdFext,replyFext);

    return true;
}

/************************************************************************/
bool Objects3DExplorer::changeModelName(const string& modelname)
{
    // Changes the name with which the pointclouds will be saved and read
    cloudName = modelname;

    Bottle cmdOR, replyOR;
    cmdOR.clear();	replyOR.clear();
    cmdOR.addString("name");
    cmdOR.addString(modelname);
    rpcObjRecPort.write(cmdOR,replyOR);

    /*
    Bottle cmdMPC, replyMPC;
    cmdMPC.clear();	replyMPC.clear();
    cmdMPC.addString("name");
    cmdMPC.addString(modelname + "_merged");
    rpcMergerPort.write(cmdMPC,replyMPC);
    */

    Bottle cmdFext, replyFext;
    cmdFext.clear();	replyFext.clear();
    cmdFext.addString("setName");
    cmdFext.addString(modelname + "_merged.ply");
    rpcFeatExtPort.write(cmdFext,replyFext);

    printf("Name changed to %s.\n", modelname.c_str());
return true;
}

/*************************** -Conf Commands- ******************************/
bool Objects3DExplorer::setVerbose(const string& verb)
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
    return false;
}

bool Objects3DExplorer::setNormalization(const string& norm)
{
    if (norm == "ON"){
        toolExploration = true;
        fprintf(stdout,"Normalization is : %s\n", norm.c_str());
        return true;
    } else if (norm == "OFF"){
        toolExploration = false;
        fprintf(stdout,"Normalization is : %s\n", norm.c_str());
        return true;
    }
    return false;
}


bool Objects3DExplorer::setInitialAlignment(const string& fpfh)
{
    if (fpfh == "ON"){
        initAlignment = true;
        fprintf(stdout,"Initial Alignment is : %s\n", fpfh.c_str());
        return true;
    } else if (fpfh == "OFF"){
        initAlignment = false;
        fprintf(stdout,"Initial Alignment is : %s\n", fpfh.c_str());
        return true;
    }
    return false;
}


/************************************************************************/
/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }


    ResourceFinder rf;
    rf.setDefaultContext("objects3DModeler");
    rf.setDefaultConfigFile("objects3DExplorer.ini");
    rf.setVerbose(true);
    rf.configure(argc,argv);

    Objects3DExplorer objects3DExplorer;

    cout<< endl <<"Configure module..."<<endl;
    objects3DExplorer.configure(rf);
    cout<< endl << "Start module..."<<endl;
    objects3DExplorer.runModule();

    cout<<"Main returning..."<<endl;

    return 0;
}


