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


// XXX XXX This module deals with 3D object/tool exploration, partial reconstruction extraction, alignment, pose estimation, etc.
// - Provides some functions to explore an object in hand.
// - Communicates with toolFeatExt for OMS-EGI feature extraction.
// - Automatizes partial object recosntruction (communicating with obj3Drec), and alignment
// - Performs pose estimation from model using alignment.
// - Computes tooltip from model and estimated pose

  
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

    saveName = rf.check("saveName", Value("cloud")).asString();
    hand = rf.check("hand", Value("right")).asString();
    camera = rf.check("camera", Value("left")).asString();
    verbose = rf.check("verbose", Value(true)).asBool();
    handFrame = rf.check("handFrame", Value(true)).asBool();
    cloudLoaded = false;
    poseFound  = false;
    seg2D = true;
    initAlignment = true;
    saving = true;

    toolPose = eye(4);
    //tooltip = Vector(4,0.0);

    //icp variables
    icp_maxIt = 100;
    icp_maxCorr = 0.03;
    icp_ranORT = 0.05;
    icp_transEp = 1e-6;

    noise_mean = 0.0;
    noise_sigma = 0.003;


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
    numCloudsSaved = 0;
    NO_FILENUM = -1;

    cloud_temp = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());  // Point cloud
    cloud_model = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ()); // Point cloud
    cloud_pose = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());  // Point cloud

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



    /****   FUNCTION CALLS   ***/
    /******************************************************************************/
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


    }else if (receivedCmd == "loadCloud"){
        string cloud_file_name = command.get(1).asString();
        cout << "Attempting to load " << (cloudsPathFrom + cloud_file_name).c_str() << "... "<< endl;

        // load cloud to be displayed
        if (!CloudUtils::loadCloud(cloudsPathFrom, cloud_file_name, cloud_model))  {
            std::cout << "Error loading point cloud " << cloud_file_name.c_str() << endl << endl;
            return false;
        }

        cout << "cloud of size "<< cloud_model->points.size() << " points loaded from "<< cloud_file_name.c_str() << endl;

        saveName = cloud_file_name.substr(5);
        cloudLoaded = true;        

        // Display the merged cloud
        sendPointCloud(cloud_model);
        reply.addString(" [ack] Cloud successfully displayed");
        return true;



	}else if (receivedCmd == "get3D"){
        // segment object and get the pointcloud using objectReconstrucor module save it in file or array        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rec (new pcl::PointCloud<pcl::PointXYZRGB> ());
        bool ok = getPointCloud(cloud_rec);

        if (ok) {            
            sendPointCloud(cloud_rec);
            reply.addString(" [ack] 3D registration successfully completed.");            
        } else {
		    fprintf(stdout,"Couldnt reconstruct pointcloud. \n");
            reply.addString("[nack] Couldnt reconstruct pointcloud. ");
		    return false;
        }

        if (!cloudLoaded){
            cloud_model = cloud_rec;
        }

        return true;



    }else if (receivedCmd == "findToolPose"){
        // Check if model is loaded, else return false
        if (!cloudLoaded){
            cout << "Model needed to find Pose. Load model" << endl;
            reply.addString("[nack] Load model first to find grasp. \n");
            return false;
        }

        // Find grasp by comparing partial view with model        
        bool ok = findToolPose(cloud_model, cloud_pose, toolPose);

        if (ok){
            reply.addList().read(toolPose);
            return true;
        }else {
            fprintf(stdout,"Grasp pose could not be obtained. \n");
            reply.addString("[nack] Grasp pose could not be obtained \n");
            return false;
        }

    }else if (receivedCmd == "setPose"){
        // Check if model is loaded, else return false
        if (!cloudLoaded){
            cout << "Model needed to set a pose on. Load model" << endl;
            reply.addString("[nack] Load model first to find grasp.");
            return false;
        }

        // Setting default valules
        double ori = 0.0;
        double disp = 0.0;
        double tilt = 45.0;
        double shift = 0.0;

        // Reading only the available parameters
        if (command.size() > 1)
            ori = command.get(1).asDouble();

        if (command.size() > 2)
            disp = command.get(2).asDouble();

        if (command.size() > 3)
            tilt = command.get(3).asDouble();

        if (command.size() > 4)
            shift = command.get(4).asDouble();

        poseFromParam(ori, disp, tilt, shift, toolPose);

        // Set the oriented cloud by transforming model accordign to pose.
        bool ok = setToolPose(cloud_model, toolPose, cloud_pose);
        sendPointCloud(cloud_pose);

        if (ok){
            reply.addString("[ack]");
            return true;
        }else {
            fprintf(stdout,"Grasp pose could not be obtained. \n");
            reply.addString("[nack] Grasp pose could not be obtained");
            return false;
        }

    }else if (receivedCmd == "findTooltipCanon"){
        // Check if model is loaded, else return false
        if (!cloudLoaded){
            cout << "Model needed to find tooltip. Load model" << endl;
            reply.addString("[nack] Load model first to find tooltip.");
            return false;
        }

        // Find grasp by comparing partial view with model
        if(!findTooltipCanon(cloud_model, tooltipCanon)){
            cout << "Could not compute canonical tooltip fom model" << endl;
            reply.addString("[nack] Could not compute canonical tooltip.");
            return false;
        }

        reply.addDouble(tooltipCanon.x);
        reply.addDouble(tooltipCanon.y);
        reply.addDouble(tooltipCanon.z);

        return true;


    }else if (receivedCmd == "findTooltip"){
        // Check if model is loaded, else return false
        if (!cloudLoaded){
            cout << "Model needed to find tooltip. Load model" << endl;
            reply.addString("[nack] Load model first to find tooltip.");
            return false;
        }

        // Find grasp by comparing partial view with model
        if(!findTooltipCanon(cloud_model, tooltipCanon)){
            cout << "Could not compute canonical tooltip fom model" << endl;
            reply.addString("[nack] Could not compute canonical tooltip.");
            return false;
        }

        int green[3] = {0,255,0};
        addPoint(cloud_model, tooltipCanon, green);
        sendPointCloud(cloud_model);

        if (command.size() > 1)        // grasp parameters are given by command
        {
            // Setting default valules
            double ori = 0.0;
            double disp = 0.0;
            double tilt = 45.0;
            double shift = 0.0;

            // Reading only the available parameters
            if (command.size() > 1)
                ori = command.get(1).asDouble();

            if (command.size() > 2)
                disp = command.get(2).asDouble();

            if (command.size() > 3)
                tilt = command.get(3).asDouble();

            if (command.size() > 4)
                shift = command.get(4).asDouble();


            if(!findTipAndPose(tooltipCanon, ori, disp,tilt, shift, tooltip, toolPose)){
                cout << "Could not compute tooltip from the canonical one and the pose parameters" << endl;
                reply.addString("[nack] Could not compute tooltip. \n");
                return false;
            }

        }else{          // grasp parameters are given by command
            // Check if pose has been found, else return false
            if (!poseFound){
                cout << "Pose needed to estimate tooltip, please call  'findToolPose' first." << endl;
                reply.addString("[nack] Pose must be known first to find tooltip.");
                return false;
            }

            if(!findTooltip(tooltipCanon, toolPose, tooltip)){
                cout << "Could not compute tooltip from the canonical one and the pose matrix" << endl;
                reply.addString("[nack] Could not compute tooltip.");
                return false;
            }
        }

        cout << "Transforming the model with pose" << endl;
        setToolPose(cloud_model, toolPose, cloud_pose);
        cloud_pose->erase(cloud_pose->end()); // Remove last point
        addPoint(cloud_pose, tooltip, true);
        Time::delay(1.0);
        sendPointCloud(cloud_pose);


        reply.addDouble(tooltip.x);
        reply.addDouble(tooltip.y);
        reply.addDouble(tooltip.z);

        double ori, displ, tilt, shift;
        paramFromPose(toolPose, ori, displ, tilt, shift);

        cout << "Param returned from paramFromPose = " << ori << ", " << displ << ", " << tilt << ", " << shift << "." << endl;

        reply.addDouble(ori);
        reply.addDouble(displ);
        reply.addDouble(tilt);
        reply.addDouble(shift);

        return true;

    }else if (receivedCmd == "getAffordance"){

        if (!(poseFound)&&(cloudLoaded))
        {
            cout << "Tool and pose need to estimate affordance of tool-pose. Load tool and find pose." << endl;
            reply.addString("[nack] Tool-pose must be known to predict affordances.");
            return false;
        }

        bool ok;
        Bottle aff;
        ok = getAffordances(aff);
        reply = aff;
        if (ok){
            return true;
        } else {
            fprintf(stdout,"Affordance could not be obtained or was not present. \n");
            reply.addString("[nack] Affordance could not be obtained or was not present. ");
            return false;
        }

    }else if (receivedCmd == "extractFeats"){
        if(!extractFeats()){
            cout << "Could not extract the features" << endl;
            reply.addString("[nack]Features not extracted.");
            return false;
        }
        cout << "Features extracted by toolFeatExt" << endl;
        reply.addString("[ack]");
        return true;


    }else if (receivedCmd == "alignFromFiles"){

        // Clear visualizer
        Bottle cmdVis, replyVis;
        cmdVis.clear();	replyVis.clear();
        cmdVis.addString("clearVis");
        rpcVisualizerPort.write(cmdVis,replyVis);

        string cloud_from_name = command.get(1).asString();
        string cloud_to_name = command.get(2).asString();

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_from (new pcl::PointCloud<pcl::PointXYZRGB> ());
        cloud_from->clear();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to (new pcl::PointCloud<pcl::PointXYZRGB> ());
        cloud_to->clear();

        // load cloud to be aligned
        cout << "Attempting to load " << (cloudsPathFrom + cloud_from_name).c_str() << "... "<< endl;
        if (CloudUtils::loadCloud(cloudsPathFrom, cloud_from_name, cloud_from))  {
            cout << "cloud of size "<< cloud_from->points.size() << " points loaded from "<< cloud_from_name.c_str() << endl;
        } else{
            std::cout << "Error loading point cloud " << cloud_from_name.c_str() << endl << endl;
            return false;
        }

        Time::delay(1);
        int blue[3] = {0,0,255};    // Plot partial view blue
        addNoise(cloud_from, noise_mean , noise_sigma);
        changeCloudColor(cloud_from, blue);
        sendPointCloud(cloud_from);

        // Set accumulator mode.
        cmdVis.clear();	replyVis.clear();
        cmdVis.addString("accumClouds");
        cmdVis.addInt(1);
        rpcVisualizerPort.write(cmdVis,replyVis);

        // load model cloud to align to
        cout << "Attempting to load " << (cloudsPathFrom + cloud_to_name).c_str() << "... "<< endl;
        if (CloudUtils::loadCloud(cloudsPathFrom, cloud_to_name, cloud_to))  {
            cout << "cloud of size "<< cloud_to->points.size() << " points loaded from" <<cloud_to_name.c_str() << endl;
        } else{
            std::cout << "Error loading point cloud " << cloud_to_name.c_str() << endl << endl;
            return false;
        }

        Time::delay(1);
        sendPointCloud(cloud_to);
        findTooltipCanon(cloud_to, tooltipCanon);

        // Show clouds original position
        Eigen::Matrix4f alignMatrix;
        Eigen::Matrix4f poseMatrix;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());


        // Find cloud alignment
        alignWithScale(cloud_from, cloud_to, cloud_aligned, alignMatrix, 0.7, 1.3);
        //alignPointClouds(cloud_from, cloud_to, cloud_aligned, alignMatrix);

        // Compute pose matrix as inverse of alignment, and display model on view pose.
        poseMatrix = alignMatrix.inverse();
        pcl::transformPointCloud (*cloud_to, *cloud_pose, poseMatrix);

        //  and format to YARP to send.
        Matrix poseMatYARP = CloudUtils::eigMat2yarpMat(poseMatrix);
        cout << "Alignment Matrix is "<< endl << alignMatrix << endl;
        cout << "Pose Matrix YARP is "<< endl << poseMatYARP.toString() << endl;

        int green[3] = {0,255,0};          // Change color oriented model green
        changeCloudColor(cloud_pose,green);
        poseFound = true;

        // Add tooltip in purple
        findTooltip(tooltipCanon, poseMatYARP, tooltip);
        cloud_pose->erase(cloud_pose->end()); // Remove last point
        addPoint(cloud_pose, tooltip, true);

        //Display oriented cloud.
        sendPointCloud(cloud_pose);
        Time::delay(1);

        cmdVis.clear();	replyVis.clear();
        cmdVis.addString("accumClouds");
        cmdVis.addInt(0);
        rpcVisualizerPort.write(cmdVis,replyVis);

        reply.addString("[ack]");
        reply.addList().read(poseMatYARP);

        //CloudUtils::savePointsPly(cloud_to, cloudsPathTo,cloudName,numCloudsSaved);

        return true;
    }

    /****   Parameter Setting calls ***/
    /******************************************************************************/
    else if (receivedCmd == "handFrame"){
        // activates the normalization of the pointcloud to the hand reference frame.
        bool ok = setHandFrame(command.get(1).asString());
        if (ok){
            reply.addString(" [ack] Transformation to hand frame successfully set to ");
            return true;}
        else {
            fprintf(stdout,"Transformation to hand frame has to be set to ON or OFF. \n");
            reply.addString("[nack] Transformation to hand frame has to be set to ON or OFF. ");
            return false;
        }

    }else if (receivedCmd == "FPFH"){
        // activates the normalization of the pointcloud to the hand reference frame.
        bool ok = setInitialAlignment(command.get(1).asString());
        if (ok){
            reply.addString(" [ack] Use of Point Features for initial alignment successfully set to ");
            return true;}
        else {
            fprintf(stdout,"FPFH based Initial Alignment has to be set to ON or OFF. \n");
            reply.addString("[nack] FPFH based Initial Alignment has to be set to ON or OFF. ");
            return false;
        }

    }else if (receivedCmd == "icp"){
        // icp -> sets parameters for iterative closest point aligning algorithm
        //int maxIt, double maxCorr, double ranORT, double transEp)
            icp_maxIt = command.get(1).asInt();
            icp_maxCorr = command.get(2).asDouble();
            icp_ranORT = command.get(3).asDouble();
            icp_transEp = command.get(4).asDouble();
            cout << " icp Parameters set to " <<  icp_maxIt << ", " << icp_maxCorr << ", " << icp_ranORT<< ", " << icp_transEp << endl;
            return true;

    }else if (receivedCmd == "noise"){
        // icp -> sets parameters for iterative closest point aligning algorithm
        //int maxIt, double maxCorr, double ranORT, double transEp)
            noise_mean = command.get(1).asDouble();
            noise_sigma = command.get(2).asDouble();
            cout << "Noise Parameters set to mean:" <<  noise_mean << ", sigma: " << noise_sigma << endl;
            return true;

    }else if (receivedCmd == "verbose"){
        bool ok = setSeg(command.get(1).asString());
        if (ok){
            reply.addString(" [ack] Segmentation successfully set to ");
            return true;
        }
        else {
            fprintf(stdout,"Verbose can only be set to ON or OFF. \n");
            reply.addString("[nack] Verbose can only be set to ON or OFF.");
            return false;
        }

    }else if (receivedCmd == "savename"){
        // changes the name with which files will be saved by the object-reconstruction module
        string save_name;
        if (command.size() >= 2){
            save_name = command.get(1).asString();
        }else{
            fprintf(stdout,"Please provide a name. \n");
            return false;
        }
        bool ok = changeSaveName(save_name);
        if (ok){
            reply.addString(" [ack] Model name successfully changed to ");
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
            return true;
        }
		else {
		    fprintf(stdout,"Verbose can only be set to ON or OFF. \n");
            reply.addString("[nack] Verbose can only be set to ON or OFF.");
		    return false;
		}


    }else if (receivedCmd == "saving"){
        // changes whether the reconstructed clouds will be saved or not.
        bool ok = setSaving(command.get(1).asString());
        if (ok){
            reply.addString(" [ack] Recorded clouds saved ");
            return true;
        }else {
            fprintf(stdout,"Verbose can only be set to ON or OFF. \n");
            reply.addString("[nack] Verbose can only be set to ON or OFF.");
            return false;
        }

	}else if (receivedCmd == "help"){
		reply.addVocab(Vocab::encode("many"));
		responseCode = Vocab::encode("ack");
		reply.addString("Available commands are:");
        reply.addString("-------------EXPLORATIVE ACTIONS-----------");
        reply.addString("exploreAuto - automatically gets 3D pointcloud from different perspectives and merges them in a single model.");
        reply.addString("exploreInt - interactively explores the tool and asks for confirmation on each registration until a proper 3D model is built.");
        reply.addString("turnHand  (int)X (int)Y- moves arm to home position and rotates hand 'int' X and Y degrees around the X and Y axis  (0,0 by default).");

        reply.addString("-----------CLOUD INFO -----------");
        reply.addString("loadCloud - Loads a cloud from a file (.ply or .pcd).");
		reply.addString("get3D - segment object and get the pointcloud using objectReconstrucor module.");            
        reply.addString("findToolPose - Find the actual grasp by comparing the actual registration to the given model of the tool.");
        reply.addString("setPose (double)orientation (double)displacement (double)tilt (double)shift - Set the tool pose given the grasp parameters.");
        reply.addString("findTooltip - Computes the tooltip by rotating the canonical toolpose with the estimated tool pose.");
        reply.addString("alignFromFiles (sting)part (string)model - merges cloud 'part' to cloud 'model' loaded from .ply or .pcd files.");
        reply.addString("getAffordance - returns the pre-learnt affordances for the loaded tool-pose.");

        reply.addString("---------- SET PARAMETERS ------------");
        reply.addString("handFrame (ON/OFF) - Activates/deactivates transformation of the registered clouds to the hand coordinate frame. (default ON).");
        reply.addString("FPFH (ON/OFF) - Activates/deactivates fast local features (FPFH) based Initial alignment for registration. (default ON).");
        reply.addString("icp (int)maxIt (double)maxCorr (double)ranORT (double)transEp - sets ICP parameters (default 100, 0.03, 0.05, 1e-6).");
        reply.addString("noise (double)mean (double)sigma - sets noise parameters (default 0.0, 0.003)");
        reply.addString("seg2D (ON/OFF) - Set the segmentation to 2D (ON) from graphBasedSegmentation, or 3D (OFF), from 'flood3d' .");
        reply.addString("savename (string) - Changes the name with which the pointclouds will be saved.");
        reply.addString("save (ON/OFF) - Controls whether recorded clouds are saved or not.");
        reply.addString("verbose (ON/OFF) - Sets ON/OFF printouts of the program, for debugging or visualization.");
        reply.addString("help - produces this help.");
		reply.addString("quit - closes the module.");

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
                    PROTECTED METHODS
/**********************************************************/

/*  EXPLORATIVE ACTIONS  */
/************************************************************************/

bool Objects3DExplorer::exploreAutomatic()
{
    // Explore the tool from different angles and save pointclouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rec (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud

    // Register the first pointcloud to initialize
    bool initDone = false;
    cloud_model->clear();
    string  mergedName = saveName + "_merged";
    turnHand(0,0);
    while (!initDone)
    {
        // Register and display the cloud
        getPointCloud(cloud_rec);
        sendPointCloud(cloud_rec);
        *cloud_model = *cloud_rec;  //Initialize cloud merged
        CloudUtils::savePointsPly(cloud_model, cloudsPathTo, mergedName, NO_FILENUM);
        initDone = true;
        printf("Base cloud initialized \n");
    }

    // Perform Automatic Exploration
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());
    for (int degX = 60; degX>=-75; degX -= 15)
    {
        turnHand(degX,0);
        //lookAround();
        getPointCloud(cloud_rec);
        sendPointCloud(cloud_rec);
        printf("Exploration from  rot X= %i, Y= %i done. \n", degX, 0 );
        CloudUtils::savePointsPly(cloud_rec, cloudsPathTo, saveName, numCloudsSaved);

        // Aling last reconstructred cloud and merge it with the existing cloud_model
        Eigen::Matrix4f alignMatrix;
        alignPointClouds(cloud_rec, cloud_model, cloud_aligned, alignMatrix);
        *cloud_model += *cloud_aligned;

        // Display the merged cloud
        sendPointCloud(cloud_model);
        CloudUtils::savePointsPly(cloud_model, cloudsPathTo, mergedName, NO_FILENUM);

        Time::delay(0.5);
    }
    for (int degY = 0; degY<=60; degY += 15)
    {
        turnHand(-60,degY);
        //lookAround();
        getPointCloud(cloud_rec);
        sendPointCloud(cloud_rec);
        CloudUtils::savePointsPly(cloud_rec,cloudsPathTo, saveName, numCloudsSaved);
        printf("Exploration from  rot X= %i, Y= %i done. \n", -60, degY );

        // Aling last reconstructred cloud and merge it with the existing cloud_model
        Eigen::Matrix4f alignMatrix;
        alignPointClouds(cloud_rec, cloud_model, cloud_aligned, alignMatrix);
        *cloud_model += *cloud_aligned;

        // Display the merged cloud
        sendPointCloud(cloud_model);
        CloudUtils::savePointsPly(cloud_model, cloudsPathTo, mergedName, NO_FILENUM);

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rec (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud

    bool initDone = false;
    cloud_model->clear();
    string  mergedName = saveName + "_merged";
    while (!initDone)
    {
        turnHand(0, 0);

        // Register and display the cloud
        getPointCloud(cloud_rec);
        sendPointCloud(cloud_rec);

        printf("Is the registered cloud clean? (y/n)? \n");
        string answerInit;
        cin >> answerInit;
        if ((answerInit == "y")||(answerInit == "Y"))
        {
            *cloud_model = *cloud_rec;  //Initialize cloud merged
            *cloud_temp = *cloud_rec;    //Initialize auxiliary cloud on which temporal merges will be shown before confirmation
            CloudUtils::savePointsPly(cloud_model, cloudsPathTo, mergedName,NO_FILENUM );
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
        getPointCloud(cloud_rec);
        sendPointCloud(cloud_rec);

        // If it is ok, do the merge and display again.
        printf("Is the registered cloud clean? (y/n)? \n >> ");
        string answerReg;
        cin >> answerReg;
        if ((answerReg == "y")||(answerReg == "Y"))
        {
            printf("\n Saving partial registration for later use \n ");
            CloudUtils::savePointsPly(cloud_rec, cloudsPathTo, saveName, numCloudsSaved);

            // If the cloud is clean, merge the last recorded cloud_rec with the existing cloud_merged and save on cloud_temp
            Eigen::Matrix4f alignMatrix;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());
            alignPointClouds(cloud_rec, cloud_model, cloud_aligned, alignMatrix);
            *cloud_temp += *cloud_aligned;                            // write aligned registration first to a temporal merged

            // Display the merged cloud
            sendPointCloud(cloud_temp);

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
                cloud_model->clear();
                *cloud_model = *cloud_temp;
                printf(" The model has been updated \n");

                // Ask if more registrations need to be made, and if so, loop again. Otherwise, break.
                printf(" Should I take another registration? (y/n) \n");
                string answerModel;
                cin >> answerModel;
                if ((answerModel == "y")||(answerModel == "Y"))
                {
                    printf(" Model not finished, continuing with exploration \n");
                } else {
                    CloudUtils::savePointsPly(cloud_model, cloudsPathTo, mergedName, NO_FILENUM );
                    printf(" Final model saved as %s, finishing exploration \n", mergedName.c_str());
                    explorationDone = true;
                }
            } else {
                printf("\n Ignoring merge, continue with exploration \n");
                *cloud_temp = *cloud_model;
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



/* CLOUD INFO */
/************************************************************************/
bool Objects3DExplorer::getPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rec)
{
    cloud_rec->points.clear();
    cloud_rec->clear();   // clear receiving cloud

    // requests 3D reconstruction to objectReconst module
    Bottle cmdOR, replyOR;
    cmdOR.clear();	replyOR.clear();
    if (seg2D){
        cmdOR.addString("seg");
        if (verbose){printf("Please click on seed point from the Disparity or Segmentation images. \n");}
    } else {
        cmdOR.addString("flood3d");
        if (verbose){printf("Please click on seed point from the Disparity image. \n");}
    }
    rpcObjRecPort.write(cmdOR,replyOR);


    // read the cloud from the objectReconst output port
    Bottle *cloudBottle = cloudsInPort.read(true);
    if (cloudBottle!=NULL){
        if (verbose){	cout << "Bottle of size " << cloudBottle->size() << " read from port \n"	<<endl;}
        CloudUtils::bottle2cloud(*cloudBottle,cloud_rec);
    } else{
        if (verbose){	printf("Couldnt read returned cloud \n");	}
        return false;
    }

    cmdOR.clear();	replyOR.clear();
    cmdOR.addString("clear");
    rpcObjRecPort.write(cmdOR,replyOR);

    // Apply some filtering to clean the cloud
    // Process the cloud by removing distant points ...
    const float depth_limit = 0.5;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud_rec);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, depth_limit);
    pass.filter (*cloud_rec);

     // ... and removing outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; //filter to remove outliers
    sor.setStddevMulThresh (1.0);
    sor.setInputCloud (cloud_rec);
    sor.setMeanK(cloud_rec->size()/2);
    sor.filter (*cloud_rec);

    scaleCloud(cloud_rec,1.3);

    // Transform the cloud's frame so that the bouding box is aligned with the hand coordinate frame
    if (handFrame) {
        printf("Transforming cloud to hand reference frame \n");
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudNorm (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
        frame2Hand(cloud_rec, cloud_rec);
        printf("Cloud transformed to hand reference frame \n");
    }

    if (verbose){ cout << " Cloud of size " << cloud_rec->points.size() << " obtained from 3D reconstruction" << endl;}

    if (saving){
        CloudUtils::savePointsPly(cloud_rec, cloudsPathTo, saveName, numCloudsSaved);}

    return true;
}


/************************************************************************/
bool Objects3DExplorer::frame2Hand(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trans)
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
    Eigen::Matrix4f TM = CloudUtils::yarpMat2eigMat(R2H);
    //cout << TM.matrix() << endl;

    // Executing the transformation
    pcl::transformPointCloud(*cloud_orig, *cloud_trans, TM);

    if (verbose){	printf("Transformation done \n");	}

    return true;
}

/************************************************************************/
bool Objects3DExplorer::findToolPose(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr poseCloud, Matrix &pose)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rec (new pcl::PointCloud<pcl::PointXYZRGB> ());
    Bottle cmdVis, replyVis;
    bool poseValid = false;
    int trial = 0;

    cmdVis.clear();	replyVis.clear();
    cmdVis.addString("accumClouds");
    cmdVis.addInt(1);
    rpcVisualizerPort.write(cmdVis,replyVis);

    while (!poseValid){

        // Set accumulator mode.
        cmdVis.clear();	replyVis.clear();
        cmdVis.addString("clearVis");
        rpcVisualizerPort.write(cmdVis,replyVis);

        Time::delay(1);
        sendPointCloud(modelCloud);
        Time::delay(1);

        // Get a registration
        getPointCloud(cloud_rec);              // Registration get and normalized to hand-reference frame.
        int blue[3] = {0,0,255};               // Plot oriented model green
        changeCloudColor(cloud_rec, blue);
        sendPointCloud(cloud_rec);
        Time::delay(1);

        // Align it to the canonical model
        Eigen::Matrix4f alignMatrix;
        Eigen::Matrix4f poseMatrix;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());

        if (!alignWithScale(cloud_rec, modelCloud, cloud_aligned, alignMatrix, 0.7, 1.3))
        //if (!alignPointClouds(cloud_rec, modelCloud, cloud_aligned, alignMatrix))
        return false;

        // Inverse the alignment to find tool pose
        poseMatrix = alignMatrix.inverse();

        // transform pose Eigen matrix to YARP Matrix
        pose = CloudUtils::eigMat2yarpMat(poseMatrix);
        cout << "Estimated Pose:" << endl << pose.toString() << endl;

        double ori,displ, tilt, shift;
        paramFromPose(pose, ori, displ, tilt, shift);
        //cout << "Corresponds to parameters: or= " << ori << ", disp= " << displ << ", tilt= " << tilt << ", shift= " << shift << "." <<endl;

        poseValid = checkGrasp(pose);

        poseCloud->clear();
        pcl::transformPointCloud(*modelCloud, *poseCloud, poseMatrix);

        int green[3] = {0,255,0};          // Plot oriented model green
        changeCloudColor(poseCloud, green);
        sendPointCloud(poseCloud);
        Time::delay(1);

        if (!poseValid) {
            cout << "The estimated grasp is not possible, retry with a new pointcloud" << endl;
        }

        trial++;  // to limit number of trials
        if (trial > 10){
            cout << "Could not find a valid grasp in 10 trials" << endl;
            return false;
        }

    }

    cmdVis.clear();	replyVis.clear();
    cmdVis.addString("accumClouds");
    cmdVis.addInt(0);
    rpcVisualizerPort.write(cmdVis,replyVis);

    poseFound = true;
    return true;
}

bool Objects3DExplorer::findTooltipCanon(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloud, Point3D& ttCanon)
{
    // returns the xyz coordinates of the tooltip with respect to the hand coordinate frame, for the loaded model.
    // It computes the tooltip point first from the canonical model, ie, with tool oriented on -Y and end-effector always oriented along X (toolpose front).
    // Then the point coordinates are rotated in the same manner as the tool (tilted and end-effector rotated).

    // The canonical tooltip is considered to be the the middle point of the upper edge opposite to the hand (tt: tooltip, H: hand (origin))
    //           __tt__
    //          /     /|    |^| -Y-axis             so: tt.x = maxBB.x
    //         /_____/ |                                tt.y = minBB.y
    //         |     | /    /^/ X-axis                  tt.z = (maxBB.z + minBB.z)/2
    //         |__H__|/     <-- Z-axis

    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud(modelCloud);
    feature_extractor.compute();

    pcl::PointXYZRGB min_point_AABB;
    pcl::PointXYZRGB max_point_AABB;
    if (!feature_extractor.getAABB(min_point_AABB, max_point_AABB))
        return false;

    // cout << endl<< "Max AABB x: " << max_point_AABB.x << ". Min AABB x: " << min_point_AABB.x << endl;
    // cout << "Max AABB y: " << max_point_AABB.y << ". Min AABB y: " << min_point_AABB.y << endl;
    // cout << "Max AABB z: " << max_point_AABB.z << ". Min AABB z: " << min_point_AABB.z << endl;

    double effLength = fabs(max_point_AABB.x- min_point_AABB.x);        //Length of the effector
    ttCanon.x = max_point_AABB.x-effLength/3;                           // tooltip not on the extreme, but sligthly in  -  x coord of ttCanon
    ttCanon.y = min_point_AABB.y;                                       // y coord of ttCanon
    ttCanon.z = (max_point_AABB.z + min_point_AABB.z)/2;                // z coord of ttCanon

    cout << "Canonical tooltip at ( " << ttCanon.x << ", " << ttCanon.y << ", " << ttCanon.z <<")." << endl;    
    return true;
}


bool Objects3DExplorer::findTooltip(const Point3D &ttCanon, const Matrix &pose, Point3D &tooltipTrans)
{

    Vector ttCanonVec(4,0.0), tooltipVec(4);
    ttCanonVec[0] = ttCanon.x;
    ttCanonVec[1] = ttCanon.y;
    ttCanonVec[2] = ttCanon.z;
    ttCanonVec[3] = 1.0;

    // Rotate the tooltip Canon according to the toolpose
    tooltipVec = pose*ttCanonVec;

    tooltipTrans.x = tooltipVec[0];
    tooltipTrans.y = tooltipVec[1];
    tooltipTrans.z = tooltipVec[2] + 0.03; // Add 3 cm in the direction of Z because the tool origin is not exactin IN the palm, but ON the palm, 3cm let of the refrence frame

    cout << "Transformed tooltip at ( " << tooltipTrans.x << ", " << tooltipTrans.y << ", " << tooltipTrans.z <<")." << endl;

    return true;
}


bool Objects3DExplorer::findTipAndPose(const Point3D &tooltipCanon, const double graspOr, const double graspDisp, const double graspTilt, const double graspShift, Point3D &tooltipTrans, Matrix &pose)
{

    cout << "Transforming Canonical tooltip (" << tooltipCanon.x << ", " << tooltipCanon.y << ", " << tooltipCanon.z << ")" << endl;
    cout << "With parameters: or= " << graspOr << ", disp= " << graspDisp << ", tilt= " << graspTilt << ", shift= " << graspShift << "." <<endl;

    poseFromParam(graspOr, graspDisp, graspTilt, graspShift, pose);     // Get the pose matrix from the parameters

    findTooltip(tooltipCanon, pose, tooltipTrans);                      // Rotate tooltip with pose

    return true;
}


/************************************************************************/
bool Objects3DExplorer::paramFromPose(const Matrix &pose, double &ori, double &displ, double &tilt, double &shift)
{
    Matrix R = pose.submatrix(0,2,0,2); // Get the rotation matrix

    double rotZ = atan2(R(1,0), R(0,0));
    double rotY = atan2(-R(2,0),sqrt(pow(R(2,1),2) + pow(R(2,2),2)));
    double rotX = atan2(R(2,1),R(2,2));

    ori = -rotY* 180.0/M_PI;        // Orientation in degrees
    displ = -pose(1,3) * 100.0;     // Displacement along -Y axis in cm
    tilt = rotZ* 180.0/M_PI;        // Tilt in degrees
    shift = pose(2,3) * 100.0;      // Displacement along Z axis in cm

    //cout << "Parameters computed: or= " << ori << ", disp= " << displ << ", tilt= " << tilt << ", shift= " << shift << "." <<endl;

    return true;
}

/************************************************************************/
bool Objects3DExplorer::poseFromParam(const double ori, const double disp, const double tilt, const double shift, Matrix &pose)
{

    // Rotates the tool model 'deg' degrees around the hand -Y axis
    // Positive angles turn the end effector "inwards" wrt the iCub, while negative ones rotate it "outwards" (for tool on the right hand).
    double radOr = ori*M_PI/180.0; // converse deg into rads
    double radTilt = tilt*M_PI/180.0; // converse deg into rads

    cout << "Ori: rotation of "<< ori << " around -Y: " << radOr << "Rad." << endl;
    Vector oy(4);   // define the rotation over the -Y axis or effector orientation
    oy[0]=0.0; oy[1]=-1.0; oy[2]=0.0; oy[3]= radOr; // tool is along the -Y axis!!
    Matrix R_ori = axis2dcm(oy);          // from axis/angle to rotation matrix notation
    cout << endl << "R_ori = "<< endl << R_ori.toString() << endl;

    cout << "Tilt: rotation of "<< tilt << " around Z: "<< radTilt << "Rad." << endl;
    Vector oz(4);   // define the rotation over the Z axis for tilt.
    oz[0]=0.0; oz[1]=0.0; oz[2]=1.0; oz[3]= radTilt; // tilt around the Z axis!!
    Matrix R_tilt = axis2dcm(oz);
    cout << endl <<"R_tilt = "<< endl << R_tilt.toString() << endl;

    pose = R_tilt*R_ori;         // Compose matrices in order, first rotate around -Y (R_ori), then around Z
    cout << "Combined rotation matrix R = R_tilt*R_ori" << endl;
    cout << endl << "R = "<< endl << pose.toString() << endl;

    pose(1,3) = -disp /100.0;   // This accounts for the traslation of 'disp' in the -Y axis in the hand coord system along the extended thumb).
    cout << "Disp: Translation of "<< disp << " along -Y " << endl;

    pose(2,3) = shift/100.0;   // This accounts for the Z translation to match the tooltip    
    cout << "Shift: Translation of "<< shift << " along Z " << endl;

    cout << "pose = "<< endl << pose.toString() << endl;

    return true;
}

/************************************************************************/
bool Objects3DExplorer::getAffordances(Bottle &tpAff)
{
    cout << "Computing affordances of the tool-pose in hand " << endl;
    Matrix affMatrix(12,3);
    // affMatrix contains the pre-learnt affordances of the 4 possible tools. (can be easily extended for new tools).
    // each row is corresponds to a tool-pose, each column (p) to a possible action (a).
    // Thus, tools are represented in groups of 3 rows, corresponding to poses : left, front, right.
    // The value on (p,a) represents if the action 'a' can be achieved with tool-pose 'p', and therfore is boolean (could also be extended to percentage).

    //    Drag Left           Drag Diagonal left        Drag down
    // Tool 1 -> hoe
    affMatrix(0,0) = 0.0;   affMatrix(0,1) = 0.0;   affMatrix(0,2) = 1.0;  // Pose left (90)
    affMatrix(1,0) = 0.0;   affMatrix(1,1) = 1.0;   affMatrix(1,2) = 0.0;  // Pose front (0)
    affMatrix(2,0) = 1.0;   affMatrix(2,1) = 0.0;   affMatrix(2,2) = 0.0;  // Pose right (-90)

    // Tool 2 -> hook
    affMatrix(3,0) = 1.0;   affMatrix(3,1) = 0.0;   affMatrix(3,2) = 0.0;  // Pose left  (90)
    affMatrix(4,0) = 0.0;   affMatrix(4,1) = 0.0;   affMatrix(4,2) = 0.0;  // Pose front (0)
    affMatrix(5,0) = 0.0;   affMatrix(5,1) = 1.0;   affMatrix(5,2) = 0.0;  // Pose right (-90)

    // Tool 3 -> rake
    affMatrix(6,0) = 0.0;   affMatrix(6,1) = 0.0;   affMatrix(6,2) = 1.0;  // Pose left  (90)
    affMatrix(7,0) = 0.0;   affMatrix(7,1) = 1.0;   affMatrix(7,2) = 0.0;  // Pose front (0)
    affMatrix(8,0) = 1.0;   affMatrix(8,1) = 0.0;   affMatrix(8,2) = 0.0;  // Pose right (-90)

    // Tool 4 -> stick
    affMatrix(9,0) = 0.0;   affMatrix(9,1) = 1.0;   affMatrix(9,2) = 0.0;  // Pose left  (90)
    affMatrix(10,0) = 0.0;  affMatrix(10,1) = 1.0;  affMatrix(10,2) = 1.0; // Pose front (0)
    affMatrix(11,0) = 1.0;  affMatrix(11,1) = 0.0;  affMatrix(11,2) = 0.0; // Pose right (-90)


    int toolposeI = getAffTP(saveName, toolPose);
    Vector affVector = affMatrix.getRow(toolposeI);

    cout << "Aff vector for tool-pose " << toolposeI << " =  " << affVector.toString() << endl;

    tpAff.clear();
    bool affOK  = false;
    if (affVector[0] > 0.0){
        cout << "drag_left affordable " << endl;
        tpAff.addString("drag_left");
        affOK = true;
    }

    if (affVector[1] > 0.0){
        cout << "drag_diag affordable " << endl;
        tpAff.addString("drag_diag");
        affOK = true;
    }

    if (affVector[2] > 0.0){
        cout << "drag_down affordable " << endl;
        tpAff.addString("drag_down");
        affOK = true;
    }

    if (!affOK){
        cout << "This tool-pose does not afford any of the possible actions." << endl;
        tpAff.addString("no_aff");
    }


    cout << "Affordance bottle is: "<< tpAff.toString() <<endl;

    return affOK;
}

int Objects3DExplorer::getAffTP(const std::string &tool, const yarp::sig::Matrix &pose)
{
    double ori, displ, tilt, shift;
    paramFromPose(pose, ori, displ, tilt, shift);
    cout << "Param returned from paramFromPose to set aff = " << ori << ", " << displ << ", " << tilt << ", " << shift << "." << endl;
    double toolI = 0, poseI = 0;
    if (tool == "pipeHoe3"){
        toolI = 0;
    }
    if (tool == "pipeHook3"){
        toolI = 1;
    }
    if (tool == "rakeGreen"){
        toolI = 2;
    }
    if (tool == "realStick1"){
        toolI = 3;
    }

    cout << "Tool index is: "<< toolI << endl;

    if (ori > 45.0){                        // oriented left
        poseI = 0;
        cout << "Tool oriented left " << endl;
    }else if ((ori < 45.0) && (ori > -45.0)) // oriented front
    {
        poseI = 1;
        cout << "Tool oriented front " << endl;
    }else if (ori < -45.0)                  // oriented right
    {
        cout << "Tool oriented right" << endl;
        poseI = 2;
    }else {
        cout << "Pose out of limits" << endl;
        return -1;
    }

    int tpi = toolI*3 + poseI; // tool-pose index
    cout << "Tool-Pose index is: "<< toolI << endl;

    return tpi;
}


/************************************************************************/
bool Objects3DExplorer::extractFeats()
{
    sendPointCloud(cloud_pose);     // Send the oriented poincloud, TFE should receive it and make it its model.
    Time::delay(0.5);

    // Sends an RPC command to the toolFeatExt module to extract the 3D features of the merged point cloud/
    Bottle cmdTFE, replyTFE;
    cmdTFE.clear();	replyTFE.clear();
    cmdTFE.addString("getFeat");

    rpcFeatExtPort.write(cmdTFE,replyTFE);

    return true;
}


/************************************************************************/
bool Objects3DExplorer::alignWithScale(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_align, Eigen::Matrix4f& transfMat, double minScale, double maxScale, double stepSize)
{
    bool alignOK = false;
    double scale = 1.0;
    int tryI = 0;
    double step = 0.1;
    while (!alignOK)
    {
        cout << "Trying alignment, with scale "<< scale << endl;
        alignOK = alignPointClouds(cloud_source, cloud_target, cloud_align, transfMat);
        if (!alignOK){

            step = -1*getSign(step)* stepSize* tryI;
            scale = scale + step;
            tryI += 1;
            scaleCloud(cloud_source, scale);

            if ((scale > maxScale) || (scale < minScale)){
                cout << "Couldnt align clouds at any given scale"<<endl;
                return false;
            }

        }
    }
    return true;
}

int Objects3DExplorer::getSign(const double x)
{
    if (x >= 0) return 1;
    if (x < 0) return -1;
    return 1;
}


/************************************************************************/
bool Objects3DExplorer::alignPointClouds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_align, Eigen::Matrix4f& transfMat)
{
    Matrix guess;
    poseFromParam(0,0,45,0,guess); // Initial guess to no orientation and tilted 45 degree.
    Eigen::Matrix4f guessEig = CloudUtils::yarpMat2eigMat(guess);

    Eigen::Matrix4f initial_T;
    cloud_align->clear();
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
        sac_ia.align(*cloud_IA, guessEig);

        if (!sac_ia.hasConverged()){
            printf("SAC_IA could not align clouds \n");
            return false;
        }

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

    //ICP algorithm
    // Carefully clean NaNs, as otherwise they make the alignment crash horribly.
    //std::vector <int> nanInd;

    //removeNaNs(cloud_source,cloud_source, nanInd);
    //cout << "Found " << nanInd.size() << " NaNs on source cloud." <<endl;
    //removeNaNs(cloud_target,cloud_target, nanInd);
    //cout << "Found " << nanInd.size() << " NaNs on target cloud." <<endl;

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
    if (!icp.hasConverged()){
        printf("ICP could not fine align clouds \n");
        return false;
    }
    printf("Clouds Aligned! \n");

    if (initAlignment){
        transfMat = icp.getFinalTransformation() * initial_T;
    }else{
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
    fpfh_est.setRadiusSearch(0.01f);
    fpfh_est.compute(*features);
}

bool Objects3DExplorer::checkGrasp(const Matrix &pose)
{
    if(!handFrame) {
        cout << "Grasp can only be checked with cloud referred to hand frame" << endl;
        return false;
    }

    Matrix R = pose.submatrix(0,2,0,2); // Get the rotation matrix
    double rotZ = atan2(R(1,0), R(0,0)) * 180.0/M_PI;                                  // tilt
    double rotY = (atan2(-R(2,0),sqrt(pow(R(2,1),2) + pow(R(2,2),2))))*180.0/M_PI;      // orientation
    double rotX = atan2(R(2,1),R(2,2))*180.0/M_PI;                                      // rotX

    cout << " Rotations estimated: " << endl << "RotX= " << rotX << "RotY= " << rotY << "RotZ= " << rotZ << endl;

    double transX = pose(0,3) * 100.0;      // Displacement along X axis in cm
    double transY = pose(1,3) * 100.0;      // Displacement along Y axis in cm
    double transZ = pose(2,3) * 100.0;      // Displacement along Z axis in cm


    cout << " Translations estimated: " << endl << "TransX= " << transX << "TransY= " << transY << "TransZ= " << transZ << endl;

    if ((fabs(transX) > 10) || (fabs(transY) > 10) || (fabs(transZ) > 10)){
        cout << "Detected translation is over possible grasp" << endl;
        return false;
    }

    /*
    // Limit rotX to -70, 70 (there should be none);
    if ((rotX < -70) || (rotX > 70)){
        cout << "Too much rotation on X" << endl;
        return false;
    }

    // Limit rotY to -135, 135 (should be from -90 to 90)
    if ((rotY < -135) || (rotY > 135)){
        cout << "Too much rotation on Y" << endl;
        return false;
    }

    // Limit rotZ to -15, 105 (should be around 45).
    if ((rotZ < -15) || (rotZ > 105)){
        cout << "Too much rotation on Z" << endl;
        return false;
    }
    */
    return true;
}


void Objects3DExplorer::computeSurfaceNormals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    printf("Computing Surface Normals\n");
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;

    norm_est.setInputCloud(cloud);
    norm_est.setSearchMethod(tree);
    norm_est.setRadiusSearch(0.01f);
    norm_est.compute(*normals);
}


/************************************************************************/
bool Objects3DExplorer::sendPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    Bottle &cloudBottleOut = cloudsOutPort.prepare();
    cloudBottleOut.clear();
    
    CloudUtils::cloud2bottle(cloud, cloudBottleOut);    
       
    if (verbose){printf("Sending out cloud. \n");}
    cloudsOutPort.write();
    return true;

}

/************************************************************************/
bool Objects3DExplorer::addPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Point3D coords, bool shift)
{
   int color[3] = {127, 0, 255};  // Purple by default
   return addPoint(cloud, coords, color, shift);
}

bool Objects3DExplorer::addPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Point3D coords, int color[], bool shift)
{
    cout << "Addint point at " << coords.x << ", " << coords.y << ", " << coords.z << ") " << endl;

    pcl::PointXYZRGB point;
    point.x = coords.x;
    point.y = coords.y;
    point.z = coords.z;
    if (shift){
        point.z -= 0.03;}

    point.r = color[0];
    point.g = color[1];
    point.b = color[2];

    cloud->push_back(point);

    return true;
}


/************************************************************************/
bool Objects3DExplorer::scaleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double scale)
{
    cout << "Changing scaling cloud with scale "<< scale << endl;

    // Find the minimum point of the cloud to and use it to normalize cloud position,
    // so that scaling does not drag towards or away from origin.
    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    pcl::PointXYZRGB min_point_AABB;
    pcl::PointXYZRGB max_point_AABB;
    if (!feature_extractor.getAABB(min_point_AABB, max_point_AABB))
        return false;

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    for (unsigned int i=0; i<cloud->points.size(); i++)
    {
        pcl::PointXYZRGB *point = &cloud->at(i);
        point->x = (point->x - centroid[0]) * scale + centroid[0];
        point->y = (point->y - centroid[1]) * scale + centroid[1];
        point->z = (point->z - centroid[2]) * scale + centroid[2];
    }
    return true;
}


/************************************************************************/
bool Objects3DExplorer::changeCloudColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    int color[3] = {0, 255, 0};    
    return changeCloudColor(cloud, color);
}

bool Objects3DExplorer::changeCloudColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int color[])
{
    cout << "Changing cloud color" << endl;
    for (unsigned int i=0; i<cloud->points.size(); i++)
    {
        pcl::PointXYZRGB *point = &cloud->at(i);
        point->r = color[0];
        point->g = color[1];
        point->b = color[2];

    }
    return true;
}

/************************************************************************/
bool Objects3DExplorer::setToolPose(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const yarp::sig::Matrix &pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudInPose)
{
    cout << "Setting cloud to pose " << endl << pose.toString() << endl;
    Eigen::Matrix4f TM = CloudUtils::yarpMat2eigMat(pose);
    pcl::transformPointCloud(*cloud, *cloudInPose, TM);
    poseFound = true;
    return true;
}

/************************************************************************/
bool Objects3DExplorer::addNoise(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double mean, double sigma)
{
    cout << "Adding noise to cloud" << endl;
    for (unsigned int i=0; i<cloud->points.size(); i++)
    {
        pcl::PointXYZRGB *point = &cloud->at(i);
        point->x = point->x + Random::normal(mean, sigma);
        point->y = point->y + Random::normal(mean, sigma);
        point->z = point->z + Random::normal(mean, sigma);

    }
    return true;
}

/************************************************************************/
bool Objects3DExplorer::changeSaveName(const string& fname)
{
    // Changes the name with which the pointclouds will be saved and read
    saveName = fname;

    Bottle cmdOR, replyOR;
    cmdOR.clear();	replyOR.clear();
    cmdOR.addString("name");
    cmdOR.addString(saveName);
    rpcObjRecPort.write(cmdOR,replyOR);

    Bottle cmdFext, replyFext;
    cmdFext.clear();	replyFext.clear();
    cmdFext.addString("setName");
    cmdFext.addString(saveName + "_merged.ply");
    rpcFeatExtPort.write(cmdFext,replyFext);

    printf("Name changed to %s.\n", saveName.c_str());
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

bool Objects3DExplorer::setSeg(const string& seg)
{
    if (seg == "ON"){
        seg2D = true;
        fprintf(stdout,"Segmentation is 2D");
        return true;
    } else if (seg == "OFF"){
        seg2D = false;
        fprintf(stdout,"Segmentation is 3D");
        return true;
    }
    return false;
}

bool Objects3DExplorer::setSaving(const string& sav)
{
    if (sav == "ON"){
        saving = true;
        cout << "Recorded clouds are being saved at: " << cloudsPathTo <<"/" << saveName << "N" << endl;
        return true;
    } else if (sav == "OFF"){
        saving = false;
        cout << "Recorded clouds NOT being saved." << endl;
        return true;
    }
    return false;
}



bool Objects3DExplorer::setHandFrame(const string& hf)
{
    if (hf == "ON"){
        handFrame = true;
        fprintf(stdout,"Transformation to Hand reference frame is: %s\n", hf.c_str());
        return true;
    } else if (hf == "OFF"){
        handFrame = false;
        fprintf(stdout,"Transformation to Hand reference frame is: %s\n", hf.c_str());
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


