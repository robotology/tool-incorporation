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

/**
 *
@ingroup icub_module
\defgroup icub_objects3DExplorer objects3DExplorer


// This module deals with 3D object/tool exploration, partial reconstruction extraction, alignment, pose estimation, etc.
// - Provides some functions to explore an object in hand.
// - Communicates with toolFeatExt for OMS-EGI feature extraction.
// - Automatizes partial object reconstruction (communicating with obj3Drec), and alignment
// - Performs pose estimation from model using alignment.
// - Computes tooltip from model and estimated pose
**/
  
/**********************************************************
                    PUBLIC METHODS
/**********************************************************/

bool Objects3DExplorer::configure(ResourceFinder &rf)
{
    cout << endl <<"\nInitializing variables... " << endl;

    // External variables -- Modifieable by command line or ini file
    string name = rf.check("name",Value("objects3DExplorer")).asString().c_str();
    robot = rf.check("robot",Value("icub")).asString().c_str();
    string cloudpath_file = rf.check("clouds",Value("cloudsPath.ini")).asString().c_str();
    rf.findFile(cloudpath_file.c_str());

    // Set the path that contains previously saved pointclouds
    if (rf.check("clouds_path")){
        cloudsPathFrom = rf.find("clouds_path").asString().c_str();
    }else{
        string defPathFrom = "/share/ICUBcontrib/contexts/objects3DModeler/sampleClouds/";
        string localModelsPath    = rf.check("local_path")?rf.find("local_path").asString().c_str():defPathFrom;
        string icubContribEnvPath = yarp::os::getenv("ICUBcontrib_DIR");
        cloudsPathFrom  = icubContribEnvPath + localModelsPath;
    }

    // Set the path where new pointclouds will be saved
    string defSaveDir = "/saveModels";
    string cloudsSaveDir = rf.check("save")?rf.find("save").asString().c_str():defSaveDir;
    if (cloudsSaveDir[0]!='/')
        cloudsSaveDir = "/"+cloudsSaveDir;
    cloudsPathTo = "."+cloudsSaveDir;
    yarp::os::mkdir_p(cloudsPathTo.c_str());            // Create the save folder if it didnt exist

    printf("Base path to read clouds from: %s",cloudsPathFrom.c_str());
    printf("Path to save new clouds to: %s",cloudsPathTo.c_str());


    hand = rf.check("hand", Value("right")).asString();
    camera = rf.check("camera", Value("left")).asString();    
    verbose = rf.check("verbose", Value(true)).asBool();

    handFrame = rf.check("handFrame", Value(true)).asBool();            // Sets whether the recorded cloud is automatically transformed w.r.t the hand reference frame
    initAlignment = rf.check("initAlign", Value(true)).asBool();        // Sets whether FPFH initial alignment is used for cloud alignment
    seg2D = rf.check("seg2D", Value(false)).asBool();                   // Sets whether segmentation would be doen in 2D (true) or 3D (false)
    saving = rf.check("saving", Value(true)).asBool();                  // Sets whether recorded pointlcouds are saved or not.
    saveName = rf.check("saveName", Value("cloud")).asString();         // Sets the root name to save recorded clouds

    // Flow control variables
    closing = false;
    numCloudsSaved = 0;
    NO_FILENUM = -1;
    cloudLoaded = false;
    poseFound  = false;
    toolPose = eye(4);
    tooltip.x = 0.0; tooltip.y = 0.0; tooltip.z = 0.0;
    tooltipCanon = tooltip;

    //icp variables
    // XXX eventually make this selectable by .ini or cmd line.
    icp_maxIt = 100;
    icp_maxCorr = 0.03;
    icp_ranORT = 0.05;
    icp_transEp = 1e-6;

    // Noise generation variables
    noise_mean = 0.0;
    noise_sigma = 0.003;


    // Clouds
    cloud_temp = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());  // Point cloud
    cloud_model = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ()); // Point cloud
    cloud_pose = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());  // Point cloud

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

    if (hand=="left"){
        iCartCtrl=iCartCtrlL;
        otherHandCtrl=iCartCtrlR;}
    else if (hand=="right"){
        iCartCtrl=iCartCtrlR;
        otherHandCtrl=iCartCtrlL;}
    else
        return false;

    iGaze->setSaccadesMode(false);
    if (robot == "icubSim"){
            iGaze->setNeckTrajTime(1.5);
            iGaze->setEyesTrajTime(0.5);
            iGaze->blockEyes(5.0);
    }else{
            iGaze->setNeckTrajTime(1.5);
            iGaze->setEyesTrajTime(0.5);
            iGaze->blockEyes(5.0);
    }

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

    //================================= GET MODEL COMMANDS ================================

    if (receivedCmd == "loadCloud"){
            string cloud_file_name = command.get(1).asString();
            cout << "Attempting to load " << (cloudsPathFrom + cloud_file_name).c_str() << "... "<< endl;

            // load cloud to be displayed
            if (!CloudUtils::loadCloud(cloudsPathFrom, cloud_file_name, cloud_model))  {
                std::cout << "Error loading point cloud " << cloud_file_name.c_str() << endl << endl;
                return false;
            }

            cout << "cloud of size "<< cloud_model->points.size() << " points loaded from "<< cloud_file_name.c_str() << endl;

            saveName = cloud_file_name;
            cloudLoaded = true;
            poseFound = false;

            // Display the loaded cloud
            sendPointCloud(cloud_model);
            reply.addString("[ack]");
            return true;


    }else if (receivedCmd == "get3D"){
        // segment object and get the pointcloud using objectReconstrucor module save it in file or array
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rec (new pcl::PointCloud<pcl::PointXYZRGB> ());
        Point2D seed;
        bool ok = getPointCloud(cloud_rec, seed);

        if (ok) {

            if (!cloudLoaded){
                 cloud_model = cloud_rec;
                 cloudLoaded = true;
            }
            sendPointCloud(cloud_rec);

        } else {
            fprintf(stdout,"Couldnt reconstruct pointcloud. \n");
            reply.addString("[nack] Couldnt reconstruct pointcloud. ");
            return false;
        }

        reply.addString("[ack]");
        return true;


    }else if (receivedCmd == "exploreTool"){

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merged (new pcl::PointCloud<pcl::PointXYZRGB> ());
        bool ok = exploreTool(cloud_merged);

        if (ok) {
            sendPointCloud(cloud_merged);
        }

        *cloud_pose = *cloud_merged;
        cloudLoaded = true;
        poseFound = true;

        reply.addString("[ack]");
        return true;


    }else if (receivedCmd == "turnHand"){
            // Turn the hand 'int' degrees, or go to 0 if no parameter was given.
            int rotDegX = 0;
            int rotDegY = 0;
            if (command.size() == 2){
                rotDegY = command.get(1).asInt();
            } else if  (command.size() == 3){
                rotDegY = command.get(1).asInt();
                rotDegX = command.get(2).asInt();
            }

            bool ok = turnHand(rotDegX, rotDegY);
            if (ok){
                reply.addString("[ack]");
                return true;
            } else {
                fprintf(stdout,"Couldnt go to the desired position. \n");
                reply.addString("[nack] Couldnt go to the desired position." );
                return false;
            }


    }else if (receivedCmd == "lookAtTool"){
        bool ok = lookAtTool();
        if (ok){
            reply.addString("[ack]");
            return true;
        } else {
            fprintf(stdout,"Couldnt look at the tool. \n");
            reply.addString("[nack] Couldnt look at the tool." );
            return false;
        }



//================================= POSE COMMANDS ================================

    }else if (receivedCmd == "findPoseAlign"){
        // Check if model is loaded, else return false
        if (!cloudLoaded){
            cout << "Model needed to find Pose. Load model" << endl;
            reply.addString("[nack] Load model first to find grasp. \n");
            return false;
        }

        // Find grasp by comparing partial view with model        
        bool ok = findPoseAlign(cloud_model, cloud_pose, toolPose);

        if (ok){
            reply.addString("[ack]");
            reply.addList().read(toolPose);
            return true;
        }else {
            fprintf(stdout,"Grasp pose could not be obtained. \n");
            reply.addString("[nack] Grasp pose could not be obtained \n");
            return false;
        }

        reply.addString("[ack]");
        return true;


    }else if (receivedCmd == "setPoseParam"){
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

        cout << "Computing pose matrix from params" << endl;

        poseFromParam(ori, disp, tilt, shift, toolPose);     // Get the pose matrix from the parameters

        cout << "Transforming cloud by pose matrix" << endl;
        setToolPose(cloud_model, toolPose, cloud_pose);

        cout << "Sending rotated cloud out of size " <<cloud_pose->size() << endl;
        sendPointCloud(cloud_pose);

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

       Time::delay(1.0);
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

       Time::delay(1.0);
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
       placeTipOnPose(tooltipCanon, poseMatYARP, tooltip);
       cloud_pose->erase(cloud_pose->end()); // Remove last point
       addPoint(cloud_pose, tooltip, true);

       //Display oriented cloud.
       sendPointCloud(cloud_pose);
       Time::delay(1.0);

       cmdVis.clear();	replyVis.clear();
       cmdVis.addString("accumClouds");
       cmdVis.addInt(0);
       rpcVisualizerPort.write(cmdVis,replyVis);

       reply.addString("[ack]");
       reply.addList().read(poseMatYARP);
       return true;



    }else if (receivedCmd == "clearpose"){
        toolPose.resize(4,4);
        toolPose.eye();
        poseFound = false;
        return true;


//================================= TOOLTIP ESTIMATION ================================

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

        int green[3] = {0,255,0};
        addPoint(cloud_model, tooltipCanon,green);
        sendPointCloud(cloud_model);

        reply.addString("[ack]");
        reply.addDouble(tooltipCanon.x);
        reply.addDouble(tooltipCanon.y);
        reply.addDouble(tooltipCanon.z);

        showTooltip(tooltipCanon,green);
        return true;


    }else if (receivedCmd == "findTooltipParam"){   // XXX Function only for simulation

        // Check if model is loaded, else return false
        if (!cloudLoaded){
            cout << "Model needed to find tooltip. Load model" << endl;
            reply.addString("[nack] Load model first to find tooltip.");
            return false;
        }

        // Find tooltip of tool in canonical position
        if(!findTooltipCanon(cloud_model, tooltipCanon)){
            cout << "Could not compute canonical tooltip fom model" << endl;
            reply.addString("[nack] Could not compute canonical tooltip.");
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

        poseFromParam(ori, disp, tilt, shift, toolPose);     // Get the pose matrix from the parameters

        placeTipOnPose(tooltipCanon, toolPose, tooltip);                      // Rotate tooltip with pose

        setToolPose(cloud_model, toolPose, cloud_pose);

        // Display tooltip in oriented cloud
        int green[3] = {0,255,0};
        addPoint(cloud_pose, tooltip,green);
        sendPointCloud(cloud_pose);

        showTooltip(tooltip,green);

        reply.addString("[ack]");
        reply.addDouble(tooltip.x);
        reply.addDouble(tooltip.y);
        reply.addDouble(tooltip.z);
        return true;


    }else if (receivedCmd == "findTooltipAlign"){
        // Check if model is loaded, else return false
        if (!cloudLoaded){
            cout << "Model needed to find tooltip. Load model" << endl;
            reply.addString("[nack] Load model first to find tooltip.");
            return false;
        }

        // Find tooltip of tool in canonical position
        if(!findTooltipCanon(cloud_model, tooltipCanon)){
            cout << "Could not compute canonical tooltip fom model" << endl;
            reply.addString("[nack] Could not compute canonical tooltip.");
            return false;
        }

        // Check if pose has been found, else return false
        if (!poseFound){
            cout << "Pose needed to estimate tooltip, please call 'findPoseAlign' or 'setPoseParam' first." << endl;
            reply.addString("[nack] Pose must be known first to find tooltip.");
            return false;
        }

        // Rotate tooltip to given toolPose
        if(!placeTipOnPose(tooltipCanon, toolPose, tooltip)){
            cout << "Could not compute tooltip from the canonical one and the pose matrix" << endl;
            reply.addString("[nack] Could not compute tooltip.");
            return false;
        }

        // Add point to indicate tooltip.
        int green[3] = {0,255,0};
        addPoint(cloud_model, tooltipCanon, green);
        sendPointCloud(cloud_model);

        // Rotate canonical cloud to found pose
        cout << "Transforming the model with pose" << endl;
        setToolPose(cloud_model, toolPose, cloud_pose);
        cloud_pose->erase(cloud_pose->end()); // Remove last point
        addPoint(cloud_pose, tooltip, true);
        Time::delay(1.0);
        sendPointCloud(cloud_pose);

        double ori, displ, tilt, shift;
        paramFromPose(toolPose, ori, displ, tilt, shift);

        cout << "Param returned from paramFromPose = " << ori << ", " << displ << ", " << tilt << ", " << shift << "." << endl;

        showTooltip(tooltip,green);

        reply.addString("[ack]");
        reply.addDouble(tooltip.x);
        reply.addDouble(tooltip.y);
        reply.addDouble(tooltip.z);
        reply.addDouble(ori);
        reply.addDouble(displ);
        reply.addDouble(tilt);
        reply.addDouble(shift);
        return true;


    }else if (receivedCmd == "findTooltipSym"){
        // Check if model is loaded, else return false
        if (!cloudLoaded){
            cout << "Model needed to find tooltip. Load model" << endl;
            reply.addString("[nack] Load model first to find tooltip.");
            return false;
        }

        // Check if pose has been found, else return false
        int green[3] = {0,255,0};
        if (!poseFound){
            if(!findTooltipSym(cloud_model, tooltip)){
                cout << "Could not compute tooltip from canonical cloud" << endl;
                reply.addString("[nack] Could not compute tooltip.");
                return false;
            }
            Time::delay(0.5);
            addPoint(cloud_model, tooltip,green);
            sendPointCloud(cloud_model);
        }else {
            if(!findTooltipSym(cloud_pose, tooltip)){
                cout << "Could not compute tooltip from cloud in pose" << endl;
                reply.addString("[nack] Could not compute tooltip.");
                return false;
            }
            Time::delay(0.5);
            addPoint(cloud_pose, tooltip,green);
            sendPointCloud(cloud_pose);
        }

        cout << "Tooltip found at ( " << tooltip.x <<  ", " << tooltip.y <<  ", "<< tooltip.z <<  "). " << endl;
        showTooltip(tooltip,green);

        reply.addString("[ack]");
        reply.addDouble(tooltip.x);
        reply.addDouble(tooltip.y);
        reply.addDouble(tooltip.z);

        return true;

    }else if (receivedCmd == "cleartip"){
        tooltip.x == 0.0;
        tooltip.y == 0.0;
        tooltip.z == 0.0;

        tooltipCanon = tooltip;
        return true;

// ======================= AFFORDANCES ================================

    }else if (receivedCmd == "getAffordance"){

        if (!(poseFound)&&(cloudLoaded))
        {
            cout << "Tool and pose need to estimate affordance of tool-pose. Load tool and find pose." << endl;
            reply.addString("[nack] Tool-pose must be known to predict affordances.");
            return false;
        }

        Bottle aff;
        bool all = false;
        if (command.size() > 1){
            all =  command.get(1).asBool();
        }

        bool ok;
        ok = getAffordances(aff, all);
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



// =======================  PARAMETER SET  =======================

    }else if (receivedCmd == "handFrame"){
        // activates the normalization of the pointcloud to the hand reference frame.
        bool ok = setHandFrame(command.get(1).asString());
        if (ok){
            reply.addString("[ack]");
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
            reply.addString("[ack]");
            return true;}
        else {
            fprintf(stdout,"FPFH based Initial Alignment has to be set to ON or OFF. \n");
            reply.addString("[nack] FPFH based Initial Alignment has to be set to ON or OFF. ");
            reply.addString("[ack]");
            return false;
        }


    }else if (receivedCmd == "icp"){
        // icp -> sets parameters for iterative closest point aligning algorithm        
        icp_maxIt = command.get(1).asInt();
        icp_maxCorr = command.get(2).asDouble();
        icp_ranORT = command.get(3).asDouble();
        icp_transEp = command.get(4).asDouble();
        cout << " icp Parameters set to " <<  icp_maxIt << ", " << icp_maxCorr << ", " << icp_ranORT<< ", " << icp_transEp << endl;
        reply.addString("[ack]");
        return true;


    }else if (receivedCmd == "noise"){
        // noise -> sets parameters for noise addition for align test
        noise_mean = command.get(1).asDouble();
        noise_sigma = command.get(2).asDouble();
        cout << "Noise Parameters set to mean:" <<  noise_mean << ", sigma: " << noise_sigma << endl;
        reply.addString("[ack]");
        return true;


    }else if (receivedCmd == "seg2D"){
        bool ok = setSeg(command.get(1).asString());
        if (ok){
            reply.addString("[ack]");
            return true;
        }
        else {
            fprintf(stdout,"2D Segmentation can only be set to ON or OFF. \n");
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
            reply.addString("[ack]");
            return true;
        }else {
            fprintf(stdout,"Couldnt change the name. \n");
            reply.addString("[nack] Couldnt change the name. ");
            return false;
        }


    }else if (receivedCmd == "saving"){
        // changes whether the reconstructed clouds will be saved or not.
        bool ok = setSaving(command.get(1).asString());
        if (ok){
            reply.addString("[ack]");
            return true;
        }else {
            fprintf(stdout,"Verbose can only be set to ON or OFF. \n");
            reply.addString("[nack] Verbose can only be set to ON or OFF.");
            return false;
        }

    }else if (receivedCmd == "verbose"){
        bool ok = setVerbose(command.get(1).asString());
        if (ok){
            reply.addString("[ack]");
            return true;
        }
        else {
            fprintf(stdout,"Verbose can only be set to ON or OFF. \n");
            reply.addString("[nack] Verbose can only be set to ON or OFF.");
            return false;
        }


	}else if (receivedCmd == "help"){
		reply.addVocab(Vocab::encode("many"));
		responseCode = Vocab::encode("ack");
        reply.addString("Available commands are:");

        reply.addString("---------- GET MODEL -----------");
        reply.addString("loadCloud - Loads a cloud from a file (.ply or .pcd).");
        reply.addString("get3D - segment object and get the pointcloud using objectReconstrucor module.");
        reply.addString("exploreTool - automatically gets 3D pointcloud from different perspectives and merges them in a single model.");
        reply.addString("turnHand  (int)X (int)Y- moves arm to home position and rotates hand 'int' X and Y degrees around the X and Y axis  (0,0 by default).");
        reply.addString("lookAtTool - Moves gaze to look where the tooltip is (or a guess if it is not defined).");

        reply.addString("---------- GET POSE -----------");
        reply.addString("findPoseAlign - Find the actual grasp by comparing the actual registration to the given model of the tool.");
        reply.addString("setPoseParam [ori][disp][tilt][shift] - Set the tool pose given the grasp parameters.");
        reply.addString("alignFromFiles (sting)part (string)model - merges cloud 'part' to cloud 'model' from .ply/.pcd files (test for aligning algorithms).");
        reply.addString("clearpose - Removes any previously estimated pose.");

        reply.addString("---------- TOOLTIP ESTIMATION -----------");
        reply.addString("findTooltipCanon - Finds the tooltip of the tool in its canonical position -MODEL REQUIRED-.");
        reply.addString("findTooltipParam [ori][disp][tilt][shift]- Finds the tooltip of the tool in the position given by the parameters -MODEL REQUIRED-.");
        reply.addString("findTooltipAlign - Places the tooltip on the rotated tool after pose has been found -MODEL and POSE required-.");
        reply.addString("findTooltipSym - Finds the tooltip of the tool in any position based on symmetry planes.");
        reply.addString("cleartip - Removes any previously estimated tooltip.");

        reply.addString("---------- AFFORDANCES ------------");
        reply.addString("getAffordance - returns the pre-learnt affordances for the loaded tool-pose.");
        reply.addString("extractFeats - Sends the oriented model out and calls TFE to compute features.");

        reply.addString("---------- SET PARAMETERS ------------");
        reply.addString("handFrame (ON/OFF) - Activates/deactivates transformation of the registered clouds to the hand coordinate frame. (default ON).");
        reply.addString("FPFH (ON/OFF) - Activates/deactivates fast local features (FPFH) based Initial alignment for registration. (default ON).");
        reply.addString("icp (int)maxIt (double)maxCorr (double)ranORT (double)transEp - sets ICP parameters (default 100, 0.03, 0.05, 1e-6).");
        reply.addString("noise (double)mean (double)sigma - sets noise parameters (default 0.0, 0.003)");
        reply.addString("seg2D (ON/OFF) - Set the segmentation to 2D (ON) from graphBasedSegmentation, or 3D (OFF), from 'flood3d' .");
        reply.addString("savename (string) - Changes the name with which the pointclouds will be saved.");
        reply.addString("saving (ON/OFF) - Controls whether recorded clouds are saved or not.");
        reply.addString("verbose (ON/OFF) - Sets ON/OFF printouts of the program, for debugging or visualization.");
        reply.addString("help - produces this help.");
		reply.addString("quit - closes the module.");

		reply.addVocab(responseCode);
		return true;

    } else if (receivedCmd == "quit"){
        reply.addString("[ack]");
		closing = true;
		return true;
	}
    
    reply.addString("Invalid command, type [help] for a list of accepted commands.");    
    return true;

}


/**********************************************************
                    PROTECTED METHODS
/**********************************************************/

/************************************************************************/
bool Objects3DExplorer::turnHand(const int rotDegX, const int rotDegY, const bool followTool)
{

    if ((rotDegY > 70 ) || (rotDegY < -70) || (rotDegX > 90 ) || (rotDegX < -90) )	{
        printf("Rotation out of operational limits. \n");
        return false;
    }

    int context_arm;
    int context_eye;

    iCartCtrl->storeContext(&context_arm);
    if (followTool){
        iGaze->storeContext(&context_eye);
    }

    // intialize position and orientation matrices
    Matrix Rh(4,4);
    Rh(0,0)=-1.0;         Rh(2,1)=-1.0;         Rh(1,2)=-1.0;         Rh(3,3)=+1.0;
    //Vector r(4,0.0);
    Vector xd(3,0.0);
    Vector offset(3,0.0);;

    // set base position
    xd[0]=-0.25;
    xd[1]=(hand=="left")?-0.1:0.1;					// move sligthly out of center towards the side of the used hand
    xd[2]= 0.05;

    offset[0]=0;
    offset[1]=(hand=="left")?-0.05-(0.01*(rotDegY/10+rotDegX/3)):0.0 + (0.01*(rotDegY/10+rotDegX/3));	// look slightly towards the side where the tool is rotated
    offset[2]= 0.15 - 0.01*abs(rotDegX)/5;

    // Rotate the hand to observe the tool from different positions
    Vector ox(4), oy(4);
    ox[0]=1.0; ox[1]=0.0; ox[2]=0.0; ox[3]=(M_PI/180.0)*(hand=="left"?-rotDegX:rotDegX); // rotation over X axis
    oy[0]=0.0; oy[1]=1.0; oy[2]=0.0; oy[3]=(M_PI/180.0)*(hand=="left"?-rotDegY:rotDegY); // rotation over Y axis

    Matrix Ry=axis2dcm(oy);    // from axis/angle to rotation matrix notation
    Matrix Rx=axis2dcm(ox);
    Matrix R=Rh*Rx*Ry;         // compose the two rotations keeping the order
    Vector od=dcm2axis(R);     // from rotation matrix back to the axis/angle notation


    iCartCtrl->goToPoseSync(xd,od,1.0);

    /********************************/
    // Look at the desired tool position

    R(0,3)= xd[0];    R(1,3)= xd[1];    R(2,3)= xd[2];        // Include translation

    // Define the tooltip initial guess w.r.t to hand frame:
    Vector xTH, xTR;           // Position of an estimated tooltip (Hand and Robot referenced)
    xTH.resize(4);

    if ((tooltip.x == 0) && (tooltip.y == 0) && (tooltip.z == 0)){
        // If tooltip has not been initialized, try a generic one (0.17, -0.17, 0)
        xTH[0] = 0.17;              // X
        xTH[1] = -0.17;             // Y
        xTH[2] = 0.0;               // Z
        xTH[3] = 1.0;               // T
    }else {
        xTH[0] = tooltip.x;         // X
        xTH[1] = tooltip.y;         // Y
        xTH[2] = tooltip.z;         // Z
        xTH[3] = 1.0;               // T
    }

    // Transform point to robot coordinates:
    xTR = R * xTH;
    cout << "Initial guess for the tool is at coordinates (" << xTR[0] << ", "<< xTR[1] << ", "<< xTR[2] << ")." << endl;

    iGaze->blockEyes(5.0);
    iGaze->lookAtFixationPoint(xTR);
    iGaze->waitMotionDone(0.1);

    iCartCtrl->waitMotionDone(0.1);
    iCartCtrl->restoreContext(context_arm);
    iCartCtrl->deleteContext(context_arm);

    return true;
}

/************************************************************************/
bool Objects3DExplorer::lookAtTool(){
    // Uses the knowledge of the kinematics of the arm to look on the direction of where the tool should be.
    // The hand reference frame stays on the lower part of the image, while the orientation depends on the -Y axis.

    Vector xH,oH;                                                   // Pose of the hand ref. frame
    iCartCtrl->getPose(xH,oH);

    // Get transformation matrix
    Matrix H2R=axis2dcm(oH);                                        // from axis/angle to rotation matrix notation
    H2R(0,3)= xH[0];    H2R(1,3)= xH[1];    H2R(2,3)= xH[2];        // Include translation

    // Define the tooltip initial guess w.r.t to hand frame:
    Vector xTH, xTR;           // Position of an estimated tooltip (Hand and Robot referenced)
    xTH.resize(4);

    if ((tooltip.x == 0) && (tooltip.y == 0) && (tooltip.z == 0)){
        // If tooltip has not been initialized, try a generic one (0.17, -0.17, 0)
        xTH[0] = 0.17;              // X
        xTH[1] = -0.17;             // Y
        xTH[2] = 0.0;               // Z
        xTH[3] = 1.0;               // T
    }else {
        xTH[0] = tooltip.x;         // X
        xTH[1] = tooltip.y;         // Y
        xTH[2] = tooltip.z;         // Z
        xTH[3] = 1.0;               // T
    }

    // Transform point to robot coordinates:
    xTR = H2R * xTH;
    cout << "Initial guess for the tool is at coordinates (" << xTR[0] << ", "<< xTR[1] << ", "<< xTR[2] << ")." << endl;

    iGaze->blockEyes(5.0);
    iGaze->lookAtFixationPoint(xTR);
    iGaze->waitMotionDone();

    return true;
}

bool Objects3DExplorer::exploreTool(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rec_merged)
{
    // Rotates the tool in hand, gets successive partial reconstructions and returns a merge-> cloud_model
    cloud_rec_merged->points.clear();
    Point2D seed;

    // Move not exploring hand out of the way:
    Vector away;
    away.resize(3);
    away[0] = -0.05;
    away[1] = (hand=="left")?0.2:-0.2;
    away[2] = -0.1;

    otherHandCtrl->goToPose(away);


    // Get inital cloud model on central orientation
    turnHand(0,0);
    //lookAtTool();
    bool ok = false;
    while(!ok){          // Keep on getting clouds until one is valid (should be the first)
        ok = getPointCloud(cloud_rec_merged, seed);
        lookAround();
    }
    sendPointCloud(cloud_rec_merged);

    cout << " ====================================== Starting Exploration  =======================================" <<endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rec (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());
    Eigen::Matrix4f alignMatrix;
    Eigen::Matrix4f poseMatrix;

    // Rotate tool in hand
    bool mergeAlign = true;            // XXX make rpc selectable
    int minX = -70, maxX = 70;
    int minY = -40, maxY = 60;
    for (int degX = minX; degX<=maxX; degX += 40)
    {
        cout << endl << endl << " +++++++++++ EXPLORING NEW ANGLE " << degX << " ++++++++++++++++++++" << endl <<endl;
        // Move hand to new position
        turnHand(degX,minY);
        //lookAtTool();
        Time::delay(1.0);

        // Get partial reconstruction
        cloud_rec->points.clear();
        cloud_rec->clear();

        // If cloud was found by any of the prrevious methods
        if(getPointCloud(cloud_rec, seed)){
            if (!mergeAlign){
                // Add clouds without aligning (aligning is implicit because they are all transformed w.r.t the hand reference frame)
                *cloud_rec_merged += *cloud_rec;
            }else{
                // Align new reconstructions to model so far.
                alignWithScale(cloud_rec, cloud_rec_merged, cloud_aligned, alignMatrix, 0.7, 1.3);
                poseMatrix = alignMatrix.inverse();             // Inverse the alignment to find tool pose
                Matrix pose = CloudUtils::eigMat2yarpMat(poseMatrix);  // transform pose Eigen matrix to YARP Matrix
                bool poseValid = checkGrasp(pose);

                if (poseValid)
                    *cloud_rec_merged += *cloud_aligned;
            }

            // Downsample to reduce size and fasten computation
            downsampleCloud(cloud_rec_merged, cloud_rec_merged, 0.002);
            cout << " Cloud at angle X" << degX << " reconstructed " << endl;
            sendPointCloud(cloud_rec_merged);

        } else {
            cout << " Could not reconstruct the cloud" << endl;
        }
    }

    cout << endl << " + + FINISHED X ROTATION + + " << endl <<endl;


    // Rotate tool in hand
    for (int degY = minY; degY<=maxY; degY += 25)
    {
        cout << endl << endl << " +++++++++++ EXPLORING NEW ANGLE " << degY << " ++++++++++++++++++++" << endl <<endl;
        // Move hand to new position
        turnHand(0,degY);
        //lookAtTool();
        Time::delay(1.0);

        // Get partial reconstruction
        cloud_rec->points.clear();
        cloud_rec->clear();

        // If cloud was found by any of the prrevious methods
        if(getPointCloud(cloud_rec, seed)){
            if (!mergeAlign){
                // Add clouds without aligning (aligning is implicit because they are all transformed w.r.t the hand reference frame)
                *cloud_rec_merged += *cloud_rec;
            }else{
                // Align new reconstructions to model so far.
                alignWithScale(cloud_rec, cloud_rec_merged, cloud_aligned, alignMatrix, 0.7, 1.3);
                poseMatrix = alignMatrix.inverse();             // Inverse the alignment to find tool pose
                Matrix pose = CloudUtils::eigMat2yarpMat(poseMatrix);  // transform pose Eigen matrix to YARP Matrix
                bool poseValid = checkGrasp(pose);

                if (poseValid)
                    *cloud_rec_merged += *cloud_aligned;
            }

            // Downsample to reduce size and fasten computation
            downsampleCloud(cloud_rec_merged, cloud_rec_merged, 0.002);
            cout << " Cloud at angle Y" << degY << " reconstructed " << endl;
            sendPointCloud(cloud_rec_merged);

        } else {
            cout << " Could not reconstruct the cloud" << endl;
        }
    }
    cout << endl << " + + FINISHED Y ROTATION + + " << endl <<endl;

}


/************************************************************************/
bool Objects3DExplorer::lookAround()
{
    Vector fp,fp_aux(3,0.0);
    iGaze->getFixationPoint(fp);
    printf("Looking at %.2f, %.2f, %.2f \n", fp[0], fp[1], fp[2] );
    fp_aux[0] = fp[0] + Rand::scalar(-0.02,0.02);
    fp_aux[1] = fp[1] + Rand::scalar(-0.04,0.04);
    fp_aux[2] = fp[2] + Rand::scalar(-0.02,0.02);
    iGaze->lookAtFixationPoint(fp_aux);
    iGaze->waitMotionDone(0.05);

    return true;
}


/* CLOUD INFO */
/************************************************************************/
bool Objects3DExplorer::getPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rec, Point2D& seed)
{
    cloud_rec->points.clear();
    cloud_rec->clear();   // clear receiving cloud

    // requests 3D reconstruction to objectReconst module
    Bottle cmdOR, replyOR;
    cmdOR.clear();	replyOR.clear();
    if (seg2D){
        cmdOR.addString("seg");
    } else {
        cmdOR.addString("flood3d");        
    }
    rpcObjRecPort.write(cmdOR,replyOR);

    seed.u= replyOR.get(1).asInt();
    seed.v= replyOR.get(2).asInt();

    // read the cloud from the objectReconst output port
    Bottle *cloudBottle = cloudsInPort.read(true);
    if (cloudBottle!=NULL){
        if (verbose){	cout << "Bottle of size " << cloudBottle->size() << " read from port \n"	<<endl;}
        CloudUtils::bottle2cloud(*cloudBottle,cloud_rec);
    } else{
        if (verbose){	printf("Couldnt read returned cloud \n");	}
        return false;
    }

    // Transform the cloud's frame so that the bouding box is aligned with the hand coordinate frame
    if (handFrame) {
        frame2Hand(cloud_rec, cloud_rec);
    }

    // Apply some filtering to clean the cloud
    // Process the cloud by removing distant points ...
    pcl::PassThrough<pcl::PointXYZRGB> passX;
    passX.setInputCloud (cloud_rec);
    passX.setFilterFieldName ("x");
    passX.setFilterLimits (0.0, 0.35);
    passX.filter (*cloud_rec);

    pcl::PassThrough<pcl::PointXYZRGB> passY;
    passY.setInputCloud (cloud_rec);
    passY.setFilterFieldName ("y");
    passY.setFilterLimits (-0.3, 0.0);
    passY.filter (*cloud_rec);

    pcl::PassThrough<pcl::PointXYZRGB> passZ;
    passZ.setInputCloud (cloud_rec);
    passZ.setFilterFieldName ("z");
    passZ.setFilterLimits (-0.1, 0.1);
    passZ.filter (*cloud_rec);

     // ... and removing outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; //filter to remove outliers
    sor.setStddevMulThresh (3.0);
    sor.setInputCloud (cloud_rec);
    sor.setMeanK(10);
    sor.filter (*cloud_rec);

    // Remove hand (all points within 6 cm from origin)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_nohand (new pcl::PointCloud<pcl::PointXYZRGB> ());
    if (handFrame) {
        double handRad = 0.06;
        pcl::PointIndices::Ptr pointsTool (new pcl::PointIndices ());
        for (unsigned int ptI=0; ptI<cloud_rec->points.size(); ptI++)
        {
            pcl::PointXYZRGB *pt = &cloud_rec->at(ptI);
            double distP = sqrt(pt->x*pt->x+pt->y*pt->y+pt->z*pt->z);           // Compute distance around hand reference frame
            if (distP > handRad){
                pointsTool->indices.push_back(ptI);
            }
        }

        pcl::ExtractIndices<pcl::PointXYZRGB> eifilter (true);
        eifilter.setInputCloud (cloud_rec);
        eifilter.setIndices(pointsTool);
        eifilter.filter(*cloud_nohand);

        cloud_rec->points.clear();
        *cloud_rec = *cloud_nohand;          //overwrite cloud with filtered one with no points fo hand.
    }


    scaleCloud(cloud_rec,1.1);

    if (cloud_rec->size() < 100){
        cout << " Not enough points left after filtering. Something must have happened on reconstruction" << endl;
        return false;
    }


    if (verbose){ cout << " Cloud of size " << cloud_rec->points.size() << " obtained from 3D reconstruction" << endl;}

    if (saving){
        CloudUtils::savePointsPly(cloud_rec, cloudsPathTo, saveName, numCloudsSaved);}

    return true;
}


/************************************************************************/
bool Objects3DExplorer::findPoseAlign(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr poseCloud, Matrix &pose)
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

        Time::delay(1.0);
        sendPointCloud(modelCloud);
        Time::delay(1.0);

        // Get a registration
        Point2D seed;
        if(!getPointCloud(cloud_rec,seed))          // Registration get and normalized to hand-reference frame.
        {
            cout << " Cloud not valid" << endl;
            continue;
        }
        int blue[3] = {0,0,255};               // Plot oriented model green
        changeCloudColor(cloud_rec, blue);
        sendPointCloud(cloud_rec);
        Time::delay(1.0);

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
        Time::delay(1.0);

        if (!poseValid) {
            cout << "The estimated grasp is not possible, retry with a new pointcloud" << endl;
        }

        trial++;  // to limit number of trials
        if (trial > 10){
            cout << "Could not find a valid grasp in 10 trials" << endl;


            cmdVis.clear();	replyVis.clear();
            cmdVis.addString("accumClouds");
            cmdVis.addInt(0);
            rpcVisualizerPort.write(cmdVis,replyVis);

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
    ttCanon.x = max_point_AABB.x;//-effLength/3;                        // tooltip not on the extreme, but sligthly in  -  x coord of ttCanon
    ttCanon.y = min_point_AABB.y;                                       // y coord of ttCanon
    ttCanon.z = (max_point_AABB.z + min_point_AABB.z)/2;                // z coord of ttCanon

    cout << "Canonical tooltip at ( " << ttCanon.x << ", " << ttCanon.y << ", " << ttCanon.z <<")." << endl;    
    return true;
}


bool Objects3DExplorer::placeTipOnPose(const Point3D &ttCanon, const Matrix &pose, Point3D &tooltipTrans)
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

/*************************************************************************/
bool Objects3DExplorer::findTooltipSym(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_raw,  Point3D& ttSym, int K)
{

    // Cloud can be strongly downsamlped to incrase speed in computation, shouldnt change much the results.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    downsampleCloud(cloud_raw, cloud, 0.005);

    // 1- Find the Major axes of the cloud -> Find major planes as normal to those vectors
    pcl::MomentOfInertiaEstimation <pcl::PointXYZRGB> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    // Find major axes as eigenVectors
    vector< Eigen::Vector3f> eigenVectors(3);               // Normals to the 3 main planes
    Eigen::Vector3f eigenValues(3);                            // Relative legth in each eigenVector direction.
    yarp::sig::Vector eVs(3);
    Eigen::Vector3f mc;                                    // Center of mass
    feature_extractor.getEigenValues(eigenValues[0], eigenValues[1], eigenValues[2]);
    eVs[0]= eigenValues[0];     eVs[1]= eigenValues[1];     eVs[2]= eigenValues[2];
    feature_extractor.getEigenVectors(eigenVectors[0], eigenVectors[1], eigenVectors[2]);
    feature_extractor.getMassCenter(mc);

    int symPlane_i= -1;
    //Plane3D symPlane;
    vector<Plane3D> mainPlanes;
    vector<Plane3D> unitPlanes;
    float minSymDist = 1e9;
    // Find major planes as normal to those vectors
    for (int plane_i = 0; plane_i < eigenVectors.size(); plane_i ++){
        // Compute coefficients of plane equation ax + by + cz + d = 0
        // float a, b, c, d;
        Plane3D P;
        P.a = eigenVectors[plane_i](0);
        P.b = eigenVectors[plane_i](1);
        P.c = eigenVectors[plane_i](2);
        P.d = -P.a*mc(0)-P.b*mc(1)-P.c*mc(2);

        mainPlanes.push_back(P);

    // 2- Compute symmetry coeffcients w.r.t each of the planes -> Select symmetry plane as one with higher symCoeff
        pcl::PointIndices::Ptr pointsA (new pcl::PointIndices ());
        pcl::PointIndices::Ptr pointsB (new pcl::PointIndices ());
        // Loop through all the points in the cloud and select which side of the plane they belong to.
        for (unsigned int ptI=0; ptI<cloud->points.size(); ptI++)
        {
            pcl::PointXYZRGB *pt = &cloud->at(ptI);
            if (P.a*pt->x + P.b*pt->y + P.c*pt->z + P.d > 0){
                pointsA->indices.push_back(ptI);
            } else {
                pointsB->indices.push_back(ptI);
            }
        }

        // Split cloud into 2 point vectors (at each side of the plane_i).
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudAB (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudMirror (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::ExtractIndices<pcl::PointXYZRGB> eifilter (true); // Initializing with true will allow us to extract the removed indices
        eifilter.setInputCloud (cloud);
        eifilter.setIndices(pointsA);
        eifilter.filter(*cloudA);

        eifilter.setIndices(pointsB);
        eifilter.filter(*cloudB);

        cout << "The original cloud of size " << cloud->size() << " is divided in two clouds of size " << cloudA->size() << " and " << cloudB->size() << ". "<< endl;

        // Mirror one of the vectors wrt the plane_i.
        Plane3D uP;     // Unit plane
        double M = sqrt(P.a*P.a+P.b*P.b+P.c*P.c);    // Elements of normal unit vector
        uP.a = P.a/M;
        uP.b = P.b/M;
        uP.c = P.c/M;
        uP.d = P.d/M;
        unitPlanes.push_back(uP);
        cloudMirror->clear();

        for (unsigned int ptI=0; ptI<cloudB->points.size(); ptI++)
        {
            pcl::PointXYZRGB *pt = &cloudB->at(ptI);
            pcl::PointXYZRGB ptMirror;
            float dn = uP.a*pt->x + uP.b*pt->y + uP.c*pt->z + uP.d;   // Normalized signed distance of point to plane_i
            ptMirror.x = pt->x - 2*(uP.a*dn);
            ptMirror.y = pt->y - 2*(uP.b*dn);
            ptMirror.z = pt->z - 2*(uP.c*dn);
            cloudMirror->push_back(ptMirror);
        }

        // compute avg distance between mirrored vector and the other one.
        pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
        kdtree.setInputCloud(cloudA);
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        float accumCloudKNNdist = 0.0;
        float avgCloudKNNdist = 0.0;
        int valPt = 0;
        for (unsigned int ptI=0; ptI<cloudMirror->points.size(); ptI++)
        {
            pcl::PointXYZRGB *pt = &cloudMirror->at(ptI);

            // compute NN in cloudA to each point in cloudMirror and average.
            float avgPointKNNdist = 0.0;
            if ( kdtree.nearestKSearch (*pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            {
                float accumPointKNNdist = 0.0;
                for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i){
                    accumPointKNNdist += pointNKNSquaredDistance[i];
                }
                avgPointKNNdist = accumPointKNNdist/ pointIdxNKNSearch.size();
                valPt ++;
            }
            accumCloudKNNdist += avgPointKNNdist;
        }

        // normalize
        avgCloudKNNdist = accumCloudKNNdist/valPt;

        // merge cloud A and B in cloud AB
        int blue[3] = {0,0,255};
        changeCloudColor(cloudB, blue );
        *cloudAB = *cloudA;
        *cloudAB += *cloudB;
        sendPointCloud(cloudAB);
        Time::delay(2.5);
        cout << "Average  distance between two sides of the symmetry plane " << plane_i << " is " << sqrt(avgCloudKNNdist) << endl;


        if (sqrt(avgCloudKNNdist) < minSymDist){
            //symPlane = P;
            symPlane_i = plane_i;
            minSymDist = sqrt(avgCloudKNNdist);
        }
    }
    // Check that a symmetry plane has been found
    if (symPlane_i <0 ){
        cout << "The symmetry plane could not be found" << endl;
        return false;
    }
    cout << "The symmetry plane is plane " << symPlane_i << endl;


    // 3-  Tooltip is the furthest point on a weighted sum of distance to origin (small weight) and effector plane (large weight)

    // find the effector plane: shortest eigenvector of the 2 remaning ones (excluding the symmery plane eigenvector).
    eVs[symPlane_i]= 1e09;          // Make the value of symmetrical plane huge;
    float effEV = 1e08;
    int effPlane_i= -1;
    for (int eV = 0; eV< eVs.size();eV++)
    {
        if ((eVs[eV] < effEV) && (eV != symPlane_i)) {      // min egenvalue not of symmetry plane
            effEV = eVs[eV];
            effPlane_i = eV;
        }
    }

    if ((effPlane_i < 0 )) {
        cout << "There was some error finding effector planes" << endl;
        return false;
    }
    cout << "The effector is perpendicular to plane " << effPlane_i << endl;

    // Find furthest point along effector eigenvector.
    Plane3D effP = unitPlanes[effPlane_i];
    double maxDist = 0.0;
    int maxPt_i = -1;
    double w = 0.8;     // Weight assigned to effector distance w.r.t. orig distance
    for (unsigned int ptI=0; ptI<cloud->points.size(); ptI++)
    {
        pcl::PointXYZRGB *pt = &cloud->at(ptI);
        float dist_eff = fabs(effP.a*pt->x + effP.b*pt->y + effP.c*pt->z + effP.d);   // Normalized signed distance of point to eff plane
        float dist_orig = sqrt(pt->x*pt->x + pt->y*pt->y + pt->z*pt->z);              // Distance from the origin.
        float dist_aux = w*dist_eff + (1-w)*dist_orig;
        if (dist_aux > maxDist){
            maxDist = dist_aux;
            maxPt_i = ptI;
        }
    }
    if (maxPt_i < 0 ){
        cout << "There was some error finding furthest point" << endl;
        return false;
    }

    // Project point on symmetry plane
    pcl::PointXYZRGB *maxPt = &cloud->at(maxPt_i);
    Plane3D sP = unitPlanes[symPlane_i];
    float dn = sP.a*maxPt->x + sP.b*maxPt->y + sP.c*maxPt->z + sP.d;   // Normalized signed distance of point to plane_i
    ttSym.x = maxPt->x - (sP.a*dn);
    ttSym.y = maxPt->y - (sP.b*dn);
    ttSym.z = maxPt->z - (sP.c*dn);

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

    Vector oy(4);   // define the rotation over the -Y axis or effector orientation
    oy[0]=0.0; oy[1]=-1.0; oy[2]=0.0; oy[3]= radOr; // tool is along the -Y axis!!
    Matrix R_ori = axis2dcm(oy);          // from axis/angle to rotation matrix notation
    Vector oz(4);   // define the rotation over the Z axis for tilt.
    oz[0]=0.0; oz[1]=0.0; oz[2]=1.0; oz[3]= radTilt; // tilt around the Z axis!!
    Matrix R_tilt = axis2dcm(oz);

    pose = R_tilt*R_ori;         // Compose matrices in order, first rotate around -Y (R_ori), then around Z

    pose(1,3) = -disp /100.0;   // This accounts for the traslation of 'disp' in the -Y axis in the hand coord system along the extended thumb).
    pose(2,3) = shift/100.0;   // This accounts for the Z translation to match the tooltip


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
bool Objects3DExplorer::getAffordances(Bottle &affBottle, bool allAffs)
{
    cout << "Computing affordances of the tool-pose in hand " << endl;
    int rows = 12;
    int cols = 4;
    Matrix affMatrix(rows,cols);
    // affMatrix contains the pre-learnt affordances of the 4 possible tools. (can be easily extended for new tools).
    // each row is corresponds to a tool-pose, each column (p) to a possible action (a).
    // Thus, tools are represented in groups of 3 rows, corresponding to poses : left, front, right.
    // The value on (p,a) represents if the action 'a' can be achieved with tool-pose 'p', and therfore is boolean (could also be extended to percentage).

    //    Drag Left           Drag Diagonal left        Drag down            Drag diagonal right
    // Tool 0 -> hoe
    affMatrix(0,0) = 1.0;   affMatrix(0,1) = 1.0;   affMatrix(0,2) = 0.0;   affMatrix(0,3) = 0.0;  // Pose left (90)
    affMatrix(1,0) = 0.0;   affMatrix(1,1) = 0.0;   affMatrix(1,2) = 1.0;   affMatrix(1,3) = 0.0;  // Pose front (0)
    affMatrix(2,0) = 1.0;   affMatrix(2,1) = 0.0;   affMatrix(2,2) = 0.0;   affMatrix(2,3) = 1.0;  // Pose right (-90)

    // Tool 1 -> hook
    affMatrix(3,0) = 1.0;   affMatrix(3,1) = 1.0;   affMatrix(3,2) = 1.0;   affMatrix(3,3) = 1.0;  // Pose left  (90)
    affMatrix(4,0) = 0.0;   affMatrix(4,1) = 0.0;   affMatrix(4,2) = 0.0;   affMatrix(4,3) = 0.0;  // Pose front (0)
    affMatrix(5,0) = 1.0;   affMatrix(5,1) = 1.0;   affMatrix(5,2) = 1.0;   affMatrix(5,3) = 1.0;  // Pose right (-90)

    // Tool 2 -> rake
    affMatrix(6,0) = 1.0;   affMatrix(6,1) = 1.0;   affMatrix(6,2) = 0.0;   affMatrix(6,3) = 0.0;  // Pose left  (90)
    affMatrix(7,0) = 0.0;   affMatrix(7,1) = 1.0;   affMatrix(7,2) = 1.0;   affMatrix(7,3) = 1.0;  // Pose front (0)
    affMatrix(8,0) = 1.0;   affMatrix(8,1) = 0.0;   affMatrix(8,2) = 0.0;   affMatrix(8,3) = 1.0;  // Pose right (-90)

    // Tool 3 -> stick
    affMatrix(9,0) = 1.0;   affMatrix(9,1) = 0.0;   affMatrix(9,2) = 0.0;   affMatrix(9,3) = 0.0;  // Pose left  (90)
    affMatrix(10,0) = 1.0;  affMatrix(10,1) = 0.0;  affMatrix(10,2) = 0.0;  affMatrix(10,3) = 0.0; // Pose front (0)
    affMatrix(11,0) = 1.0;  affMatrix(11,1) = 0.0;  affMatrix(11,2) = 0.0;  affMatrix(11,3) = 0.0; // Pose right (-90)


    affBottle.clear();

    Matrix toolAffMat;
    if (allAffs){
        int toolI = 0;
        string toolName;
        for (int toolVecInd = 0; toolVecInd < affMatrix.rows()/3; toolVecInd + 3){

            // Select the tool
            if (toolI == 0){
                toolName = "real/pipeHoe3";
            }
            if (toolI == 1){
                toolName = "real/pipeHook3";
            }
            if (toolI = 2){
                toolName = "real/rakeBlue";
            }
            if (toolI = 3){
                toolName = "real/realStick3";
            }
            Bottle &toolAffBot = affBottle.addList();
            toolAffBot.addString(toolName);
            Property &affProps = toolAffBot.addDict();

            // Get all affordances corresponding to that tool
            toolAffMat = affMatrix.submatrix(toolVecInd, toolVecInd + 2, 0, cols-1);

            getAffProps(toolAffMat, affProps);
        }
    }else{

        // Write the name of the tool in the bottle
        // Get index of tool pose in hand
        int toolposeI = getTPindex(saveName, toolPose);
        if (toolposeI < 0){
            cout << "No tool loaded " << endl;
            affBottle.addString("no_aff");
            return false;
        }

        // Get name of tool in hand
        Bottle &toolAffBot = affBottle.addList();
        toolAffBot.addString(saveName);
        Property &affProps = toolAffBot.addDict();

        toolAffMat = affMatrix.submatrix(toolposeI, toolposeI, 0 , cols-1);

        // get the tool-pose affordances
        getAffProps(toolAffMat, affProps);
    }

    return true;
}

bool Objects3DExplorer::getAffProps(const Matrix &affMatrix, Property &affProps)
{

    int rows = affMatrix.rows();
    int cols = affMatrix.cols();
    Vector affVector(cols, 0.0);
    if (rows > 1){
        // Sum all vectors into one.
        for (int r = 0; r < rows; r++){
            affVector = affVector + affMatrix.getRow(r);
        }
    }

    affProps.clear();
    bool affOK  = false;
    if (affVector[0] > 0.0){
        cout << "drag_left affordable " << endl;
        affProps.put("drag_left",affVector[0]);
        affOK = true;
    }

    if (affVector[1] > 0.0){
        cout << "drag_diag_left affordable " << endl;
        affProps.put("drag_diag_left",affVector[1]);
        affOK = true;
    }

    if (affVector[2] > 0.0){
        cout << "drag_down affordable " << endl;
        affProps.put("drag_down",affVector[2]);
        affOK = true;
    }

    if (affVector[3] > 0.0){
        cout << "drag_diag_right affordable " << endl;
        affProps.put("drag_diag_right",affVector[3]);
        affOK = true;
    }

    if (!affOK){
        cout << "This tool-pose does not afford any of the possible actions." << endl;
        affProps.put("no_aff", 0.0);
    }

    cout << "Tool (pose) affordances are " << affProps.toString() << endl;

    return affOK;
}

int Objects3DExplorer::getTPindex(const std::string &tool, const yarp::sig::Matrix &pose)
{
    double ori, displ, tilt, shift;
    paramFromPose(pose, ori, displ, tilt, shift);
    cout << "Param returned from paramFromPose to set aff = " << ori << ", " << displ << ", " << tilt << ", " << shift << "." << endl;

    double toolI = -1, poseI = 0;
    if (tool == "real/pipeHoe3"){
        toolI = 0;
    }
    if (tool == "real/pipeHook3"){
        toolI = 1;
    }
    if (tool == "real/rakeBlue"){
        toolI = 2;
    }
    if (tool == "real/realStick3"){
        toolI = 3;
    }
    if (toolI == -1){
        cout << "No tool is loaded" << endl;
        return -1;
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

    cout << " Rotations estimated: " << endl << "RotX = " << rotX << ". RotY= " << rotY << ". RotZ= " << rotZ << endl;

    double transX = pose(0,3) * 100.0;      // Displacement along X axis in cm
    double transY = pose(1,3) * 100.0;      // Displacement along Y axis in cm
    double transZ = pose(2,3) * 100.0;      // Displacement along Z axis in cm


    cout << " Translations estimated: " << endl << "TransX= " << transX << ". TransY= " << transY << ". TransZ= " << transZ << endl;

    if ((fabs(transX) > 10) || (fabs(transY) > 10) || (fabs(transZ) > 10)){
        cout << "Detected translation is over possible grasp" << endl;
        return false;
    }

    return true;
}


/************************************************************************/
bool Objects3DExplorer::frame2Hand(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trans)
{   // Normalizes the frame of the point cloud from the robot frame (as acquired) to the hand frame.

    // Transform (translate-rotate) the pointcloud by inverting the hand pose
    Vector H2Rpos, H2Ror;
    iCartCtrl->getPose(H2Rpos,H2Ror);
    Matrix H2R = axis2dcm(H2Ror);   // from axis/angle to rotation matrix notation

    // Include translation
    H2R(0,3)= H2Rpos[0];
    H2R(1,3)= H2Rpos[1];
    H2R(2,3)= H2Rpos[2];
    //if (verbose){ printf("Hand to robot transformatoin matrix (H2R):\n %s \n", H2R.toString().c_str());}

    Matrix R2H = SE3inv(H2R);    //inverse the affine transformation matrix from robot to hand
    //if (verbose){printf("Robot to Hand transformatoin matrix (R2H):\n %s \n", R2H.toString().c_str());}

    // Put Transformation matrix into Eigen Format
    Eigen::Matrix4f TM = CloudUtils::yarpMat2eigMat(R2H);
    //cout << TM.matrix() << endl;

    // Executing the transformation
    pcl::transformPointCloud(*cloud_orig, *cloud_trans, TM);

    if (verbose){	printf("Transformation done \n");	}

    return true;
}

/************************************************************************/
bool Objects3DExplorer::sendPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    Bottle &cloudBottleOut = cloudsOutPort.prepare();
    cloudBottleOut.clear();
    
    CloudUtils::cloud2bottle(cloud, cloudBottleOut);    
       
    if (verbose){cout << "Sending out cloud of size " << cloud->size()<< endl;}
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
    //cout << "Addint point at (" << coords.x << ", " << coords.y << ", " << coords.z << ") " << endl;

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

bool Objects3DExplorer::showTooltip(Point3D coords, int color[])
{
    cout << "Adding sphere at (" << coords.x << ", " << coords.y << ", " << coords.z << ") " << endl;
    Time::delay (0.5);
    Bottle cmdVis, replyVis;
    cmdVis.clear();	replyVis.clear();
    cmdVis.addString("addSphere");
    Bottle& bCoords = cmdVis.addList();
    bCoords.addDouble(coords.x);
    bCoords.addDouble(coords.y);
    bCoords.addDouble(coords.z);
    Bottle& bColor = cmdVis.addList();
    bColor.addInt(color[0]);
    bColor.addInt(color[1]);
    bColor.addInt(color[2]);

    rpcVisualizerPort.write(cmdVis,replyVis);

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
bool Objects3DExplorer::downsampleCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ds, double res)
{
    //if (verbose){cout << "Model total points: " << cloud_orig->size () << endl;}

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fil(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud_orig);
    vg.setLeafSize(res, res, res);
    vg.filter (*cloud_fil);
    cloud_ds->points.clear();
    cloud_ds->clear();
    copyPointCloud(*cloud_fil, *cloud_ds);

    // if (verbose){cout << " Downsampled to: " << cloud_ds->size () << endl;}

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

    if (rf.check("help"))
        {
            yInfo(" ");
            yInfo("Options:");
            yInfo("  --context    path:        where to find the called resource (default objects3DModeler).");
            yInfo("  --from       from:        the name of the .ini file (default objects3DExplorer.ini).");
            yInfo("  --name       name:        the name of the module (default objects3DExplorer).");
            yInfo("  --robot      robot:       the name of the robot. Default icub.");
            yInfo("  --hand       left/right:  the default hand that the robot will use (default 'right')");
            yInfo("  --camera     left/right:  the default camera that the robot will use (default 'left')");
            yInfo("  --verbose    bool:        verbosity (default false).");

            yInfo("  --handFrame  bool:      Sets whether the recorded cloud is automatically transformed w.r.t the hand reference frame. (default true)");
            yInfo("  --initAlign  bool:      Sets whether FPFH initial alignment is used for cloud alignment. (default true)");
            yInfo("  --seg2D      bool:      Sets whether segmentation would be doen in 2D (true) or 3D (false) (default false)");
            yInfo("  --saving     bool:      Sets whether recorded pointlcouds are saved or not. (default true)");
            yInfo("  --saveName   string:    Sets the root name to save recorded clouds. Defaults:  'cloud'");
            yInfo(" ");
            return 0;
        }


    Objects3DExplorer objects3DExplorer;

    cout<< endl <<"Configure module..."<<endl;
    objects3DExplorer.configure(rf);
    cout<< endl << "Start module..."<<endl;
    objects3DExplorer.runModule();

    cout<<"Main returning..."<<endl;

    return 0;
}


