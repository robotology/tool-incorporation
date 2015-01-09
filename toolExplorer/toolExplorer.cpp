/* 
 * Copyright (C) 2014 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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


#include <stdio.h>
#include <time.h>
#include <string>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <iCub/ctrl/math.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;


/************************************************************************/
class ToolExplorer: public RFModule
{
protected:
    
    RpcServer            			rpcPort;
    yarp::os::BufferedPort<yarp::os::Bottle >   seedInPort;
    yarp::os::RpcClient         		rpcObjRecPort;          //rpc port to communicate with objectReconst module
    yarp::os::RpcClient         		rpcMergerPort;        	//rpc port to communicate with MERGE_POINT_CLOUDS module
    yarp::os::RpcClient         		rpcVisualizerPort;      //rpc port to communicate with tool3Dshow module to display pointcloud


    PolyDriver driverG;
    PolyDriver driverL;
    PolyDriver driverR;
    PolyDriver driverHL;
    PolyDriver driverHR;

    IGazeControl      *iGaze;
    ICartesianControl *iCartCtrlL;
    ICartesianControl *iCartCtrlR;
    ICartesianControl *iCartCtrl;

    string hand;
    string eye;
    string robot;
    bool verbose;
    bool saveF;
    bool closing;
        

    /************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
	/* This method is called when a command string is sent via RPC */
    	reply.clear();  // Clear reply bottle

	/* Get command string */
	string receivedCmd = command.get(0).asString().c_str();
	int responseCode;   //Will contain Vocab-encoded response

	if (receivedCmd == "turnHand"){        
		// Turn the hand 'int' degrees, or go to 0 if no parameter was given.
		int rotDegX = 0;
		int rotDegY = 0;
		if (command.size() == 2){
			rotDegY = command.get(1).asInt();
		} else if  (command.size() == 3){
			rotDegX = command.get(1).asInt();
			rotDegY = command.get(2).asInt();
		}			

		bool ok = turnHand(rotDegX, rotDegY, hand, eye);
		if (ok)
		    responseCode = Vocab::encode("ack");
		else {
		    fprintf(stdout,"Couldnt go to the desired position. \n");
		    responseCode = Vocab::encode("nack");
		    reply.addVocab(responseCode);
  		    return false;
		}
		reply.addVocab(responseCode);
		return true;

	}else if (receivedCmd == "get3D"){
		// segment object and get the pointcloud using objectReconstrucor module
		// save it in file or array
		bool ok = getPointCloud(false, saveF);
		if (ok)
		    responseCode = Vocab::encode("ack");
		else {
		    fprintf(stdout,"Couldnt reconstruct pointcloud. \n");
		    responseCode = Vocab::encode("nack");
		    reply.addVocab(responseCode);
		    return false;
		}
		reply.addVocab(responseCode);
		return true;

	}else if (receivedCmd == "modelname"){
		// changes the name with which files will be saved by the object-reconstruction module
        string modelname;
        if (command.size() >= 2){
            modelname = command.get(1).asString();
        }else{
            fprintf(stdout,"Please provide a name. \n");
            return false
        }
		bool ok = changeModelName(modelname);
		if (ok){
		    responseCode = Vocab::encode("ack");
	    	reply.addVocab(responseCode);
    		return true;
		}else {
		    fprintf(stdout,"Couldnt change the name. \n");
		    responseCode = Vocab::encode("nack");
		    reply.addVocab(responseCode);
		    return false;
		}

	}else if (receivedCmd == "merge"){
		// check that enough pointclouds have been gathered
		// and use merge_point_clouds module to merge them
		// save complete  pointcloud elsewhere 
		bool ok = mergePointClouds();
		if (ok)
		    responseCode = Vocab::encode("ack");
		else {
		    fprintf(stdout,"Couldnt merge pointclouds. \n");
		    responseCode = Vocab::encode("nack");
		    reply.addVocab(responseCode);
		    return false;
		}
		reply.addVocab(responseCode);
		return true;

	}else if (receivedCmd == "explore"){
		// check that enough pointclouds have been gathered
		// and use merge_point_clouds module to merge them
		// save complete  pointcloud elsewhere 
		bool contF = true;
		if (command.size() == 2){
			string cmd2 = command.get(1).asString();
			if (cmd2 == "all"){
			    contF= false;}			
		}
		bool ok = explore(contF);

		if (ok)
		    responseCode = Vocab::encode("ack");
		else {
		    fprintf(stdout,"Couldnt obtain 3D model successfully. \n");
		    responseCode = Vocab::encode("nack");
		    reply.addVocab(responseCode);
		    return false;
		}
		reply.addVocab(responseCode);
		return true;

	}else if (receivedCmd == "hand"){
		bool ok = setHand(command.get(1).asString());
		if (ok)
		    responseCode = Vocab::encode("ack");
		else {
		    fprintf(stdout,"Hand can only be set to 'right' or 'left'. \n");
		    responseCode = Vocab::encode("nack");
		    reply.addVocab(responseCode);
		    return false;
		}
		reply.addVocab(responseCode);
		return true;

	}else if (receivedCmd == "eye"){
		bool ok = setEye(command.get(1).asString());
		if (ok)
		    responseCode = Vocab::encode("ack");
		else {
		    fprintf(stdout,"Eye can only be set to 'right' or 'left'. \n");
		    responseCode = Vocab::encode("nack");
		    		    reply.addVocab(responseCode);
		    return false;
		    
		}
		reply.addVocab(responseCode);
		return true;

	}else if (receivedCmd == "verbose"){
		bool ok = setVerbose(command.get(1).asString());
		if (ok)
		    responseCode = Vocab::encode("ack");
		else {
		    fprintf(stdout,"Verbose can only be set to ON or OFF. \n");
		    responseCode = Vocab::encode("nack");
		    reply.addVocab(responseCode);
		    return false;
		}
		reply.addVocab(responseCode);
		return true;


	}else if (receivedCmd == "help"){
		reply.addVocab(Vocab::encode("many"));
		responseCode = Vocab::encode("ack");
		reply.addString("Available commands are:");
		reply.addString("turnHand  (int)X (int)Y- moves arm to home position and rotates hand 'int' X and Y degrees around the X and Y axis  (0,0 by default).");
		reply.addString("get3D - segment object and get the pointcloud using objectReconstrucor module.");
		reply.addString("merge - use merge_point_clouds module to merge gathered views.");
		reply.addString("explore (all)- gets 3D pointcloud from different perspectives and merges them in a single model. If 'all' is given, it will merge all pointclouds at the end, otherwise incrementally.");
		reply.addString("modelname (string) - Changes the name with which the pointclouds will be saved.");
		reply.addString("hand left/right - Sets active the hand (default right).");
		reply.addString("eye left/right - Sets active the eye (default left).");
		reply.addString("verbose ON/OFF - Sets active the printouts of the program, for debugging or visualization.");
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
   

    /************************************************************************/
    bool turnHand(const int rotDegX = 0, const int rotDegY = 0,  const string &arm = "right", const string &eye  = "left")
    {
        if (arm=="left")
            iCartCtrl=iCartCtrlL;
        else if (arm=="right")
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
        xd[1]=(arm=="left")?-0.1:0.1;					// move sligthly out of center towards the side of the used hand 
        xd[2]= 0.1;

	offset[0]=0;
        offset[1]=(arm=="left")?-0.05-(0.01*(rotDegY/10+rotDegX/3)):0.05 + (0.01*(rotDegY/10+rotDegX/3));	// look slightly towards the side where the tool is rotated
        offset[2]= 0.15 - 0.01*abs(rotDegX)/5;

	// Rotate the hand to observe the tool from different positions
	Vector ox(4), oy(4);
	ox[0]=1.0; ox[1]=0.0; ox[2]=0.0; ox[3]=CTRL_DEG2RAD*(arm=="left"?-rotDegX:rotDegX); // rotation over X axis
	oy[0]=0.0; oy[1]=1.0; oy[2]=0.0; oy[3]=CTRL_DEG2RAD*(arm=="left"?-rotDegY:rotDegY); // rotation over Y axis

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
    bool lookAround()
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
    }

    /************************************************************************/
    bool getPointCloud(const bool visF, const bool saveFlag = true)
	{
	    // read coordinates from yarpview
	    if (verbose){printf("Getting tip coordinates \n");}
	    Bottle *toolTipIn = seedInPort.read(true);	//waits until it receives coordinates
	    int u = toolTipIn->get(0).asInt();
	    int v = toolTipIn->get(1).asInt();
	    if (verbose){cout << "Retrieving tool blob from u: "<< u << ", v: "<< v << endl;	}
		
	    // Set obj rec to save mode to keep ply files.
	    Bottle cmdOR, replyOR;
	    if (saveFlag){
		    cmdOR.clear();	replyOR.clear();
		    cmdOR.addString("set");
		    cmdOR.addString("write");
		    cmdOR.addString("on");
		    rpcObjRecPort.write(cmdOR,replyOR);
	    }
		
	    // send coordinates as rpc to objectRec
	    cmdOR.clear();	replyOR.clear();
	    cmdOR.addInt(u);
	    cmdOR.addInt(v);
	    rpcObjRecPort.write(cmdOR,replyOR);
	
	    // requests 3D reconstruction to objectReconst module
	    cmdOR.clear();	replyOR.clear();
	    cmdOR.addString("3Drec");
        if (visF){
    	    cmdOR.addString("visualize");
        }
	    rpcObjRecPort.write(cmdOR,replyOR);
	    
 	    printf("3D reconstruction obtrained and saved.\n");
	
	return true;
	}

    /************************************************************************/
    bool mergePointClouds()
	{
	// sets the folder path to wherever the pcl files have been saved, or reads the array.
	// calls 'merge' rpc command to mergeClouds
   	Bottle cmdMPC, replyMPC;
	cmdMPC.clear();	replyMPC.clear();
	cmdMPC.addString("merge");
	rpcMergerPort.write(cmdMPC,replyMPC);

	return true;
    }
    
    /************************************************************************/
    bool showPointCloud()
	{
	// Sends an RPC command to the tool3Dshow module to display the merged pointcloud on the visualizer 
   	Bottle cmdVis, replyVis;
	cmdVis.clear();	replyVis.clear();
	cmdVis.addString("show");
	rpcVisualizerPort.write(cmdVis,replyVis);

	return true;
    }

    /************************************************************************/
    bool explore(bool contMerge = true)
	{
	    for (int degX = 60; degX>=-75; degX -= 15)
	    {
		    turnHand(degX,0);
		    printf("Exploration from  rot X= %i, Y= %i done. \n", degX, 0 );
		    getPointCloud(false);
		    if (contMerge){
			    mergePointClouds();}
		    Time::delay(0.5);
	    }
	    for (int degY = 0; degY<=60; degY += 15)
	    {
		    turnHand(-60,degY);
            //lookAround();
		    printf("Exploration from  rot X= %i, Y= %i done. \n", -60, degY );
		    getPointCloud(false);
		    if (contMerge){
			    mergePointClouds();}
		    Time::delay(0.5);
	    } 
	    printf("Exploration finished, merging clouds \n");
	    if (!contMerge){
		    mergePointClouds();}
	    printf("Clouds merged, saving full model \n");
        
        // Visualize merged pointcloud
	    showPointCloud();
        printf("PC displayed \n");
        
        return true;
	}

    /************************************************************************/
    bool changeModelName(const string& modelname)
	{
    	// Changes the name with which the pointclouds will be saved
       	Bottle cmdOR, replyMPC;
	    // requests 3D reconstruction to objectReconst module
	    cmdOR.clear();	replyOR.clear();
	    cmdOR.addString("name");
	    cmdOR.addString(modelname);
	    rpcObjRecPort.write(cmdOR,replyOR);
	    
 	    printf("Name changed to %s.\n", modelname.c_str());
	return true;
    }

    /*************************** -Conf Commands- ******************************/
    bool setVerbose(const string& verb)
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

    bool setHand(const string& handName)
	{
	    if ((handName == "left")|| (handName == "right")){
		    hand = handName;
		    fprintf(stdout,"Active hand is: %s\n", handName.c_str());
		    return true;
	    }
	    return false;
	}

    bool setEye(const string& eyeName)
	{
	    if ((eyeName == "left")|| (eyeName == "right")){
		    eye = eyeName;
		    fprintf(stdout,"Active eye is: %s\n", eyeName.c_str());
		    return true;
	    }
	    return false;
	}

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("toolExplorer")).asString().c_str();
        robot=rf.check("robot",Value("icub")).asString().c_str();
        hand = rf.check("hand", Value("right")).asString();
	    eye = rf.check("camera", Value("left")).asString();
	    verbose = rf.check("verbose", Value(false)).asBool();

	    //ports
        rpcPort.open(("/"+name+"/rpc:i").c_str());
        attach(rpcPort);
        

	    bool ret = true;  
        ret = seedInPort.open(("/"+name+"/seed:i").c_str());	                       // input port to receive data from user
        ret = ret && rpcObjRecPort.open(("/"+name+"/objrec:rpc").c_str());             // port to send data out for recording
	    ret = ret && rpcMergerPort.open(("/"+name+"/merger:rpc").c_str());             // port to command the pointcloud IPC merger module
	    ret = ret && rpcVisualizerPort.open(("/"+name+"/visualizer:rpc").c_str());         // port to command the visualizer module
	    if (!ret){
	        printf("Problems opening ports\n");
	        return false;
	    }

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


        iGaze->setSaccadesStatus(false);
        if (robot == "icubSim"){
                iGaze->setNeckTrajTime(1.5);
            	iGaze->setEyesTrajTime(0.5);
        }else{
	            iGaze->setNeckTrajTime(1.5);
            	iGaze->setEyesTrajTime(0.5);
        }


        closing = false;
    	saveF = true;
    	
    	cout << endl << "Configuring done." << endl;
            	
        return true;
    }

    /************************************************************************/
    bool interruptModule()
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

        
        return true;
    }

    /************************************************************************/
    bool close()
    {
        rpcPort.close();

        driverG.close();
        driverL.close();
        driverR.close();
        driverHL.close();
        driverHR.close();
        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 0.02;
    }

    /************************************************************************/
    bool updateModule()
    {
        return !closing;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure(argc,argv);

    ToolExplorer toolExplorer;
    return toolExplorer.runModule(rf);
}


