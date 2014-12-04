/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

	if (receivedCmd == "moveArm"){        
		// Go to position 'int', or random among the possible ones
		int rotDeg;
		if (command.size() == 2)
			rotDeg = command.get(1).asInt();
		else
			rotDeg = 0;

		bool ok = moveArm(rotDeg, hand, eye);
		if (ok)
		    responseCode = Vocab::encode("ack");
		else {
		    fprintf(stdout,"Couldnt go to the desired position. \n");
		    responseCode = Vocab::encode("nack");
  		    return false;
		}
		reply.addVocab(responseCode);
		return true;

	}else if (receivedCmd == "get3D"){
		// segment object and get the pointcloud using objectReconstrucor module
		// save it in file or array
		bool ok = getPointCloud(true, saveF);
		if (ok)
		    responseCode = Vocab::encode("ack");
		else {
		    fprintf(stdout,"Couldnt reconstruct pointcloud. \n");
		    responseCode = Vocab::encode("nack");
		    return false;
		}
		reply.addVocab(responseCode);
		return true;

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
		    return false;
		}
		reply.addVocab(responseCode);
		return true;

	}else if (receivedCmd == "explore"){
		// check that enough pointclouds have been gathered
		// and use merge_point_clouds module to merge them
		// save complete  pointcloud elsewhere 
		bool ok = explore();
		if (ok)
		    responseCode = Vocab::encode("ack");
		else {
		    fprintf(stdout,"Couldnt obtain 3D model successfully. \n");
		    responseCode = Vocab::encode("nack");
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
		}
		reply.addVocab(responseCode);
		return true;


	}else if (receivedCmd == "help"){
		reply.addVocab(Vocab::encode("many"));
		responseCode = Vocab::encode("ack");
		reply.addString("Available commands are:");
		reply.addString("moveArm  (int) - moves arm to home position and rotates hand 'int' degrees (0 by default).");
		reply.addString("get3D - segment object and get the pointcloud using objectReconstrucor module.");
		reply.addString("merge - use merge_point_clouds module to merge gathered views.");
		reply.addString("explore - gets 3D pointcloud from different perspectives and merges them in a single model.");
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
    bool moveArm(const int rotDeg = 0, const string &arm = "right", const string &eye  = "left")
    {
        if (arm=="left")
            iCartCtrl=iCartCtrlL;
        else if (arm=="right")
            iCartCtrl=iCartCtrlR;
        else
            return false;

	if ((rotDeg > 90 ) || (rotDeg < -90))	{
	    printf("Hand rotation has to be between -90 and 90 degrees \n");	
	}

        int context_arm,context_gaze;
        iCartCtrl->storeContext(&context_arm);
        iGaze->storeContext(&context_gaze);
	//iGaze->restoreContext(0);
        

        iGaze->setSaccadesStatus(true);
	if (robot == "icubSim"){
	        iGaze->setNeckTrajTime(1.5);
        	iGaze->setEyesTrajTime(0.5);
	}else{
		iGaze->setNeckTrajTime(2.5);
        	iGaze->setEyesTrajTime(1.5);
	}

	// intialize position and orientation matrices
        Matrix R(4,4);
        R(0,0)=-1.0;
        R(2,1)=-1.0;
        R(1,2)=-1.0;
        R(3,3)=+1.0;
        Vector r(4,0.0); 
        Vector xd(3,0.0),od;
        Vector offset(3,0.0);;
	
	// set base position
        xd[0]=-0.25;
        xd[1]=(arm=="left")?-0.1:0.1;
        xd[2]=0.0;
	offset[0]=0;
        offset[1]=(arm=="left")?(0.01*rotDeg/5):-(0.01*rotDeg/5);
        offset[2]=0.15;

	// Rotate the hand to observe the tool from different positions
	r[2]=1.0;	// rotation over Y axis 
 	r[3]=CTRL_DEG2RAD*(arm=="left"?-rotDeg:rotDeg);
        od=dcm2axis(axis2dcm(r)*R); 
	if (verbose){	printf("Orientation change is:\n %s \n", r.toString().c_str());
			printf("Orientation vector matrix is:\n %s \n", od.toString().c_str());	}

	// move!
	iGaze->setTrackingMode(true);
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
    bool getPointCloud( bool visF, bool saveF)
	{
	    // read coordinates from yarpview
	    printf("Getting tip coordinates \n");
	    Bottle *toolTipIn = seedInPort.read(true);	//waits until it receives coordinates
	    int u = toolTipIn->get(0).asInt();
	    int v = toolTipIn->get(1).asInt();
	    cout << "Retrieving tool blob from u: "<< u << ", v: "<< v << endl;	
		
	    // Set obj rec to save mode to keep ply files.
	    Bottle cmdOR, replyOR;
	    if (saveF){
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
	// sets the /mergeModule path to wherever the pcl files have been saved, or reads the array.
	// calls 'merge' rpc command to merge_point_clouds

	    // Set obj rec to save mode to keep ply files.
	    Bottle cmdMPC, replyMPC;
	    cmdMPC.clear();	replyMPC.clear();
	    cmdMPC.addString("merge");
	    rpcMergerPort.write(cmdMPC,replyMPC);

	return true;
	}

    /************************************************************************/
    bool explore()
	{
	for (int deg = -45; deg<=60; deg += 15)
	   {
		moveArm(deg);
		if (verbose) { printf("Exploration from %i degrees done \n", deg );}
		getPointCloud(false, true);
		Time::delay(0.5);
	   }
    printf("All perspectives explored.\n");
	mergePointClouds();
    printf("Clouds merged.\n");
	return true;
	}

    /*************************** -Conf Commands- ******************************/
    bool setVerbose(string verb)
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

    bool setHand(string handName)
	{
	    if ((handName == "left")|| (handName == "right")){
		hand = handName;
		fprintf(stdout,"Active hand is: %s\n", handName.c_str());
		return true;
	    }
	    return false;
	}

    bool setEye(string eyeName)
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
	ret = ret && rpcMergerPort.open(("/"+name+"/merger:rpc").c_str());             // port to send data out for recording
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


        closing = false;
    	saveF = true;
            	
	// handUsed="null";

        return true;
    }

    /************************************************************************/
    bool interruptModule()
    {
        closing=true;

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


