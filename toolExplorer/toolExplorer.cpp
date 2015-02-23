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
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>

#include <iCub/ctrl/math.h>
#include <iCub/data3D/SurfaceMeshWithBoundingBox.h>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>

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
    
    // ports
    BufferedPort<Bottle >   seedInPort;
    BufferedPort<iCub::data3D::SurfaceMeshWithBoundingBox> cloudsInPort;

    // rpc ports
    RpcServer               rpcPort;
    RpcClient         		rpcObjRecPort;          //rpc port to communicate with objectReconst module
    RpcClient         		rpcMergerPort;        	//rpc port to communicate with mergeClouds module
    RpcClient         		rpcVisualizerPort;      //rpc port to communicate with tool3Dshow module to display pointcloud
    RpcClient         		rpcFeatExtPort;         //rpc port to communicate with the 3D feature extraction module

    // Drivers
    PolyDriver driverG;
    PolyDriver driverL;
    PolyDriver driverR;
    PolyDriver driverHL;
    PolyDriver driverHR;

    IGazeControl      *iGaze;
    ICartesianControl *iCartCtrlL;
    ICartesianControl *iCartCtrlR;
    ICartesianControl *iCartCtrl;

    // config variables
    string hand;
    string eye;
    string robot;    
    string cloudsPath;
    string cloudName;
    bool verbose;
    bool normalizePose;

    // module parameters
    bool saveF;
    bool closing;    
    int numClouds;

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

        bool ok = turnHand(rotDegX, rotDegY);
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
            return false;
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
        // checks that enough pointclouds have been gathered
		// and use merge_point_clouds module to merge them
        // saving the complete pointcloud is done by the merging module
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

    }else if (receivedCmd == "normalize"){
        // activates the normalization of the pointcloud to the hand reference frame.
        bool ok = setNormalization(command.get(1).asString());
        if (ok)
            responseCode = Vocab::encode("ack");
        else {
            fprintf(stdout,"Normalization has to be set to ON or OFF. \n");
            responseCode = Vocab::encode("nack");
            reply.addVocab(responseCode);
            return false;
        }
        reply.addVocab(responseCode);
        return true;

	}else if (receivedCmd == "explore"){
        // Moves the tool in different direction to obtain different points of view
        // and extract corresponding partial pointclouds.
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
        reply.addString("transform - transforms the poincloud using affine to the reference frame of the hand.");
		reply.addString("explore (all)- gets 3D pointcloud from different perspectives and merges them in a single model. If 'all' is given, it will merge all pointclouds at the end, otherwise incrementally.");
		reply.addString("modelname (string) - Changes the name with which the pointclouds will be saved.");
        reply.addString("hand (left/right) - Sets the active hand (default right).");
        reply.addString("eye (left/right) - Sets the active eye (default left).");
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
   
    /************************************************************************/
    bool explore(bool contMerge = true)
    {
        // Explore the tool from different angles and save pointclouds
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

        // Merge together registered partial point clouds
        if (!contMerge){
            mergePointClouds();}
        printf("Clouds merged, saving full model \n");

        // Visualize merged pointcloud
        showPointCloud();
        printf("PC displayed \n");

        // Extract 3D features from merged pointcloud.
        extractFeatures();

        return true;
    }

    /************************************************************************/
    // XXX

    /*
    bool exploreInteractive()
    {
        // Explore the tool and incrementally check registration and add merge pointclouds if correct


        // Explore the tool from different angles and save pointclouds
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

        // Merge together registered partial point clouds
        if (!contMerge){
            mergePointClouds();}
        printf("Clouds merged, saving full model \n");

        // Visualize merged pointcloud
        showPointCloud();
        printf("PC displayed \n");

        // Extract 3D features from merged pointcloud.
        extractFeatures();

        return true;
    }
    */

    /************************************************************************/
    bool turnHand(const int rotDegX = 0, const int rotDegY = 0)
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
        ox[0]=1.0; ox[1]=0.0; ox[2]=0.0; ox[3]=CTRL_DEG2RAD*(hand=="left"?-rotDegX:rotDegX); // rotation over X axis
        oy[0]=0.0; oy[1]=1.0; oy[2]=0.0; oy[3]=CTRL_DEG2RAD*(hand=="left"?-rotDegY:rotDegY); // rotation over Y axis

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

        return true;
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
		
        // send image blob coordinates as rpc to objectRec to seed the cloud
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

        // read the cloud from the objectReconst output port
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
        iCub::data3D::SurfaceMeshWithBoundingBox *cloudMesh = cloudsInPort.read(true);	//waits until it receives coordinates
        if (cloudMesh!=NULL){
            if (verbose){	printf("Cloud read from port \n");	}
            mesh2cloud(*cloudMesh,cloud);
        } else{
            if (verbose){	printf("Couldnt read returned cloud \n");	}
            return -1;
        }

        // Transform the cloud's frame so that the bouding box is aligned with the hand coordinate frame
        if (normalizePose) {
	    printf("Normalizing cloud to hand reference frame \n");
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudNorm (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
            transformFrame(cloud, cloudNorm);
            string normS = "_norm";
            savePointsPly(cloudNorm, cloudName+normS);
            printf("Cloud normalized to hand reference frame \n");	
        }

        if (verbose){	printf("3D reconstruction obtained and saved.\n");}	

        return true;
	}

    /************************************************************************/
    bool transformFrame(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, const bool saveFlag = true)
    {
        // Transform (translate-rotate) the pointcloud by inverting the hand pose
        if (hand=="left")
            iCartCtrl=iCartCtrlL;
        else if (hand=="right")
            iCartCtrl=iCartCtrlR;
        else
            return false;

        Vector H2Rpos, H2Ror;
        iCartCtrl->getPose(H2Rpos,H2Ror);
        Matrix H2R = axis2dcm(H2Ror);   // from axis/angle to rotation matrix notation

        // Include translation
        H2R(0,3)= H2Rpos[0];
        H2R(1,3)= H2Rpos[1];
        H2R(2,3)= H2Rpos[2];
        printf("Hand to robot transformatoin matrix (H2R):\n %s \n", H2R.toString().c_str());

        Matrix R2H = SE3inv(H2R);    //inverse the affine transformation matrix from robot to hand
        printf("Robot to Hand transformatoin matrix (R2H):\n %s \n", R2H.toString().c_str());

        // Put Transformation matrix into Eigen Format
        Eigen::Matrix4f TM = Eigen::Matrix4f::Identity();
        TM(0,0) = R2H(0,0);     TM(0,1) = R2H(0,1);     TM(0,2) = R2H(0,2);     TM(0,3) = R2H(0,3);
        TM(1,0) = R2H(1,0);     TM(1,1) = R2H(1,1);     TM(1,2) = R2H(1,2);     TM(1,3) = R2H(1,3);
        TM(2,0) = R2H(2,0);     TM(2,1) = R2H(2,1);     TM(2,2) = R2H(2,2);     TM(2,3) = R2H(2,3);
        TM(3,0) = R2H(3,0);     TM(3,1) = R2H(3,1);     TM(3,2) = R2H(3,2);     TM(3,3) = R2H(3,3);
        cout << TM.matrix() << endl;

        // Executing the transformation
        pcl::transformPointCloud(*cloud_in, *cloud_out, TM);

        if (verbose){	printf("Transformation done \n");	}

        return true;
    }

    /************************************************************************/
    bool mergePointClouds()
	{
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
    bool extractFeatures()
    {
        // Sends an RPC command to the toolFeatExt module to extract the 3D features of the merged point cloud/
        Bottle cmdFext, replyFext;
        cmdFext.clear();	replyFext.clear();
        cmdFext.addString("getFeat");
        
        rpcFeatExtPort.write(cmdFext,replyFext);

        return true;
    }

    /************************************************************************/
    bool changeModelName(const string& modelname)
	{
        // Changes the name with which the pointclouds will be saved and read
        cloudName = modelname;

        Bottle cmdOR, replyOR;
        cmdOR.clear();	replyOR.clear();
	    cmdOR.addString("name");
	    cmdOR.addString(modelname);
	    rpcObjRecPort.write(cmdOR,replyOR);        

        Bottle cmdMPC, replyMPC;
        cmdMPC.clear();	replyMPC.clear();
        cmdMPC.addString("name");
        cmdMPC.addString(modelname + "Merged.ply");
        rpcMergerPort.write(cmdMPC,replyMPC);

        Bottle cmdFext, replyFext;
        cmdFext.clear();	replyFext.clear();
        cmdFext.addString("setName");
        cmdFext.addString(modelname + "Merged.ply");
        rpcFeatExtPort.write(cmdFext,replyFext);
	    
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

    bool setNormalization(const string& norm)
    {
        if (norm == "ON"){
            normalizePose = true;
            fprintf(stdout,"Normalization is : %s\n", norm.c_str());
            return true;
        } else if (norm == "OFF"){
            normalizePose = false;
            fprintf(stdout,"Normalization is : %s\n", norm.c_str());
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


    /*************************** -Helper functions- ******************************/

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
        if (verbose){	printf("Mesh fromatted as Point Cloud \n");	}
    }

    /************************************************************************/
    void savePointsPly(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const string& name)
    {
        stringstream s;
        s.str("");
        s << cloudsPath + "/" + name.c_str() << numClouds;
        string filename = s.str();
        string filenameNumb = filename+".ply";
        ofstream plyfile;
        plyfile.open(filenameNumb.c_str());
        plyfile << "ply\n";
        plyfile << "format ascii 1.0\n";
        plyfile << "element vertex " << cloud->width <<"\n";
        plyfile << "property float x\n";
        plyfile << "property float y\n";
        plyfile << "property float z\n";
        plyfile << "property uchar diffuse_red\n";
        plyfile << "property uchar diffuse_green\n";
        plyfile << "property uchar diffuse_blue\n";
        plyfile << "end_header\n";

        for (unsigned int i=0; i<cloud->width; i++)
            plyfile << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " " << (int)cloud->at(i).r << " " << (int)cloud->at(i).g << " " << (int)cloud->at(i).b << "\n";

        plyfile.close();

        numClouds++;
        fprintf(stdout, "Writing finished\n");
    }

/************************************************************************/

public:
    bool configure(ResourceFinder &rf)
    {
        string name = rf.check("name",Value("toolExplorer")).asString().c_str();
        robot = rf.check("robot",Value("icub")).asString().c_str();
        if (strcmp(robot.c_str(),"icub")==0)
            cloudsPath = rf.find("clouds_path").asString();
        else
            cloudsPath = rf.find("clouds_path_sim").asString();
        cloudName = rf.check("modelName", Value("cloud")).asString();
        hand = rf.check("hand", Value("right")).asString();
	    eye = rf.check("camera", Value("left")).asString();
	    verbose = rf.check("verbose", Value(false)).asBool();
       // normalizePose = rf.check("normalize", Value(true)).asBool();


	normalizePose = true;

	    //ports
        bool ret = true;
        ret = seedInPort.open(("/"+name+"/seed:i").c_str());	                       // input port to receive data from user
        ret = ret && cloudsInPort.open(("/"+name+"/clouds:i").c_str());                  // port to receive pointclouds from
        if (!ret){
            printf("Problems opening ports\n");
            return false;
        }

        // RPC ports
        bool retRPC = true;
        retRPC = rpcPort.open(("/"+name+"/rpc:i").c_str());
        retRPC = retRPC && rpcObjRecPort.open(("/"+name+"/objrec:rpc").c_str());             // port to send data out for recording
        retRPC = retRPC && rpcMergerPort.open(("/"+name+"/merger:rpc").c_str());             // port to command the pointcloud IPC merger module
        retRPC = retRPC && rpcFeatExtPort.open(("/"+name+"/featExt:rpc").c_str());           // port to command the pointcloud feature extraction module
        retRPC = retRPC && rpcVisualizerPort.open(("/"+name+"/visualizer:rpc").c_str());     // port to command the visualizer module
        if (!retRPC){
            printf("Problems opening RPC ports\n");
	        return false;
	    }

        attach(rpcPort);

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
        numClouds = 0;
    	
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

        seedInPort.interrupt();
        cloudsInPort.interrupt();

        rpcPort.interrupt();
        rpcObjRecPort.interrupt();
        rpcMergerPort.interrupt();
        rpcVisualizerPort.interrupt();
        rpcFeatExtPort.interrupt();
        
        return true;
    }

    /************************************************************************/
    bool close()
    {
        seedInPort.close();
        cloudsInPort.close();

        rpcPort.close();
        rpcObjRecPort.close();
        rpcMergerPort.close();
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
    rf.setDefaultContext("toolModeler");
    rf.setDefaultConfigFile("toolExplorer.ini");
    rf.setVerbose(true);
    rf.configure(argc,argv);

    ToolExplorer toolExplorer;
    return toolExplorer.runModule(rf);
}


