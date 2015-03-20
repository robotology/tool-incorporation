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

#include <iCub/data3D/SurfaceMeshWithBoundingBox.h>
#include <iCub/data3D/minBoundBox.h>
#include <iCub/data3D/RGBA.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

/************************************************************************/
class ToolExplorer: public RFModule
{
protected:
    
    // ports
    BufferedPort<Bottle >   seedInPort;
    BufferedPort<iCub::data3D::SurfaceMeshWithBoundingBox> meshInPort;
    BufferedPort<iCub::data3D::SurfaceMeshWithBoundingBox> meshOutPort;

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
    bool initAlignment;
    bool closing;    
    int numCloudsSaved;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in;       // Last registered pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp;   // Merged pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merged;   // Merged pointcloud


    /************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
	/* This method is called when a command string is sent via RPC */
    reply.clear();  // Clear reply bottle

	/* Get command string */
	string receivedCmd = command.get(0).asString().c_str();
	int responseCode;   //Will contain Vocab-encoded response

    if (receivedCmd == "explore"){
        // Moves the tool in different direction to obtain different points of view and extracts corresponding partial pointclouds.
        bool ok = explore();
        if (ok){
            responseCode = Vocab::encode("ack");
            reply.addVocab(responseCode);
            return true;
        } else {
            fprintf(stdout,"Couldnt obtain 3D model successfully. \n");
            reply.addString("[nack] Couldnt obtain 3D model successfully.\n");
            return false;
        }

    }else if (receivedCmd == "exploreInt"){
        // Moves the tool in different direction to obtain different points of view and extracts corresponding partial pointclouds.
        bool ok = exploreInteractive();
        if (ok){
            responseCode = Vocab::encode("ack");
            reply.addVocab(responseCode);
            return true;
        } else {
            fprintf(stdout,"There was an error during the interactive exploration. \n");
            reply.addString("[nack] There was an error during the interactive exploration. \n");
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
		    responseCode = Vocab::encode("ack");
            reply.addVocab(responseCode);
            return true;
        } else {
            fprintf(stdout,"Couldnt go to the desired position. \n");
            reply.addString("[nack] Couldnt go to the desired position. \n" );
  		    return false;
		}

	}else if (receivedCmd == "get3D"){
        // segment object and get the pointcloud using objectReconstrucor module save it in file or array        
        bool ok = getPointCloud();
        if (ok) {            
            showPointCloud(cloud_in);
            savePointsPly(cloud_in,cloudName);
		    responseCode = Vocab::encode("ack");
            reply.addVocab(responseCode);
            return true;
        } else {
		    fprintf(stdout,"Couldnt reconstruct pointcloud. \n");
            reply.addString("[nack] Couldnt reconstruct pointcloud. \n");
		    return false;
        }

    }else if (receivedCmd == "merge"){
        // checks that enough pointclouds have been gathered
		// and use merge_point_clouds module to merge them
        // saving the complete pointcloud is done by the merging module
        bool ok = mergeAllPointClouds();
        if (ok){
		    responseCode = Vocab::encode("ack");
            reply.addVocab(responseCode);
            return true;
        }else {
		    fprintf(stdout,"Couldnt merge pointclouds. \n");
            reply.addString("[nack] Couldnt merge pointclouds. \n");
            return false;
		}


    }else if (receivedCmd == "normalize"){
        // activates the normalization of the pointcloud to the hand reference frame.
        bool ok = setNormalization(command.get(1).asString());
        if (ok){
            responseCode = Vocab::encode("ack");        
            reply.addVocab(responseCode);
            return true;}
        else {
            fprintf(stdout,"Normalization has to be set to ON or OFF. \n");
            reply.addString("[nack] Normalization has to be set to ON or OFF. \n");
            return false;
        }


    }else if (receivedCmd == "FPFH"){
        // activates the normalization of the pointcloud to the hand reference frame.
        bool ok = setInitialAlignment(command.get(1).asString());
        if (ok){
            responseCode = Vocab::encode("ack");
            reply.addVocab(responseCode);
            return true;}
        else {
            fprintf(stdout,"FPFH based Initial Alignment has to be set to ON or OFF. \n");
            reply.addString("[nack] FPFH based Initial Alignment has to be set to ON or OFF. \n");
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
            responseCode = Vocab::encode("ack");
            reply.addVocab(responseCode);
            return true;
        }else {
            fprintf(stdout,"Couldnt change the name. \n");
            reply.addString("[nack] Couldnt change the name. \n");
            return false;
        }

	}else if (receivedCmd == "hand"){
		bool ok = setHand(command.get(1).asString());
        if (ok){
            responseCode = Vocab::encode("ack");
            reply.addVocab(responseCode);
            return true;
        } else {
		    fprintf(stdout,"Hand can only be set to 'right' or 'left'. \n");
            reply.addString("[nack] Hand can only be set to 'right' or 'left'. \n");
            return false;
		}

	}else if (receivedCmd == "eye"){
		bool ok = setEye(command.get(1).asString());
        if (ok){
		    responseCode = Vocab::encode("ack");
            reply.addVocab(responseCode);
            return true;
        }else {
		    fprintf(stdout,"Eye can only be set to 'right' or 'left'. \n");
            reply.addString("[nack] Eye can only be set to 'right' or 'left'. \n");
		    return false;		    
		}

	}else if (receivedCmd == "verbose"){
		bool ok = setVerbose(command.get(1).asString());
        if (ok){
            responseCode = Vocab::encode("ack");
            reply.addVocab(responseCode);
            return true;
        }
		else {
		    fprintf(stdout,"Verbose can only be set to ON or OFF. \n");
            reply.addString("[nack] Verbose can only be set to ON or OFF. \n");
		    return false;
		}

    }else if (receivedCmd == "mergeFromFiles"){
        string cloud_from_name = command.get(1).asString();
        string cloud_to_name = command.get(2).asString();

        cout << "Attempting to load " << (cloudsPath + cloud_from_name).c_str() << "... "<< endl;
        cout << "Attempting to load " << (cloudsPath + cloud_to_name).c_str() << "... "<< endl;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_from (new pcl::PointCloud<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to (new pcl::PointCloud<pcl::PointXYZRGB> ());
        if (pcl::io::loadPLYFile (cloudsPath + cloud_from_name, *cloud_from) < 0)  {
              std::cout << "Error loading point cloud " << cloud_from_name.c_str() << endl << endl;}
        else{
            cout << "cloud of size "<< cloud_from->points.size() << " points loaded from .ply." << endl;}
        if (pcl::io::loadPLYFile (cloudsPath + cloud_to_name, *cloud_to) < 0)  {
              std::cout << "Error loading point cloud " << cloud_to_name.c_str() << endl << endl;}
        else{
            cout << "cloud of size "<< cloud_to->points.size() << " points loaded from .ply." << endl;}

        // Merge the clouds
        Eigen::Matrix4f alignMatrix;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());
        alignPointClouds(cloud_from, cloud_to, cloud_aligned, alignMatrix);
        *cloud_to+=*cloud_aligned;                            // write aligned registration first to a temporal merged

        cout << "Alignment Matrix is "<< endl << alignMatrix << endl;
        // Display the merged cloud
        showPointCloud(cloud_to);

        responseCode = Vocab::encode("ack");
        reply.addVocab(responseCode);

        return true;

	}else if (receivedCmd == "help"){
		reply.addVocab(Vocab::encode("many"));
		responseCode = Vocab::encode("ack");
		reply.addString("Available commands are:");
        reply.addString("explore - automatically gets 3D pointcloud from different perspectives and merges them in a single model.");
        reply.addString("exploreInt - interactively explores the tool and asks for confirmation on each registration until a proper 3D model is built.");
        reply.addString("turnHand  (int)X (int)Y- moves arm to home position and rotates hand 'int' X and Y degrees around the X and Y axis  (0,0 by default).");
		reply.addString("get3D - segment object and get the pointcloud using objectReconstrucor module.");
		reply.addString("merge - use merge_point_clouds module to merge gathered views.");                
		reply.addString("modelname (string) - Changes the name with which the pointclouds will be saved.");
        reply.addString("nomalize (ON/OFF) - Activates/deactivates normalization of the cloud to the hand coordinate frame.");
        reply.addString("FPFH (ON/OFF) - Activates/deactivates fast local features (FPFH) based Initial alignment for registration.");
        reply.addString("verbose (ON/OFF) - Sets ON/OFF printouts of the program, for debugging or visualization.");
        reply.addString("hand (left/right) - Sets the active hand (default right).");
        reply.addString("eye (left/right) - Sets the active eye (default left).");              
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
   
    /************************************************************************/
    bool explore()
    {
        // Explore the tool from different angles and save pointclouds
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud

        string  mergedName = cloudName + "_merged";
        for (int degX = 60; degX>=-75; degX -= 15)
        {
            turnHand(degX,0);            
            //lookAround();
            getPointCloud();
            printf("Exploration from  rot X= %i, Y= %i done. \n", degX, 0 );
            savePointsPly(cloud_in,cloudName);
            Time::delay(0.5);
        }
        for (int degY = 0; degY<=60; degY += 15)
        {
            turnHand(-60,degY);
            //lookAround();
            getPointCloud();
            savePointsPly(cloud_in,cloudName);
            printf("Exploration from  rot X= %i, Y= %i done. \n", -60, degY );            
            Time::delay(0.5);
        }
        printf("Exploration finished, merging clouds \n");

        // Merge together registered partial point clouds
        mergeAllPointClouds();
        printf("Clouds merged, saving full model \n");

        // Visualize merged pointcloud
        showPointCloudFromFile(cloudName + "_merged.ply");
        printf("PC displayed \n");

        // Extract 3D features from merged pointcloud.
        // extractFeatures();

        return true;
    }

    /************************************************************************/
    bool exploreInteractive()
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
                savePointsPly(cloud_merged, mergedName,false);
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
                savePointsPly(cloud_in, cloudName);

                // If the cloud is clean, merge the last recorded cloud_in with the existing cloud_merged and save on cloud_temp
                Eigen::Matrix4f alignMatrix;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());
                alignPointClouds(cloud_in, cloud_merged, cloud_aligned, alignMatrix);
                *cloud_temp+=*cloud_aligned;                            // write aligned registration first to a temporal merged

                // Display the merged cloud
                showPointCloud(cloud_temp);

                // If merge is good, update model
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
                        savePointsPly(cloud_merged, mergedName, false);
                        printf(" Final model saved as %s, finishing exploration \n", mergedName.c_str());
                        explorationDone = true;
                    } else {
                        printf(" Model not finished, continuing with exploration \n");
                    }
                } else {
                    printf("\n Ignoring merge, continue with exploration \n");
                }
            } else {
                printf("\n Unproper registration, try again \n");
            }
            printf("Exploration from  rot X= %f, Y= %f done. \n",positionsX[Xind] , positionsY[Yind] );
        }

        // Visualize merged pointcloud
        printf("Model finished, visualizing \n");
        showPointCloud(cloud_merged);
        return true;
    }

    /************************************************************************/
    bool findToolPose(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr modelCloud, Matrix toolPose)
    {
         // XXX  Define where the modelCloud is found or loaded from.

         // Get a registration
         getPointCloud();    // Registration get and normalized to hand-reference frame. saved as 'cloud_in'

         // Align it to the canonical model
         Eigen::Matrix4f alignMatrix;
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB> ());
         alignPointClouds(cloud_in, modelCloud, cloud_aligned, alignMatrix);

         // return the alineation matrix as toolPose YARP Matrix
         toolPose = eigMat2yarpMat(alignMatrix);

         return true;
    }

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
    bool getPointCloud()//(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{        
        cloud_in->clear();   // clear receiving cloud

        // read coordinates from yarpview
	    if (verbose){printf("Getting tip coordinates \n");}
	    Bottle *toolTipIn = seedInPort.read(true);	//waits until it receives coordinates
	    int u = toolTipIn->get(0).asInt();
	    int v = toolTipIn->get(1).asInt();
	    if (verbose){cout << "Retrieving tool blob from u: "<< u << ", v: "<< v << endl;	}
		
        // send image blob coordinates as rpc to objectRec to seed the cloud
        Bottle cmdOR, replyOR;
	    cmdOR.clear();	replyOR.clear();
	    cmdOR.addInt(u);
	    cmdOR.addInt(v);
	    rpcObjRecPort.write(cmdOR,replyOR);
	
	    // requests 3D reconstruction to objectReconst module
	    cmdOR.clear();	replyOR.clear();
	    cmdOR.addString("3Drec");
	    rpcObjRecPort.write(cmdOR,replyOR);

        // read the cloud from the objectReconst output port
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
        iCub::data3D::SurfaceMeshWithBoundingBox *cloudMesh = meshInPort.read(true);	//waits until it receives coordinates
        if (cloudMesh!=NULL){
            if (verbose){	printf("Cloud read from port \n");	}
            mesh2cloud(*cloudMesh,cloud_in);
        } else{
            if (verbose){	printf("Couldnt read returned cloud \n");	}
            return -1;
        }

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
        if (normalizePose) {
            printf("Normalizing cloud to hand reference frame \n");
            //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudNorm (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
            transformFrame(cloud_in, cloud_in);
            printf("Cloud normalized to hand reference frame \n");
        }

        if (verbose){	printf("3D reconstruction obtained.\n");}

        return true;
	}

    /************************************************************************/
    bool transformFrame(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_trans)
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
        pcl::transformPointCloud(*cloud_orig, *cloud_trans, TM);

        if (verbose){	printf("Transformation done \n");	}

        return true;
    }

    /************************************************************************/
    bool alignPointClouds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_from, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned, Eigen::Matrix4f& transfMat)
    {
        // cloud_temp->clear();
        // *cloud_temp = *cloud_merged;        // work on the previously obtaiened merged cloud

        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_aligned (new pcl::PointCloud<pcl::PointXYZRGB>);

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
            int iterations = 50;
            float distance = 0.003;   // 3 mm accepted as max distance between models
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

    void computeLocalFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features)
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


    void computeSurfaceNormals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
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
    bool mergeAllPointClouds()
	{
        // calls 'merge' rpc command to mergeClouds
        Bottle cmdMPC, replyMPC;
        cmdMPC.clear();	replyMPC.clear();
        cmdMPC.addString("merge");
        rpcMergerPort.write(cmdMPC,replyMPC);

        return true;
    }
    

    /************************************************************************/
    bool showPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) //XXX Change so the bottle is sent as Mesh on port to tool3Dshow. This will make tool3D show to display it.
    {
        iCub::data3D::SurfaceMeshWithBoundingBox &meshBottle = meshOutPort.prepare();
        cloud2mesh(cloud, meshBottle);
        if (verbose){printf("Sending out cloud");}
        meshOutPort.write();
        return true;
    }

    /************************************************************************/
    bool showPointCloudFromFile(const string& fname)
    {

        Bottle cmdVis, replyVis;
        cmdVis.clear();	replyVis.clear();
        cmdVis.addString("name");
        cmdVis.addString(fname);
        rpcVisualizerPort.write(cmdVis,replyVis);

        // Sends an RPC command to the tool3Dshow module to display the merged pointcloud on the visualizer
        cmdVis.clear();	replyVis.clear();
        cmdVis.addString("show");
        rpcVisualizerPort.write(cmdVis,replyVis);

        return true;
    }

    /************************************************************************/
    bool extractFeatures() // XXX Change so that cloud can be sent directly via thrift.
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
        cmdMPC.addString(modelname + "_merged");
        rpcMergerPort.write(cmdMPC,replyMPC);

        Bottle cmdFext, replyFext;
        cmdFext.clear();	replyFext.clear();
        cmdFext.addString("setName");
        cmdFext.addString(modelname + "_merged.ply");
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


    bool setInitialAlignment(const string& fpfh)
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

    void mesh2cloud(const iCub::data3D::SurfaceMeshWithBoundingBox& meshB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {   // Converts mesh from a bottle into pcl pointcloud.
        for (size_t i = 0; i<meshB.mesh.points.size(); ++i)
        {
            pcl::PointXYZRGB pointrgb;
            pointrgb.x=meshB.mesh.points.at(i).x;
            pointrgb.y=meshB.mesh.points.at(i).y;
            pointrgb.z=meshB.mesh.points.at(i).z;
            if (i<meshB.mesh.rgbColour.size())
            {
                int32_t rgb= meshB.mesh.rgbColour.at(i).rgba;
                pointrgb.rgba=rgb;
                pointrgb.r = (rgb >> 16) & 0x0000ff;
                pointrgb.g = (rgb >> 8)  & 0x0000ff;
                pointrgb.b = (rgb)       & 0x0000ff;
            }
            else
                pointrgb.rgb=0;

            cloud->push_back(pointrgb);
        }
        if (verbose){	printf("Mesh formatted as Point Cloud \n");	}
    }


    void cloud2mesh(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, iCub::data3D::SurfaceMeshWithBoundingBox& meshB)
    {   // Converts pointcloud to surfaceMesh bottle.

        if (verbose){printf("Clearing mesh \n");}
        meshB.mesh.points.clear();
        meshB.mesh.rgbColour.clear();
        if (verbose){printf("Adding name and points..\n");}
        meshB.mesh.meshName = cloudName;
        for (unsigned int i=0; i<cloud->width; i++)
        {
            meshB.mesh.points.push_back(iCub::data3D::PointXYZ(cloud->at(i).x,cloud->at(i).y, cloud->at(i).z));
            meshB.mesh.rgbColour.push_back(iCub::data3D::RGBA(cloud->at(i).rgba));
        }
        if (verbose){printf("Adding bounding box \n");}
        iCub::data3D::BoundingBox BB = iCub::data3D::MinimumBoundingBox::getMinimumBoundingBox(cloud);
        meshB.boundingBox = BB.getBoundingBox();

        return;
    }


    Matrix eigMat2yarpMat(const Eigen::MatrixXf eigMat){
        // Transforms matrices from Eigen format to YARP format
        int nrows = eigMat.rows();
        int ncols = eigMat.cols();
        Matrix yarpMat = zeros(nrows,ncols);
        for (int row = 0; row<nrows; ++row){
            for (int col = 0; col<ncols; ++col){
                yarpMat(row,col) = eigMat(row,col);
            }
        }
        return yarpMat;
    }

    /************************************************************************/
    void savePointsPly(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const string& savename, const bool addNum = true)
    {
        stringstream s;
        s.str("");
        if (addNum){
            s << cloudsPath + "/" + savename.c_str() << numCloudsSaved;
            numCloudsSaved++;
        } else {
            s << cloudsPath + "/" + savename.c_str();
        }

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

        fprintf(stdout, "Writing finished\n");
    }

/************************************************************************/

public:
    bool configure(ResourceFinder &rf)
    {
        string name = rf.check("name",Value("toolExplorer")).asString().c_str();
        robot = rf.check("robot",Value("icub")).asString().c_str();        
        string cloudpath_file = rf.check("clouds",Value("cloudsPath.ini")).asString().c_str();
        rf.findFile(cloudpath_file.c_str());

        ResourceFinder cloudsRF;
        cloudsRF.setContext("toolModeler");
        cloudsRF.setDefaultConfigFile(cloudpath_file.c_str());
        cloudsRF.configure(0,NULL);

        if (strcmp(robot.c_str(),"icub")==0)
            cloudsPath = cloudsRF.find("clouds_path").asString();
        else
            cloudsPath = cloudsRF.find("clouds_path_sim").asString();
        printf("Path: %s",cloudsPath.c_str());

        cloudName = rf.check("modelName", Value("cloud")).asString();
        hand = rf.check("hand", Value("right")).asString();
	    eye = rf.check("camera", Value("left")).asString();
        verbose = rf.check("verbose", Value(true)).asBool();
        normalizePose = rf.check("normalizePose", Value(true)).asBool();

	    //ports
        bool ret = true;
        ret = seedInPort.open(("/"+name+"/seed:i").c_str());	                       // input port to receive data from user
        ret = ret && meshInPort.open(("/"+name+"/mesh:i").c_str());                  // port to receive pointclouds from
        ret = ret && meshOutPort.open(("/"+name+"/mesh:o").c_str());                  // port to receive pointclouds from
        if (!ret){
            printf("\nProblems opening ports\n");
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


        iGaze->setSaccadesStatus(false);
        if (robot == "icubSim"){
                iGaze->setNeckTrajTime(1.5);
            	iGaze->setEyesTrajTime(0.5);
        }else{
	            iGaze->setNeckTrajTime(1.5);
            	iGaze->setEyesTrajTime(0.5);
        }

        printf("\nInitializing variables... \n");

        closing = false;
        initAlignment = false;
        numCloudsSaved = 0;

        cloud_in = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
        cloud_temp = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
        cloud_merged = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB> ());// Point cloud
    	
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
        meshInPort.interrupt();
        meshOutPort.interrupt();

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
        meshInPort.close();
        meshOutPort.close();

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


