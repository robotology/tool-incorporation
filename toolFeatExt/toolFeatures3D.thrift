#toolFeatures3D.thrift

struct RotationMatrix 
{
  1: list<double> mat;
} 
(
  yarp.name = "yarp::sig::Matrix"
  yarp.includefile="yarp/sig/Matrix.h"
)

#struct VoxFeat
#{
#1: list<double> voxHist;          # Every voxel is reprsented as a histogram.
#}

struct ToolFeat3D
{   
1: string toolname;
#2: list< VoxFeat> toolFeats;            # Every tool is reprsented as a vector of voxel histograms.
2: list< list < double>> toolFeats;      # Every tool is reprsented as a vector of voxel histograms.
}

struct ToolFeat3DwithOrient
{   
1: string toolname;
#2: list< VoxFeat> toolFeats;           # Every tool is reprsented as a vector of voxel histograms.
2: list< list < double>> toolFeats;      # Every tool is reprsented as a vector of voxel histograms.
3: RotationMatrix orientation;          # Orientation with respect to the canonical position.
}


/**
* tool3DFeat_IDLServer Interface.
*/

service tool3DFeat_IDLServer
{
    // getFeats - Performs 3D feature extraction of the tool in the rotated pose.
    // @return true/false on success/failure of extracting features
    bool getFeats();

    // setPose (opt yarp::sig::Matrix) - Rotates the tool model according tot the given rotation matrix to extract pose dependent features/
    //  @return true/false on success/failure of rotating matrix
    bool setPose(1: RotationMatrix toolPose); //overloaded method for a given parameter

    // setCanonicalPose (int) - Rotates the tool model to canonical orientations left (-90 deg)/front (0 deg)/right (90 deg).
    // @return true/false on success/failure of rotating matrix
    bool setCanonicalPose(1: i32 deg = 0);

    // bins (int) - sets the number of bins per angular dimension (yaw-pitch-roll) used to compute the normal histogram. Total number of bins per voxel = bins^3. (Default bins = 4)");
    // @return true/false on success/failure of setting bins
    bool bins(1: i32 nbins = 2);

    // depth (int)- sets the number of iterative times that the bounding box will be subdivided into octants. Total number of voxels = sum(8^(1:depth)). (Default depth = 2, 72 vox).
    // @return true/false on success/failure of setting maxDepth
    bool depth(1: i32 maxDepth = 2);

    // setVerbose (ON/OFF) - sets verbose of the output on or off
    // @return true/false on success/failure of setting verbose
    bool setVerbose(1: string verb);

    // name (string) -Changes the name of the .ply file to display. Default 'cloud_merged.ply'".
    // @return true/false on success/failure of changing the files name
    bool setName(1: string cloudname = "cloud_merged.ply");

    // help - produces this thrift rpc help.
    // @return true/false on success/failure of extracting features
    //bool help();
}
