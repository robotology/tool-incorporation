#toolFeatures3D.thrift

struct Point3D
{
  1: double x;
  2: double y;
  3: double z;
}

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
    /**
     * @brief getFeats - Performs 3D feature extraction of the tool in the rotated pose.
     * @return true/false on success/failure of extracting features
     */
    bool getFeats();

    /**
     * @brief getToolTip - Returns the tooltip (as the center of the edge opposite edge of the hand the bounding box)
     * @return true/false on success/failure of computing the tooltip
     */
    Point3D getToolTip();

    /**
     * @brief getSamples - Generates n poses of the tool around a base orientation deg.
     * @param n - (int) desired number of poses (default = 10) .
     * @param deg - (double) base orientation of the tool pose (default = 0.0).
     * @return true/false on success/failure of generating n samples.
     */
    bool getSamples(1: i32 n = 10, 2: double deg = 0.0);

    /**
     * @brief loadModel - loads a model from a .pcd or .ply file for further processing.
     * @param cloudname - (string) name of the file to load cloud from (and path from base path if needed) (default = "cloud.ply") .
     * @return true/false on success/failure loading the desired cloud.
     */
    bool loadModel(1: string cloudname = "cloud.ply");

    /**
     * @brief setPose - Rotates the tool model according to any given rotation matrix
     * @param rotationMatrix - (yarp::sig::Matrix) rotation matrix to apply to the cloud.
     * @return true/false on success/failure of rotating model according to  matrix.
     */
    bool setPose(1: RotationMatrix toolPose);

    /**
     * @brief setCanonicalPose - Rotates the tool model to canonical orientations, i.e. as a rotation along the longest axis which orients the end effector.
     * @param deg - (double) degrees of rotation along the longest axis.
     * @return true/false on success/failure of rotating model according to orientation.
     */
    bool setCanonicalPose(1: double deg = 0.0);

    /**
     * @brief bins - sets the number of bins per angular dimension (yaw-pitch-roll) used to compute the normal histogram. Total number of bins per voxel = bins^3.
     * @param nbins - (int) desired number of bins per angular dimension. (default = 2, i.e. 8 bins per voxel).
     * @return true/false on success/failure of setting number of bins.
     */
    bool bins(1: i32 nbins = 2);

    /**
     * @brief depth - sets the number of times that the bounding box will be iteratively subdivided into octants. Total number of voxels = sum(8^(1:depth)).
     * @param maxDepth - (int) desired number of times that the bounding box will be iteratively subdivided into octants (default = 2, i.e. 72 vox).
     * @return true/false on success/failure of setting maxDepth
     */
    bool depth(1: i32 maxDepth = 2);

    /**
     * @brief setVerbose - sets verbose of the output on or off.
     * @param verb - (string ON/OFF) desired state of verbose.
     * @return true/false on success/failure of setting verbose.
     */
    bool setVerbose(1: string verb);
}
