#vecvec.thrift

struct RotationMatrix 
{
  1: list<double> content;
} 
(
  yarp.name = "yarp::sig::Matrix"
  yarp.includefile="yarp/sig/Matrix.h"
)

struct VoxFeat
{
1: list<double> voxHist;          # Every voxel is reprsented as a histogram.
}

struct ToolFeat3D
{   
1: string toolname;
2: list< VoxFeat> toolFeats;      # Every tool is reprsented as a vector of voxel histograms.
}

struct ToolFeat3DwithOrient
{   
1: string toolname;
2: list< VoxFeat> toolFeats;      # Every tool is reprsented as a vector of voxel histograms.
3: RotationMatrix orientation;    # Orientation with respect to the canonical position.
}
