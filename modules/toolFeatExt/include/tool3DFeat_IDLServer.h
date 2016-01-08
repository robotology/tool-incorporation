// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_tool3DFeat_IDLServer
#define YARP_THRIFT_GENERATOR_tool3DFeat_IDLServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <Point3D.h>
#include <yarp/sig/Matrix.h>

class tool3DFeat_IDLServer;


/**
 * tool3DFeat_IDLServer Interface.
 */
class tool3DFeat_IDLServer : public yarp::os::Wire {
public:
  tool3DFeat_IDLServer();
  /**
   * @brief getFeats - Performs 3D feature extraction of the tool in the rotated pose.
   * @return true/false on success/failure of extracting features
   */
  virtual bool getFeats();
  /**
   * @brief getFeats - Performs 3D feature extraction of the tool in the rotated pose.
   * @return true/false on success/failure of extracting features
   */
  virtual bool getAllToolFeats(const std::string& robot = "real");
  /**
   * @brief getToolTip - Returns the tooltip (as the center of the edge opposite edge of the hand the bounding box)
   * @return true/false on success/failure of computing the tooltip
   */
  virtual Point3D getToolTip();
  /**
   * @brief getSamples - Generates n poses of the tool around a base orientation deg.
   * @param n - (int) desired number of poses (default = 10) .
   * @param deg - (double) base orientation of the tool pose (default = 0.0).
   * @return true/false on success/failure of generating n samples.
   */
  virtual bool getSamples(const int32_t n = 10, const double deg = 0);
  /**
   * @brief loadModel - loads a model from a .pcd or .ply file for further processing.
   * @param cloudname - (string) name of the file to load cloud from (and path from base path if needed) (default = "cloud.ply") .
   * @return true/false on success/failure loading the desired cloud.
   */
  virtual bool loadModel(const std::string& cloudname = "cloud.ply");
  /**
   * @brief setPose - Rotates the tool model according to any given rotation matrix
   * @param rotationMatrix - (yarp::sig::Matrix) rotation matrix to apply to the cloud.
   * @return true/false on success/failure of rotating model according to  matrix.
   */
  virtual bool setPose(const yarp::sig::Matrix& toolPose);
  /**
   * @brief setCanonicalPose - Rotates the tool model to canonical orientations, i.e. as a rotation along the longest axis which orients the end effector.
   * @param deg - (double) degrees of rotation along the longest axis.
   * @param disp - (int) displacement along the tool axis (grasped closer or further from end-effector).
   * @return true/false on success/failure of rotating model according to orientation.
   */
  virtual bool setCanonicalPose(const double deg = 0, const double disp = 0, const double tilt = 45);
  /**
   * @brief bins - sets the number of bins per angular dimension (yaw-pitch-roll) used to compute the normal histogram. Total number of bins per voxel = bins^3.
   * @param nbins - (int) desired number of bins per angular dimension. (default = 2, i.e. 8 bins per voxel).
   * @return true/false on success/failure of setting number of bins.
   */
  virtual bool bins(const int32_t nbins = 2);
  /**
   * @brief depth - sets the number of times that the bounding box will be iteratively subdivided into octants. Total number of voxels = sum(8^(1:depth)).
   * @param maxDepth - (int) desired number of times that the bounding box will be iteratively subdivided into octants (default = 2, i.e. 72 vox).
   * @return true/false on success/failure of setting maxDepth
   */
  virtual bool depth(const int32_t maxDepth = 2);
  /**
   * @brief setVerbose - sets verbose of the output on or off.
   * @param verb - (string ON/OFF) desired state of verbose.
   * @return true/false on success/failure of setting verbose.
   */
  virtual bool setVerbose(const std::string& verb);
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

