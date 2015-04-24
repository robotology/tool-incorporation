// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_tool3Dshow_IDLServer
#define YARP_THRIFT_GENERATOR_tool3Dshow_IDLServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class tool3Dshow_IDLServer;


/**
 * tool3DShow_IDLServer Interface.
 */
class tool3Dshow_IDLServer : public yarp::os::Wire {
public:
  tool3Dshow_IDLServer();
  /**
   * @brief clearVis - Clears all clouds and effects from the visualizer.
   * @return true/false on success/failure of extracting features
   */
  virtual bool clearVis();
  /**
   * @brief accumClouds - Selects between plotting clouds together or not
   * @param accum - (bool) true to plot clouds together and false to clear the visualizer for every new cloud (default =false)
   * @return true/false on success/failure of setting verbose
   */
  virtual bool accumClouds(const bool accum = 0);
  /**
   * @brief showFileCloud - Opens the visualizer and displays a PointCloud from a file
   * @param cloudname - (string) name of the .ply or .pcd file containg the desired cloud to be shown (default "cloud.ply")
   * @return true/false on showing the poitncloud
   */
  virtual bool showFileCloud(const std::string& cloudname = "cloud.ply");
  /**
   * @brief addNormals - adds Normals to the displayed cloud radSearch is used to find neighboring points.
   * @param radSearch - (double) value (in meters) of the extension of the radius search in order to estimate the surface to compute normals from (default = 0.03).
   * @return true/false on showing the poitncloud
   */
  virtual bool addNormals(const double radSearch = 0.03);
  /**
   * @brief addBoundingBox - adds the bounding box to the displayed cloud. If minBB is true, it will be the minimum BB, otherwise the axis-aligned one.
   * @param minBB - (bool) true to compute the minimum bounding box, false to compute the axis-aligned bounding box (default = false).
   * @return true/false on showing the poitncloud
   */
  virtual bool addBoundingBox(const bool minBB = 0);
  /**
   * @brief quit - quits the module
   * @return true/false on success/failure of extracting features
   */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

