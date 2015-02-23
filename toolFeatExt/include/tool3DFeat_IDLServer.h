// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_tool3DFeat_IDLServer
#define YARP_THRIFT_GENERATOR_tool3DFeat_IDLServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <yarp/sig/Matrix.h>

class tool3DFeat_IDLServer;


/**
 * tool3DFeat_IDLServer Interface.
 */
class tool3DFeat_IDLServer : public yarp::os::Wire {
public:
  tool3DFeat_IDLServer();
  virtual bool getFeats();
  virtual bool setPose(const yarp::sig::Matrix& toolPose);
  virtual bool setCanonicalPose(const int32_t deg = 0);
  virtual bool bins(const int32_t nbins = 2);
  virtual bool depth(const int32_t maxDepth = 2);
  virtual bool setVerbose(const std::string& verb);
  virtual bool setName(const std::string& cloudname = "cloud_merged.ply");
  virtual bool help_commands();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

