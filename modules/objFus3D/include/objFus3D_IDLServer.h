// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_objFus3D_IDLServer
#define YARP_THRIFT_GENERATOR_objFus3D_IDLServer

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class objFus3D_IDLServer;


/**
 * objFus3D_IDLServer Interface.
 */
class objFus3D_IDLServer : public yarp::os::Wire {
public:
  objFus3D_IDLServer();
  /**
   * save (string name) - saved the current merged cloud into file of given name
   * @return true/false on success/failure of saving cloud
   */
  virtual bool save(const std::string& name = "model");
  /**
   * @brief track - ask the user to select the bounding box and starts tracker on template
   * @return true/false on success/failure of starting tracker
   */
  virtual bool track();
  /**
   * @brief mls - sets parameters for moving least squares filtering (def 0.03)
   * @param rad: radius used for determining the k-nearest neighbors used for fitting (def 0.005)
   * @param usRad: radius of the circle in the local point plane that will be sampled for upsampling (def 0.003).
   * @param usStep: step size for the local plane sampling for upsampling.
   * @return true/false on success/failure setting parameters
   */
  virtual bool mls(const double rad, const double usRad, const double usStep);
  /**
   * @brief ds - sets parameters for downsampling
   * @param res: 3D grid leaf size (downsampling resolution) (def 0.002).
   * @return true/false on success/failure setting parameters
   */
  virtual bool ds(const double res);
  /**
   * @brief icp - sets parameters for iterative closes algorithm
   * @param maxIt: Maximum number of Iterations (def 100)
   * @param maxCorr: Max distance between clouds to consider correspondence successful [m] (def 0.03);
   * @param ranORT: Inlier distance threshold for the internal RANSAC outlier rejection loop [m] (def 0.03).
   * @param transEp: Transformation epsilon to stop icp iterations (1e-6).
   * @return true/false on success/failure of setting parameters
   */
  virtual bool icp(const int32_t maxIt, const double maxCorr, const double ranORT, const double transEps);
  /**
   * @brief restart - Clears all clouds and visualizer, restarts tracker and restarts a new reconstruction.
   * @return true/false on success/failure of cleaning and restarting
   */
  virtual bool restart();
  /**
   * @brief pause - Pauses recosntruction/merging until it is called again.
   * @return true/false on success/failure of pausing.
   */
  virtual bool pause();
  /**
   * @brief verb - switches verbose between ON and OFF
   * @return true/false on success/failure of setting verbose
   */
  virtual bool verb();
  /**
   * @brief initAlign - Set initial feature based alignment ON/OFF
   * @return true/false on success/failure of (de)activating alingment.
   */
  virtual bool initAlign();
  /**
   * @brief quit - quits the module
   * @return true/false on success/failure of extracting features
   */
  virtual bool quit();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif

