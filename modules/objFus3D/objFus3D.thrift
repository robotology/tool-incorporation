#objFus3D.thrift


/**
* objFus3D_IDLServer Interface.
*/

service objFus3D_IDLServer
{
    /**
    * save (string name) - saved the current merged cloud into file of given name
    * @return true/false on success/failure of saving cloud
    */
    bool save(1: string name = "model");
    
    /**
    * @brief track - ask the user to select the bounding box and starts tracker on template
    * @return true/false on success/failure of starting tracker
    */
    bool track();

    /**
    * @brief mls - sets parameters for moving least squares filtering (def 0.03)
    * @param rad: radius used for determining the k-nearest neighbors used for fitting (def 0.005)
    * @param usRad: radius of the circle in the local point plane that will be sampled for upsampling (def 0.003).
    * @param usStep: step size for the local plane sampling for upsampling.
    * @return true/false on success/failure setting parameters
    */
    bool mls(1:double rad, 2:double usRad, 3:double usStep);

    /**
    * @brief ds - sets parameters for downsampling
    * @param res: 3D grid leaf size (downsampling resolution) (def 0.002).
    * @return true/false on success/failure setting parameters
    */
    bool ds(1:double res);

    /**
    * @brief icp - sets parameters for iterative closes algorithm
    * @param maxIt: Maximum number of Iterations (def 100)
    * @param maxCorr: Max distance between clouds to consider correspondence successful [m] (def 0.03);
    * @param ranORT: Inlier distance threshold for the internal RANSAC outlier rejection loop [m] (def 0.03).
    * @param transEp: Transformation epsilon to stop icp iterations (1e-6).
    * @return true/false on success/failure of setting parameters
    */
    bool icp(1:i32 maxIt, 2:double maxCorr, 3:double ranORT, 4:double transEps );    

    /**
    * @brief restart - Clears all clouds and visualizer, restarts tracker and restarts a new reconstruction.
    * @return true/false on success/failure of cleaning and restarting
    */
    bool restart();

    /**
    * @brief pause - Pauses recosntruction/merging until it is called again.
    * @return true/false on success/failure of pausing.
    */
    bool pause();

    /**
     * @brief verb - switches verbose between ON and OFF
     * @return true/false on success/failure of setting verbose
     */
    bool verb();

    /**
    * @brief initAlign - Set initial feature based alignment ON/OFF
    * @return true/false on success/failure of (de)activating alingment.
    */
    bool initAlign();

    /**
     * @brief quit - quits the module
     * @return true/false on success/failure of extracting features
     */
    bool quit();

}
