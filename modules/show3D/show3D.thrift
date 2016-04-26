#show3D.thrift

#include "objects3D.h"

/**
* show3D_IDLServer Interface.
*/

struct Bottle{}
(
    yarp.name = "yarp::os::Bottle"
    yarp.includefile="yarp/os/Bottle.h"
)


service show3D_IDLServer
{
    // setVerbose (ON/OFF) - sets verbose of the output on or off
    // @return true/false on success/failure of setting verbose
    //bool setName(1: string name);


    /**
    * @brief clearVis - Clears all clouds and effects from the visualizer.
    * @return true/false on success/failure of extracting features
    */
    bool clearVis();

    /**
    * @brief accumClouds - Selects between plotting clouds together or not
    * @param accum - (bool) true to plot clouds together and false to clear the visualizer for every new cloud (default =false)
    * @return true/false on success/failure of setting verbose
    */
    bool accumClouds(1: bool accum = false);

    /**
     * @brief showFileCloud - Opens the visualizer and displays a PointCloud from a file
     * @param cloudname - (string) name of the .ply or .pcd file containg the desired cloud to be shown (default "cloud.ply")
     * @return true/false on showing the poitncloud
     */
    bool showFileCloud(1: string cloudname = "cloud.ply");

    /**
     * @brief addNormals - adds Normals to the displayed cloud radSearch is used to find neighboring points.
     * @param radSearch - (double) value (in meters) of the extension of the radius search in order to estimate the surface to compute normals from (default = 0.03).
     * @param normalColors - (bool) Expresses whether the normals should be expressed as a vector (false), or as color gradients on the cloud (true).
     * @return true/false on showing the poitncloud
     */
    bool addNormals(1: double radSearch = 0.01, 2: bool normCol = true);
    
    /**
     * @brief addFeats - adds a visualization of the OMS-EGI features (voxel based spherical normal histograms).
     * @param res - (double) value (in meters) side of the voxel grid on which the OMS-EGI featureas are copmuted (default = 0.01).     
     * @param plotHist - (bool) Determines whether the average value of the normal histogram should be shown inside each voxel as a sphere (default = true) or not (false).
     * @return true/false on showing the poitncloud
     */
    bool addFeats(1: double res = 0.01, 2: bool plotHist = true);

    /**
     * @brief addBoundingBox - adds the bounding box to the displayed cloud. If minBB is true, it will be the minimum BB, otherwise the axis-aligned one.
     * @param tpyeBB - (int) 0 to compute the minimum bounding box, 1 to compute the axis-aligned bounding box, 2 to compute Cubic AABB (default = 2),
     * @return true/false on showing the poitncloud
     */
    bool addBoundingBox(1: i32 typeBB = 0);


    /**
     * @brief addArrow - Plots arrow with given start and end coords and color
     * @param arrowStart - start coords
     * @param arrowEnd - end coords
     * @param color - color rgb
     * @return true/false on displaying the arrow the poitncloud
     */
    //bool addArrow(1: list<double> arrowStart, 2: list<double> arrowEnd, 3: list<i32> color);
     //bool showCloud(1: SurfaceMeshWithBoundingBox mesh);


    /**
     * @brief addArrow - Plots arrow with given start and end coords and color
     * @param coords - sphere coords
     * @param color - color rgb
     * @return true/false on displaying the sphere
     */

    bool addSphere(1: list<double> coords, 2: list<i32> color);
     //bool showCloud(1: SurfaceMeshWithBoundingBox mesh);

    /**
     * @brief filter - Function to apply and show different filtering processes to the displayed cloud.
     * @param ror - bool: Activates RadiusOutlierRemoval (rad = 0.05, minNeigh = 5).
     * @param sor - bool: Activates StatisticalOutlierRemoval (meanK = 20).
     * @param mls - bool: Activates MovingLeastSquares (rad = 0.02, order 2, usRad = 0.005, usStep = 0.003)
     * @param ds - bool: Activates Voxel Grid Downsampling (rad = 0.002).
     * @return true/false on showing the poitnclouddar =
     */
     bool filter(1: bool ror = false, 2: bool sor = false, 3: bool mks = false, 4: bool ds = false);

    /**
     * @brief setVerbose (ON/OFF) - sets verbose of the output on or off
     * @return true/false on success/failure of setting verbose
     */
    // bool setVerbose(1: string verb);

    /**
     * @brief quit - quits the module
     * @return true/false on success/failure of extracting features
     */
    bool quit();

}
