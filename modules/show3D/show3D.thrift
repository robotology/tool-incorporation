#show3D.thrift

#include "objects3D.h"

/**
* show3D_IDLServer Interface.
*/

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
     * @return true/false on showing the poitncloud
     */
    bool addNormals(1: double radSearch = 0.03);

    /**
     * @brief addBoundingBox - adds the bounding box to the displayed cloud. If minBB is true, it will be the minimum BB, otherwise the axis-aligned one.
     * @param minBB - (bool) true to compute the minimum bounding box, false to compute the axis-aligned bounding box (default = false).
     * @return true/false on showing the poitncloud
     */
    bool addBoundingBox(1: bool minBB = false);


    /**
     * @brief showCloud - Displays the received mesh as pointcloud on the visualizer.
     * @param mesh - (SurfaceMeshWithBoundingBox) mesh to be displayed.
     * @return true/false on showing the poitncloud
     */
     //bool showCloud(1: SurfaceMeshWithBoundingBox mesh);

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
