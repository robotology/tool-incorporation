#tool3Dshow.thrift

#include "objects3D.h"

/**
* tool3DShow_IDLServer Interface.
*/

service tool3Dshow_IDLServer
{
    // setVerbose (ON/OFF) - sets verbose of the output on or off
    // @return true/false on success/failure of setting verbose
    //bool setName(1: string name);


    // help - produces help with longer descritpion of each command and its parameters.
    // @return true/false on success/failure of extracting features
    bool clearVis();

    // accumClouds - Selects between plotting clouds together or not
    // @return true/false on success/failure of setting verbose
    bool accumClouds(1: bool accum = false);

    // showFileCloud - Opens the visualizer and displays a PointCloud from a file
    // @return true/false on showing the poitncloud
    bool showFileCloud(1: string cloudname = "cloud_merged.ply");

    // addNormals - adds Normals to the displayed cloud radSearch is used to find neighboring points.
    // @return true/false on showing the poitncloud
    bool addNormals(1: double radSearch = 0.03);

    // addBoundingBox - adds the bounding box to the displayed cloud. If minBB is true, it will be the minimum BB, otherwise the axis-aligned one.
    // @return true/false on showing the poitncloud
    bool addBoundingBox(1: bool minBB = false);


    // showCloud - Opens the visualizer and displays the PointCloud received via thrift
    // @return true/false on showing the poitncloud
    #bool showCloud(1: objects3D.SurfaceMeshWithBoundingBox mesh);
    //bool showCloud(1: SurfaceMeshWithBoundingBox mesh);

    // setVerbose (ON/OFF) - sets verbose of the output on or off
    // @return true/false on success/failure of setting verbose
    #bool setVerbose(1: string verb);

    // help - produces help with longer descritpion of each command and its parameters.
    // @return true/false on success/failure of extracting features
    string help_commands();

    // quit - quits the module
    // @return true/false on success/failure of extracting features
    bool quit();

}
