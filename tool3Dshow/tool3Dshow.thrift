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

    // showFileCloud - Opens the visualizer and displays a PointCloud from a file
    // @return true/false on showing the poitncloud
    bool showFileCloud(1: string cloudname);

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
