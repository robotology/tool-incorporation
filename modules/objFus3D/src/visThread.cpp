#include "visThread.h"

using namespace std;
using namespace yarp::os;

// Empty constructor
VisThread::VisThread(int period, const string &_cloudname):RateThread(period), id(_cloudname){}

// Initialize Variables
bool VisThread::threadInit()
{
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); // Point cloud
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer("Point Cloud Viewer")); //viewer
    viewer->setSize(900,600);
    viewer->setPosition(320,370);
    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, vp1);
    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, vp2);
    viewer->setCameraPosition(0.5, 0.0, 0.3,0,0,0 );

    //initialize here variables
    printf("\nStarting visualizer Thread\n");

    // Flags
    initialized = false;
    clearing = false;
    updatingCloud = false;
    visNum = 0;

    return true;
}

// Module and visualization loop
void VisThread::run()
{
    if (initialized)
    {
        if (!viewer->wasStopped ())
        {
            viewer->spinOnce ();
            // Get lock on the boolean update and check if cloud was updated
            boost::mutex::scoped_lock updateLock(updateModelMutex);
            if(update)
            {
                if(updatingCloud){

                    // Determine the corresponding id and viewport
                    string idC;
                    if (visNum == 0){
                        idC = "raw";
                        vp = vp1;
                    }else if (visNum==1){
                        idC = "merged";
                        vp = vp2;
                    }

                    // Clean visualizer to plot new cloud
                    viewer->removePointCloud(id,vp);
                    viewer->removePointCloud("normals",vp);
                    viewer->removeAllShapes(vp);

                    // Check if the loaded file contains color information or not
                    bool colorCloud = false;
                    for  (int p = 1; p < cloud->points.size(); ++p)      {
                        uint32_t rgb = *reinterpret_cast<int*>(&cloud->points[p].rgb);
                        if (rgb != 0)
                            colorCloud = true;
                    }

                    // Add cloud color if it has not
                    if (!colorCloud)    {
                        // Define R,G,B colors for the point cloud
                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler(cloud, 230, 20, 0); //red
                        viewer->addPointCloud(cloud, cloud_color_handler, idC, vp);
                    }else{
                        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloud_color_handler (cloud);
                        viewer->addPointCloud(cloud, cloud_color_handler, idC, vp);
                    }

                    // Plot cloud
                    //viewer->addPointCloud(cloud, cloud_color_handler, idC, visNum);
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id,vp);
                    updatingCloud = false;
                }

                // Clear display
                if (clearing){
                    viewer->removePointCloud(id,0);
                    viewer->removePointCloud("normals",0);
                    viewer->removeAllShapes();
                    clearing = false;
                }

                update = false;
            }
            updateLock.unlock();
        }else{
            //Close viewer
            printf("Closing Visualizer\n");            
            viewer->removePointCloud(id,0);
            viewer->close();
            this->stop();
        }

    }
}

void VisThread::threadRelease()
{
    printf("Closing Visualizer Thread\n");
}

// Unlock mutex temporarily to allow an update cycle on the visualizer
void VisThread::updateVis()
{
    // Lock mutex while update is on
    boost::mutex::scoped_lock updateLock(updateModelMutex);
    update = true;
    updateLock.unlock();

    printf("Visualizer updated\n");
}

// Clear display
void VisThread::clearVisualizer()
{
    clearing = true;
    updateVis();
}

// Display new cloud received
void VisThread::updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const string &name, int vN)
{
    cout << "Updating Cloud " << name << " on viewport " << vN << endl;
    cloud->points.clear();
    cloud->clear();
    *cloud = *cloud_in;  // new cloud overwrites last one
    id = name;
    visNum = vN;
    if (!initialized)
    {
        // Set camera position and orientation
        //viewer->setBackgroundColor (0.05, 0.05, 0.05, 0); // Setting background to a dark grey
        viewer->setBackgroundColor (1,1,1, vp1); // Setting background to white
        viewer->setBackgroundColor (0.9,0.9,0.9, vp2); // Setting background to white-ish
        viewer->addCoordinateSystem (0.05, "coord", 0);
        initialized = true;

        cout << "Visualizer viewport " << vN << " initialized.";
    }

    updatingCloud = true;
    updateVis();
    printf("Cloud updated\n");
}
