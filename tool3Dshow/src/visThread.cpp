#include "visThread.h"

using namespace std;
using namespace yarp::os;

VisThread::VisThread(int period, const string &_cloudname):RateThread(period), id(_cloudname)
{

}

bool VisThread::threadInit()
{
    cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>); // Point cloud
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer("Point Cloud Viewer")); //viewer
    
    //initialize here variables
    printf("\nStarting visualizer Thread\n");

    initialized = false;
    addClouds = false;

    return true;
}


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
                if(!viewer->updatePointCloud(cloud,id)){
                    // viewer->addPointCloud(cloud, colorHandler, "Cloud");

                    // check if the loaded file contains color information or not
                    bool colorCloud = false;
                    for  (int p = 1; p < cloud->points.size(); ++p)      {
                        uint32_t rgb = *reinterpret_cast<int*>(&cloud->points[p].rgb);
                        if (rgb != 0)
                            colorCloud = true;
                    }
                    // Add cloud color if it has not
                    if (!colorCloud)    {
                        // Define R,G,B colors for the point cloud
                        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler(cloud, 230, 20, 0); //white
                        viewer->addPointCloud (cloud, cloud_color_handler, id);
                    }else{
                        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> cloud_color_handler (cloud);
                        viewer->addPointCloud (cloud, cloud_color_handler, id);
                    }
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, id);


                }
                update = false;
            }
            updateLock.unlock();
        }
        // Close viewer
        //viewer->close();
        //viewer->removePointCloud(id);
        //this->stop();
    }
}

void VisThread::threadRelease()
{
    printf("Closing Visualizer Thread\n");
}


void VisThread::updateCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
    printf("Updating displayed cloud\n");
    if (addClouds)
        *cloud += *cloud_in; // new cloud is added to last one
    else
        *cloud = *cloud_in;  // new cloud overwrites last one

    if (!initialized)
    {

        // Set camera position and orientation
        viewer->setBackgroundColor (0.05, 0.05, 0.05, 0); // Setting background to a dark grey
        viewer->addCoordinateSystem (0.05);
        initialized = true;
    }

    // Lock mutex while update is on
    boost::mutex::scoped_lock updateLock(updateModelMutex);
    update = true;
    updateLock.unlock();
    
}

void VisThread::switchAddClouds()
{
    if (addClouds){
        addClouds = false;
        printf("Clouds plotted separately now\n");
    }else{
        addClouds = true;
        printf("Clouds plotted together now\n");
    }
}

