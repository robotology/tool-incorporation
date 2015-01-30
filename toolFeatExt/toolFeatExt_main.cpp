#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/common/transforms.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>

#include <iostream>
#include <math.h>
#include <vector>
#include <ctime>


#include <boost/thread/thread.hpp>

int main (int argc, char** argv)
{
    float maxDepth;
    int binsPerDim;
    if (argc < 2){
        printf ("\n Please specify a .pcd file\n");
        return (0);
    } else if (argc == 2)   {
        maxDepth = 2;
        binsPerDim= 4;
    } else if (argc == 3)   {
        maxDepth = atof(argv[2]);
        binsPerDim= 4;
    } else if (argc == 4)   {
        maxDepth = atof(argv[2]);
        binsPerDim = atof(argv[3]);
    }
        
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
    if (pcl::io::loadPCDFile (argv[1], *source_cloud) == -1){
        printf ("\n .pcd file not valid\n");
        return (-1);
    }
    
    

    /* ===========================================================================*/ 
    // Transform (translate-rotate) the pointcloud by inverting the hand pose

    //Get hand pose
    // XXX getPose(xi,oi);
    // XXX for the moment lets use the known pose of the hoe

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    //Translate first to origin
    //transform.translation() << -xi(0), -xi(1), -xi(2);  
    transform.translation() << 0.480, -0.02565, -0.061;  


    // transform_2.rotate (XXX invert oi);

    // Print the transformation
    printf ("\nMethod #2: using an Affine3f\n");
    std::cout << transform.matrix() << std::endl;

    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*source_cloud, *cloud, transform);
    
    
    
    /* ===========================================================================*/ 
    // Get fixed bounding box to avoid octree automatically figure it out (what might change with resolution)
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();
    
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);

    /* =========================================================================== */
    // Divide the bounding box in subregions using octree, and compute the normal histogram (EGI) in each of those subregions at different levels.

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Create instances of necessary classes

    // octree
    float minVoxSize = 0.01; // Max resolution of 1 cm
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(minVoxSize);   //XXX let this be chageable via rpc.
    
    // Normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;         

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    
    // Cloud of Normals from whole pointcloud
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());

    // voxelCloud and voxelCloudNormals to contain the poitns on each voxel
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::Normal>::Ptr voxelCloudNormals(new pcl::PointCloud<pcl::Normal> ());

    // Viewer class for visualization
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Initialize octree
    //create octree structure
    octree.setInputCloud(cloud);
    //update bounding box automatically
    octree.defineBoundingBox(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z,
                             max_point_AABB.x, max_point_AABB.y, max_point_AABB.z);
    //add points in the tree
    octree.addPointsFromInputCloud();

    
    // Compute all normals from the orignial cloud, and THEN subdivide into voxels (to avoid voxel boundary problems arising when computing normals voxel-wise)
    // Clear output cloud
    cloud_normals->points.clear();
    
    // Pass the input cloud to the normal estimation class
    ne.setInputCloud (cloud);

    // Use all neighbors in a sphere of radius of size of the voxel length
    ne.setRadiusSearch (minVoxSize/2);

    // Compute the normals
    ne.compute (*cloud_normals);


    // Initialize histogram variables
    //float pi = 3.14159;

    float rangeMin = -1;    // XXX make this automatic.
    float rangeMax = 1;     //          ´´
    float rangeValid = rangeMax - rangeMin;
    float sizeBin = rangeValid/binsPerDim;

    std::vector < std::vector < float > > featureVectorOccVox;     // Vector containing histograms for occupied voxels
    std::vector < std::vector < float > > featureVectorAllVox;     // Vector containing histograms for all voxels, also the empty ones
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Find the minimum resolution octree (depth =1) to start increase it from there

    double cBB_min_x, cBB_min_y, cBB_min_z, cBB_max_x, cBB_max_y, cBB_max_z;
    octree.getBoundingBox(cBB_min_x, cBB_min_y, cBB_min_z, cBB_max_x, cBB_max_y, cBB_max_z);
    double cBBsize = fabs(cBB_max_x - cBB_min_x);           // size of the cubic bounding box encompasing the whole pointcloud

    std::cout << "AA BB: (" << min_point_AABB.x << "," << min_point_AABB.y << "," << min_point_AABB.z << "), (" << max_point_AABB.x << "," <<max_point_AABB.y << "," << max_point_AABB.z << ")" <<std::endl;
    std::cout << "Octree BB: (" << cBB_min_x << "," << cBB_min_y << "," <<cBB_min_z << "), (" << cBB_max_x << "," <<cBB_max_y << "," <<cBB_max_z << ")" <<std::endl;

    // re-initialize tree with new resolution
    minVoxSize = cBBsize/2;     //Update to minimum resolution
    octree.deleteTree();
    octree.setResolution(minVoxSize);
    octree.defineBoundingBox(cBB_min_x, cBB_min_y, cBB_min_z,
                             cBB_max_x, cBB_max_y, cBB_max_z);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Perform feature extraction iteratively on increasingly smaller voxels up to maxDepth level.
    for (int depth = 1; depth<=maxDepth; ++depth)
    {            
        std::cout << std::endl << "====================== Depth:" << depth << " ========================== " << std::endl;

        featureVectorOccVox.clear();                                // Clear the vector saving all populated histograms at depth level

        int voxPerSide = pow(2,depth);
        bool voxelOccupancy[voxPerSide][voxPerSide][voxPerSide];    // Keeps record of the position of the occupied voxels at each depth.
        for (int i = 0; i < voxPerSide; ++i)                        // initialize all positions to false
            for (int j = 0; j < voxPerSide; ++j)
                for (int k = 0; k < voxPerSide; ++k)
                   voxelOccupancy[i][j][k] = false;



        // Instantiate normal histogram at actual depth
        int numLeaves = octree.getLeafCount();      // Get the number of occupied voxels (leaves) at actual depth
        float voxelHist[numLeaves][binsPerDim][binsPerDim][binsPerDim]; // Histogram 3D matrix per voxel
        std::vector<float> featVecHist;                                 // Vector containing a seriazed version of the 3D histogram

        std::cout << "BB divided into " << voxPerSide << " voxels per side of size "<< minVoxSize << "," << numLeaves << " occupied." << std::endl;
        std::cout << "Looping through the  " << numLeaves << " occupied voxels." << std::endl;

        // Extracts all the points at leaf level from the octree        
        int octreePoints = 0;
        int voxelI = 0;
        int nanCount = 0;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::LeafNodeIterator tree_it;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::LeafNodeIterator tree_it_end = octree.leaf_end();

        // Iterate through lower level leafs (voxels).
        for (tree_it = octree.leaf_begin(depth); tree_it!=tree_it_end; ++tree_it)
        {
            //std::cout << "Finding points within voxel " << voxelI << std::endl;
            //std::cout << "Finding points within voxel " << voxelI << std::endl;

            // Clear clouds and vectors from previous voxel.
            voxelCloud->points.clear();
            voxelCloudNormals->points.clear();
            featVecHist.clear();

            // Find Voxel Bounds ...
            Eigen::Vector3f voxel_min, voxel_max;
            octree.getVoxelBounds(tree_it, voxel_min, voxel_max);            
            
            //... and center
            pcl::PointXYZ voxelCenter;
            voxelCenter.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
            voxelCenter.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
            voxelCenter.z = (voxel_min.z() + voxel_max.z()) / 2.0f;

            // Mark octree leaves as occupied voxels.
            int occx, occy, occz;                                  // voxel indices
            occx = floor((voxelCenter.x - cBB_min_x) /minVoxSize); // floor because 0-indexed array
            occy = floor((voxelCenter.y - cBB_min_y) /minVoxSize);
            occz = floor((voxelCenter.z - cBB_min_z) /minVoxSize);
            //std::cout << "Voxel (" << occx << "," << occy << "," << occz << ") is occupied" << std::endl;
            voxelOccupancy[occx][occy][occz]= true;
            
            //std::cout << "Voxel center at (" 
            //            << voxelCenter.x << " " 
            //            << voxelCenter.y << " " 
            //            << voxelCenter.z << ")" << std::endl;            

            // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            // Neighbors within voxel search
            // It assigns the search point to the corresponding leaf node voxel and returns a vector of point indices.
            // These indices relate to points which fall within the same voxel.
            std::vector<int> voxelPointsIdx;
            if (octree.voxelSearch (voxelCenter, voxelPointsIdx))
            {                      
                for (size_t i = 0; i < voxelPointsIdx.size (); ++i) 
                {                
                    // Save Points within voxel as cloud
                    voxelCloud->points.push_back(cloud->points[voxelPointsIdx[i]]);
                    voxelCloudNormals->points.push_back(cloud_normals->points[voxelPointsIdx[i]]);                    
                   
                    octreePoints++;
                } // for point in voxel
            }
            //std::cout << "Voxel has " << voxelCloud->points.size() << " points. " << std::endl;

        
            /* =================================================================================*/
            // On each voxel compute the EGI (sphere normal histogram). Normalize and send it out as feature vector.

            // Initialize histogram for new voxel
            for (int i = 0; i < binsPerDim; ++i)
                for (int j = 0; j < binsPerDim; ++j)
                    for (int k = 0; k < binsPerDim; ++k)
                       voxelHist[voxelI][i][j][k] = 0.0f;

            // Compute normal histogram looping through normals in voxel.
            unsigned int okNormalsVox = 0;
            for (size_t i = 0; i < voxelCloudNormals->points.size (); ++i)
            {
                int xbin, ybin, zbin;
                float xn, yn, zn;

                if ((pcl::isFinite(voxelCloudNormals->points[i]))){
                    xn = voxelCloudNormals->points[i].normal_x;
                    yn = voxelCloudNormals->points[i].normal_y;
                    zn = voxelCloudNormals->points[i].normal_z;
                    //std::cout << "Normal  (" << xn << "," << yn << "," << zn << ")" << std::endl;

                    xbin = floor((xn - rangeMin) /sizeBin); // floor because 0-indexed array
                    ybin = floor((yn - rangeMin) /sizeBin);
                    zbin = floor((zn - rangeMin) /sizeBin);
                    //std::cout << "lies on bin (" << xbin << "," << ybin << "," << zbin << ")" << std::endl;

                    voxelHist[voxelI][xbin][ybin][zbin]= voxelHist[voxelI][xbin][ybin][zbin] + 1.0f;
                    okNormalsVox ++;
                }else{
                    nanCount++;
                    //std::cout << "Normal is NaN." << std::endl;

                }
            }   //- close loop over normal points in voxel
            std::cout << "." ;

            //Normalize by the total number of point in voxel so that each histogram has sum 1, independent of the number of points in the voxel.
            for (int i = 0; i < binsPerDim; ++i){
                for (int j = 0; j < binsPerDim; ++j){
                    for (int k = 0; k < binsPerDim; ++k){
                       voxelHist[voxelI][i][j][k] = voxelHist[voxelI][i][j][k] / (float) okNormalsVox;

                       // Push into feature vector per histogram
                       featVecHist.push_back(voxelHist[voxelI][i][j][k]);
                    }
                }
            }

            // Print out angular histogram
            /*
            cout << "Angular histogram: " << std::endl << "{";
            for (int i = 0; i < binsPerDim; ++i){
                cout <<  "[" ;
                for (int j = 0; j < binsPerDim; ++j){
                    for (int k = 0; k < binsPerDim; ++k)
                       cout << voxelHist[voxelI][i][j][k] << "," ;
                    cout <<  std::endl;
                }
                cout <<  "]" ;
            }
            cout <<  "}" <<std::endl;
            // */

            // Keep all histograms form occupied voxels
            featureVectorOccVox.push_back(featVecHist);

            voxelI++;            

        } // close for voxel
        
        // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // Print out security checks

        // Check that all methods end with the same number of points, to avoid missing or repetition.
        std::cout << "Point Cloud has " << cloud->points.size() << " points." << std::endl;
        std::cout << "Normals Cloud has " << cloud->points.size() << " points, of which "<< nanCount << " are NaNs" << std::endl;
        std::cout << "Octree found " << octreePoints << " points." << std::endl;

        // Print out voxel occupancy and stream out histograms
        /*
        cout << "Voxel Occupancy: " << std::endl << "{";
        for (int i = 0; i < voxPerSide; ++i){
            cout <<  "[" ;
            for (int j = 0; j < voxPerSide; ++j){
                for (int k = 0; k < voxPerSide; ++k){
                   cout << voxelOccupancy[i][j][k] << "," ;
                }
                cout <<  std::endl;
            }
            cout <<  "]" ;
        }
        cout <<  "}" <<std::endl;
        //*/

        // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // Fill the complete feature vector, including histograms for empty voxels.
        int occVoxI = 0;
        for (int i = 0; i < voxPerSide; ++i){
            for (int j = 0; j < voxPerSide; ++j){
                for (int k = 0; k < voxPerSide; ++k){
                    if (voxelOccupancy[i][j][k]){        // if the voxel is occupied
                        std::vector<float> histVec;
                        histVec = featureVectorOccVox[occVoxI];
                        featureVectorAllVox.push_back(histVec); // copy histogram
                        occVoxI++;
                    }else{
                        std::vector < float >  zeroVector(pow(binsPerDim,3),0.0f);
                        featureVectorAllVox.push_back(zeroVector); // fill with the same amount of zeros
                    }
                }
            }
        }

        cout <<  "Vector has a size of " << featureVectorAllVox.size() << " x " << featVecHist.size() << " at depth "<< depth <<std::endl;

        // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // Re-initialize octree with double resolution (half minVoxSize) to analyse next level
        minVoxSize = minVoxSize/2;
        octree.deleteTree();
        octree.setResolution(minVoxSize);       // re-initialize tree with new resolution
        octree.defineBoundingBox(cBB_min_x, cBB_min_y, cBB_min_z,
                                 cBB_max_x, cBB_max_y, cBB_max_z);
        octree.setInputCloud(cloud);
        octree.addPointsFromInputCloud();
    }  
    std::cout << std::endl << "======================Feature Extraction Done========================== " << std::endl;

    /*============================================================================*/
    // Visualization         
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (0.05);
    viewer->initCameraParameters ();
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
           
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 25, 0.008, "normals");
           
    while(!viewer->wasStopped())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
  
  

