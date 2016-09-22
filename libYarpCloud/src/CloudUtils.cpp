
#include <iCub/YarpCloud/CloudUtils.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::YarpCloud; 

/************************************************************************/
bool CloudUtils::loadCloud(const string& cloudpath, const string& cloudname, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to)
{
    cloud_to->clear();

    cout << "Attempting to load " << (cloudpath + cloudname).c_str() << "... "<< endl;
    // Load the pointcloud either from a pcd or a ply file.
    DIR *dir;

    // Check that directory exists
    if ((dir = opendir (cloudpath.c_str())) == NULL) {
        printf ("can't open data directory.");
        return false;
    }

    // Check that the file has a type
    string::size_type idx;
    idx = cloudname.rfind('.');
    if(idx != std::string::npos)
    {
        // Check that the file type is valid
        string ext = cloudname.substr(idx+1);
        cout << "Extension found: " << ext << endl;

        if(strcmp(ext.c_str(),"ply")==0)        // Check if it is .ply
        {
            printf ("Loading .ply file: %s\n", cloudname.c_str());
            if (pcl::io::loadPLYFile (cloudpath + cloudname, *cloud_to) < 0)	{
                PCL_ERROR("Error loading cloud %s.\n", cloudname.c_str());
                return false;
            }

        }else if(strcmp(ext.c_str(),"pcd")==0) // Check if it is .pcd
        {
            printf ("Loading .pcd file: %s\n", cloudname.c_str());
            if (pcl::io::loadPCDFile (cloudpath + cloudname, *cloud_to) < 0)	{
                PCL_ERROR("Error loading cloud %s.\n", cloudname.c_str());
                return false;
            }

        }else if(strcmp(ext.c_str(),"off")==0) // Check if it is .off
            {
                printf ("Loading .off file: %s\n", cloudname.c_str());
                if (!loadOFFFile (cloudpath + cloudname, cloud_to))	{
                    PCL_ERROR("Error loading cloud %s.\n", cloudname.c_str());
                    return false;
                }
        }else if(strcmp(ext.c_str(),"coff")==0) // Check if it is .off
            {
                printf ("Loading .off file: %s\n", cloudname.c_str());
                if (!loadOFFFile (cloudpath + cloudname, cloud_to))	{
                    PCL_ERROR("Error loading cloud %s.\n", cloudname.c_str());
                    return false;
                }

        }else {
            PCL_ERROR("Please select a .pcd , .ply or .off file.\n");
            return false;
        }

    }else{
        string cloudnameExt;
        PCL_ERROR(" Name given without format.\n");

        cout << "-> Trying with .pcd" << endl;
        cloudnameExt = cloudname +  ".pcd";
        if (pcl::io::loadPCDFile (cloudpath + cloudnameExt, *cloud_to) >= 0)	{
            cout << "Cloud loaded from file "<< cloudnameExt << endl;
            return true;
        }

        cout << "-> Trying with .ply" << endl;
        cloudnameExt = cloudname +  ".ply";
        if (pcl::io::loadPLYFile (cloudpath + cloudnameExt, *cloud_to) >= 0)	{
            cout << "Cloud loaded from file "<< cloudnameExt << endl;
            return true;
        }

        cout << "-> Trying with .off" << endl;
        cloudnameExt = cloudname +  ".off";
        if (loadOFFFile (cloudpath + cloudnameExt, cloud_to))	{
            cout << "Cloud loaded from file "<< cloudnameExt << endl;
            return true;
        }
        cout << "-> Trying with .coff" << endl;
        cloudnameExt = cloudname +  ".coff";
        if (loadOFFFile (cloudpath + cloudnameExt, cloud_to))	{
            cout << "Cloud loaded from file "<< cloudnameExt << endl;
            return true;
        }

        PCL_ERROR("Couldnt find .pcd, .ply or .(c)off cloud.\n");
        return false;
    }

    return true;
}


/************************************************************************/
bool CloudUtils::loadOFFFile(const string& cloudpath, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to)
{
    ifstream cloudFile(cloudpath.c_str());
    cloud_to->points.clear();
    cloud_to->clear();
    int nPoints;
    int state=0;
    char line[255];
    bool color = false;

    if (!cloudFile.is_open())
    {
       PCL_ERROR("problem opening point cloud file!");
       return false;
    }

    while (!cloudFile.eof())
    {
       cloudFile.getline(line,sizeof(line),'\n');
       yarp::os::Bottle b(line);
       yarp::os::Value firstItem = b.get(0);
       bool isNumber = firstItem.isInt() || firstItem.isDouble();

       if (state==0)
       {
           string tmp = firstItem.asString().c_str();
           std::transform(tmp.begin(),tmp.end(),tmp.begin(),::toupper);
           if (tmp == "OFF")
               state++;
           if (tmp == "COFF"){
               color = true;
               state++;
           }
       }
       else if (state==1)
       {
           if (isNumber)
           {
               nPoints=firstItem.asInt();
               state++;
           }
       }
       else if (state==2)
       {
           if (isNumber && (b.size()>=3))
           {
               pcl::PointXYZRGB point;
               point.x = b.get(0).asDouble();
               point.y = b.get(1).asDouble();
               point.z = b.get(2).asDouble();

               if (color)
               {
                   point.r = b.get(3).asDouble();
                   point.g = b.get(4).asDouble();
                   point.b = b.get(5).asDouble();
                   //cout << "Point has color: (" << point.r << ", "<< point.g << ", "<< point.b << ")."<< endl;

               } else{
                   point.rgb=0;
                   point.r=255;
                   point.g=0;
                   point.b=0;
               }

               cloud_to->push_back(point);

               if (--nPoints<=0)
                   return true;
           }
       }
    }
    return false;
}




/************************************************************************/
void CloudUtils::savePointsPly(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const string& savepath, const string& savename, int addNum)
{
    stringstream s;
    s.str("");
    if (addNum >= 0){
        s << savepath + "/" + savename.c_str() << addNum;
        addNum++;
    } else {
        s << savepath + "/" + savename.c_str();
    }

    string filename = s.str();
    string filenameNumb = filename+".ply";
    ofstream plyfile;
    plyfile.open(filenameNumb.c_str());
    plyfile << "ply\n";
    plyfile << "format ascii 1.0\n";
    plyfile << "element vertex " << cloud->width <<"\n";
    plyfile << "property float x\n";
    plyfile << "property float y\n";
    plyfile << "property float z\n";
    plyfile << "property uchar diffuse_red\n";
    plyfile << "property uchar diffuse_green\n";
    plyfile << "property uchar diffuse_blue\n";
    plyfile << "end_header\n";

    for (unsigned int i=0; i<cloud->width; i++)
        plyfile << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " " << (int)cloud->at(i).r << " " << (int)cloud->at(i).g << " " << (int)cloud->at(i).b << "\n";

    plyfile.close();

    cout << "Cloud saved in file: " << filenameNumb.c_str() << endl;
    return;
}

/************************************************************************/
void CloudUtils::savePointsOff(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const string& savepath, const string& savename, int &addNum)
{
    stringstream s;
    s.str("");
    if (addNum >= 0){
        s << savepath + "/" + savename.c_str() << addNum;
        addNum++;
    } else {
        s << savepath + "/" + savename.c_str();
    }

    string filename = s.str();
    string filenameNumb = filename+".off";
    ofstream file;
    file.open(filenameNumb.c_str());
    if (file.is_open())
    {
        file<<"COFF"<<endl;
        file<<cloud->size()<<" 0 0"<<endl;
        file<<endl;
        for (size_t i=0; i<cloud->size(); i++)
        {
            file << cloud->at(i).x << " " << cloud->at(i).y << " " << cloud->at(i).z << " "
                    << (int)cloud->at(i).r << " " << (int)cloud->at(i).g << " " << (int)cloud->at(i).b << endl;
        }
        file<<endl;

    } else {
        cout<<"Some problems in opening output file!";
    }

    file.close();

    cout << "Cloud saved in file: " << filenameNumb.c_str() << endl;
    return;
}

/************************************************************************/
void CloudUtils::bottle2cloud(const yarp::os::Bottle& cloudB, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{   // Converts cloud in a bottle into pcl pointcloud.
    int bsize=cloudB.size();
    for(size_t i=0; i<bsize; i++)
    {
        yarp::os::Bottle *pointList=cloudB.get(i).asList();

        pcl::PointXYZRGB point;
        point.x=pointList->get(0).asDouble();
        point.y=pointList->get(1).asDouble();
        point.z=pointList->get(2).asDouble();

        //cout << "Received point with coordinates: (" << point.x << ", "<< point.y << ", "<< point.z << ")."<< endl;

        if (pointList->size()>3)
        {            
            point.r = pointList->get(3).asDouble();
            point.g = pointList->get(4).asDouble();
            point.b = pointList->get(5).asDouble();
            //cout << "Point has color: (" << point.r << ", "<< point.g << ", "<< point.b << ")."<< endl;
        } else{
            point.rgb=0;
            point.r=255;
            point.g=0;
            point.b=0;
        }
        cloud->push_back(point);
    }

    printf("Bottle formatted into Point Cloud \n");
}


/************************************************************************/
void CloudUtils::cloud2bottle(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, yarp::os::Bottle& cloudB)
{   // Converts pointcloud to surfaceMesh bottle.
    cloudB.clear();
    for (unsigned int i=0; i<cloud->width; i++)
    {
        yarp::os::Bottle &bpoint = cloudB.addList();
        pcl::PointXYZRGB p = cloud->at(i);
        bpoint.addDouble(p.x);
        bpoint.addDouble(p.y);
        bpoint.addDouble(p.z);
        if (p.rgb!=0)
        {
            // unpack rgb into r/g/b
            uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
            uint8_t r = (rgb >> 16) & 0x0000ff;
            bpoint.addInt(p.r);
            uint8_t g = (rgb >> 8)  & 0x0000ff;
            bpoint.addInt(p.g);
            uint8_t b = (rgb)       & 0x0000ff;
            bpoint.addInt(p.b);
        }
    }
    printf("Point Cloud formatted into Bottle\n");
    return;
}

/************************************************************************/
/*
void CloudUtils::getBB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, iCub::data3D::BoundingBox BB)
{
    BB = iCub::data3D::MinimumBoundingBox::getMinimumBoundingBox(cloud);
    return;
}
*/

/************************************************************************/
Matrix CloudUtils::eigMat2yarpMat(const Eigen::MatrixXf eigMat)
{   // Transforms matrices from Eigen format to YARP format
    int nrows = eigMat.rows();
    int ncols = eigMat.cols();
    Matrix yarpMat = zeros(nrows,ncols);
    for (int row = 0; row<nrows; ++row){
        for (int col = 0; col<ncols; ++col){
            yarpMat(row,col) = eigMat(row,col);
        }
    }
    return yarpMat;
}

/************************************************************************/
Eigen::MatrixXf CloudUtils::yarpMat2eigMat(const Matrix yarpMat){
    // Transforms matrices from YARP format to Eigen format
    int nrows = yarpMat.rows();
    int ncols = yarpMat.cols();
    Eigen::MatrixXf eigMat;
    eigMat.resize(nrows,ncols);
    for (int row = 0; row<nrows; ++row){
        for (int col = 0; col<ncols; ++col){
            eigMat(row,col) = yarpMat(row,col);
        }
    }
    return eigMat;
}

