// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <show3D_IDLServer.h>
#include <yarp/os/idl/WireTypes.h>



class show3D_IDLServer_clearVis : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class show3D_IDLServer_accumClouds : public yarp::os::Portable {
public:
  bool accum;
  bool _return;
  void init(const bool accum);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class show3D_IDLServer_showFileCloud : public yarp::os::Portable {
public:
  std::string cloudname;
  bool _return;
  void init(const std::string& cloudname);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class show3D_IDLServer_addNormals : public yarp::os::Portable {
public:
  double radSearch;
  bool normCol;
  bool _return;
  void init(const double radSearch, const bool normCol);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class show3D_IDLServer_addFeats : public yarp::os::Portable {
public:
  double res;
  bool plotHist;
  bool _return;
  void init(const double res, const bool plotHist);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class show3D_IDLServer_addBoundingBox : public yarp::os::Portable {
public:
  int32_t typeBB;
  bool _return;
  void init(const int32_t typeBB);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class show3D_IDLServer_addSphere : public yarp::os::Portable {
public:
  std::vector<double>  coords;
  std::vector<int32_t>  color;
  bool _return;
  void init(const std::vector<double> & coords, const std::vector<int32_t> & color);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class show3D_IDLServer_filter : public yarp::os::Portable {
public:
  bool ror;
  bool sor;
  bool mks;
  bool ds;
  bool _return;
  void init(const bool ror, const bool sor, const bool mks, const bool ds);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class show3D_IDLServer_quit : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool show3D_IDLServer_clearVis::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("clearVis",1,1)) return false;
  return true;
}

bool show3D_IDLServer_clearVis::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void show3D_IDLServer_clearVis::init() {
  _return = false;
}

bool show3D_IDLServer_accumClouds::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("accumClouds",1,1)) return false;
  if (!writer.writeBool(accum)) return false;
  return true;
}

bool show3D_IDLServer_accumClouds::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void show3D_IDLServer_accumClouds::init(const bool accum) {
  _return = false;
  this->accum = accum;
}

bool show3D_IDLServer_showFileCloud::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("showFileCloud",1,1)) return false;
  if (!writer.writeString(cloudname)) return false;
  return true;
}

bool show3D_IDLServer_showFileCloud::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void show3D_IDLServer_showFileCloud::init(const std::string& cloudname) {
  _return = false;
  this->cloudname = cloudname;
}

bool show3D_IDLServer_addNormals::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("addNormals",1,1)) return false;
  if (!writer.writeDouble(radSearch)) return false;
  if (!writer.writeBool(normCol)) return false;
  return true;
}

bool show3D_IDLServer_addNormals::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void show3D_IDLServer_addNormals::init(const double radSearch, const bool normCol) {
  _return = false;
  this->radSearch = radSearch;
  this->normCol = normCol;
}

bool show3D_IDLServer_addFeats::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("addFeats",1,1)) return false;
  if (!writer.writeDouble(res)) return false;
  if (!writer.writeBool(plotHist)) return false;
  return true;
}

bool show3D_IDLServer_addFeats::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void show3D_IDLServer_addFeats::init(const double res, const bool plotHist) {
  _return = false;
  this->res = res;
  this->plotHist = plotHist;
}

bool show3D_IDLServer_addBoundingBox::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("addBoundingBox",1,1)) return false;
  if (!writer.writeI32(typeBB)) return false;
  return true;
}

bool show3D_IDLServer_addBoundingBox::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void show3D_IDLServer_addBoundingBox::init(const int32_t typeBB) {
  _return = false;
  this->typeBB = typeBB;
}

bool show3D_IDLServer_addSphere::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("addSphere",1,1)) return false;
  {
    if (!writer.writeListBegin(BOTTLE_TAG_DOUBLE, static_cast<uint32_t>(coords.size()))) return false;
    std::vector<double> ::iterator _iter0;
    for (_iter0 = coords.begin(); _iter0 != coords.end(); ++_iter0)
    {
      if (!writer.writeDouble((*_iter0))) return false;
    }
    if (!writer.writeListEnd()) return false;
  }
  {
    if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(color.size()))) return false;
    std::vector<int32_t> ::iterator _iter1;
    for (_iter1 = color.begin(); _iter1 != color.end(); ++_iter1)
    {
      if (!writer.writeI32((*_iter1))) return false;
    }
    if (!writer.writeListEnd()) return false;
  }
  return true;
}

bool show3D_IDLServer_addSphere::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void show3D_IDLServer_addSphere::init(const std::vector<double> & coords, const std::vector<int32_t> & color) {
  _return = false;
  this->coords = coords;
  this->color = color;
}

bool show3D_IDLServer_filter::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(5)) return false;
  if (!writer.writeTag("filter",1,1)) return false;
  if (!writer.writeBool(ror)) return false;
  if (!writer.writeBool(sor)) return false;
  if (!writer.writeBool(mks)) return false;
  if (!writer.writeBool(ds)) return false;
  return true;
}

bool show3D_IDLServer_filter::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void show3D_IDLServer_filter::init(const bool ror, const bool sor, const bool mks, const bool ds) {
  _return = false;
  this->ror = ror;
  this->sor = sor;
  this->mks = mks;
  this->ds = ds;
}

bool show3D_IDLServer_quit::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("quit",1,1)) return false;
  return true;
}

bool show3D_IDLServer_quit::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void show3D_IDLServer_quit::init() {
  _return = false;
}

show3D_IDLServer::show3D_IDLServer() {
  yarp().setOwner(*this);
}
bool show3D_IDLServer::clearVis() {
  bool _return = false;
  show3D_IDLServer_clearVis helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool show3D_IDLServer::clearVis()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool show3D_IDLServer::accumClouds(const bool accum) {
  bool _return = false;
  show3D_IDLServer_accumClouds helper;
  helper.init(accum);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool show3D_IDLServer::accumClouds(const bool accum)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool show3D_IDLServer::showFileCloud(const std::string& cloudname) {
  bool _return = false;
  show3D_IDLServer_showFileCloud helper;
  helper.init(cloudname);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool show3D_IDLServer::showFileCloud(const std::string& cloudname)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool show3D_IDLServer::addNormals(const double radSearch, const bool normCol) {
  bool _return = false;
  show3D_IDLServer_addNormals helper;
  helper.init(radSearch,normCol);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool show3D_IDLServer::addNormals(const double radSearch, const bool normCol)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool show3D_IDLServer::addFeats(const double res, const bool plotHist) {
  bool _return = false;
  show3D_IDLServer_addFeats helper;
  helper.init(res,plotHist);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool show3D_IDLServer::addFeats(const double res, const bool plotHist)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool show3D_IDLServer::addBoundingBox(const int32_t typeBB) {
  bool _return = false;
  show3D_IDLServer_addBoundingBox helper;
  helper.init(typeBB);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool show3D_IDLServer::addBoundingBox(const int32_t typeBB)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool show3D_IDLServer::addSphere(const std::vector<double> & coords, const std::vector<int32_t> & color) {
  bool _return = false;
  show3D_IDLServer_addSphere helper;
  helper.init(coords,color);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool show3D_IDLServer::addSphere(const std::vector<double> & coords, const std::vector<int32_t> & color)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool show3D_IDLServer::filter(const bool ror, const bool sor, const bool mks, const bool ds) {
  bool _return = false;
  show3D_IDLServer_filter helper;
  helper.init(ror,sor,mks,ds);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool show3D_IDLServer::filter(const bool ror, const bool sor, const bool mks, const bool ds)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool show3D_IDLServer::quit() {
  bool _return = false;
  show3D_IDLServer_quit helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool show3D_IDLServer::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool show3D_IDLServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "clearVis") {
      bool _return;
      _return = clearVis();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "accumClouds") {
      bool accum;
      if (!reader.readBool(accum)) {
        accum = 0;
      }
      bool _return;
      _return = accumClouds(accum);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "showFileCloud") {
      std::string cloudname;
      if (!reader.readString(cloudname)) {
        cloudname = "cloud.ply";
      }
      bool _return;
      _return = showFileCloud(cloudname);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "addNormals") {
      double radSearch;
      bool normCol;
      if (!reader.readDouble(radSearch)) {
        radSearch = 0.01;
      }
      if (!reader.readBool(normCol)) {
        normCol = 1;
      }
      bool _return;
      _return = addNormals(radSearch,normCol);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "addFeats") {
      double res;
      bool plotHist;
      if (!reader.readDouble(res)) {
        res = 0.01;
      }
      if (!reader.readBool(plotHist)) {
        plotHist = 1;
      }
      bool _return;
      _return = addFeats(res,plotHist);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "addBoundingBox") {
      int32_t typeBB;
      if (!reader.readI32(typeBB)) {
        typeBB = 0;
      }
      bool _return;
      _return = addBoundingBox(typeBB);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "addSphere") {
      std::vector<double>  coords;
      std::vector<int32_t>  color;
      {
        coords.clear();
        uint32_t _size2;
        yarp::os::idl::WireState _etype5;
        reader.readListBegin(_etype5, _size2);
        coords.resize(_size2);
        uint32_t _i6;
        for (_i6 = 0; _i6 < _size2; ++_i6)
        {
          if (!reader.readDouble(coords[_i6])) {
            reader.fail();
            return false;
          }
        }
        reader.readListEnd();
      }
      {
        color.clear();
        uint32_t _size7;
        yarp::os::idl::WireState _etype10;
        reader.readListBegin(_etype10, _size7);
        color.resize(_size7);
        uint32_t _i11;
        for (_i11 = 0; _i11 < _size7; ++_i11)
        {
          if (!reader.readI32(color[_i11])) {
            reader.fail();
            return false;
          }
        }
        reader.readListEnd();
      }
      bool _return;
      _return = addSphere(coords,color);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "filter") {
      bool ror;
      bool sor;
      bool mks;
      bool ds;
      if (!reader.readBool(ror)) {
        ror = 0;
      }
      if (!reader.readBool(sor)) {
        sor = 0;
      }
      if (!reader.readBool(mks)) {
        mks = 0;
      }
      if (!reader.readBool(ds)) {
        ds = 0;
      }
      bool _return;
      _return = filter(ror,sor,mks,ds);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "quit") {
      bool _return;
      _return = quit();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> show3D_IDLServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("clearVis");
    helpString.push_back("accumClouds");
    helpString.push_back("showFileCloud");
    helpString.push_back("addNormals");
    helpString.push_back("addFeats");
    helpString.push_back("addBoundingBox");
    helpString.push_back("addSphere");
    helpString.push_back("filter");
    helpString.push_back("quit");
    helpString.push_back("help");
  }
  else {
    if (functionName=="clearVis") {
      helpString.push_back("bool clearVis() ");
      helpString.push_back("@brief clearVis - Clears all clouds and effects from the visualizer. ");
      helpString.push_back("@return true/false on success/failure of extracting features ");
    }
    if (functionName=="accumClouds") {
      helpString.push_back("bool accumClouds(const bool accum = 0) ");
      helpString.push_back("@brief accumClouds - Selects between plotting clouds together or not ");
      helpString.push_back("@param accum - (bool) true to plot clouds together and false to clear the visualizer for every new cloud (default =false) ");
      helpString.push_back("@return true/false on success/failure of setting verbose ");
    }
    if (functionName=="showFileCloud") {
      helpString.push_back("bool showFileCloud(const std::string& cloudname = \"cloud.ply\") ");
      helpString.push_back("@brief showFileCloud - Opens the visualizer and displays a PointCloud from a file ");
      helpString.push_back("@param cloudname - (string) name of the .ply or .pcd file containg the desired cloud to be shown (default \"cloud.ply\") ");
      helpString.push_back("@return true/false on showing the poitncloud ");
    }
    if (functionName=="addNormals") {
      helpString.push_back("bool addNormals(const double radSearch = 0.01, const bool normCol = 1) ");
      helpString.push_back("@brief addNormals - adds Normals to the displayed cloud radSearch is used to find neighboring points. ");
      helpString.push_back("@param radSearch - (double) value (in meters) of the extension of the radius search in order to estimate the surface to compute normals from (default = 0.03). ");
      helpString.push_back("@param normalColors - (bool) Expresses whether the normals should be expressed as a vector (false), or as color gradients on the cloud (true). ");
      helpString.push_back("@return true/false on showing the poitncloud ");
    }
    if (functionName=="addFeats") {
      helpString.push_back("bool addFeats(const double res = 0.01, const bool plotHist = 1) ");
      helpString.push_back("@brief addFeats - adds a visualization of the OMS-EGI features (voxel based spherical normal histograms). ");
      helpString.push_back("@param res - (double) value (in meters) side of the voxel grid on which the OMS-EGI featureas are copmuted (default = 0.01). ");
      helpString.push_back("@param plotHist - (bool) Determines whether the average value of the normal histogram should be shown inside each voxel as a sphere (default = true) or not (false). ");
      helpString.push_back("@return true/false on showing the poitncloud ");
    }
    if (functionName=="addBoundingBox") {
      helpString.push_back("bool addBoundingBox(const int32_t typeBB = 0) ");
      helpString.push_back("@brief addBoundingBox - adds the bounding box to the displayed cloud. If minBB is true, it will be the minimum BB, otherwise the axis-aligned one. ");
      helpString.push_back("@param tpyeBB - (int) 0 to compute the minimum bounding box, 1 to compute the axis-aligned bounding box, 2 to compute Cubic AABB (default = 2), ");
      helpString.push_back("@return true/false on showing the poitncloud ");
    }
    if (functionName=="addSphere") {
      helpString.push_back("bool addSphere(const std::vector<double> & coords, const std::vector<int32_t> & color) ");
      helpString.push_back("@brief addSphere - Plots a sphere with given coords and color ");
      helpString.push_back("@param coords - sphere coords ");
      helpString.push_back("@param color - color rgb ");
      helpString.push_back("@return true/false on displaying the sphere ");
    }
    if (functionName=="filter") {
      helpString.push_back("bool filter(const bool ror = 0, const bool sor = 0, const bool mks = 0, const bool ds = 0) ");
      helpString.push_back("@brief filter - Function to apply and show different filtering processes to the displayed cloud. ");
      helpString.push_back("@param ror - bool: Activates RadiusOutlierRemoval (rad = 0.05, minNeigh = 5). ");
      helpString.push_back("@param sor - bool: Activates StatisticalOutlierRemoval (meanK = 20). ");
      helpString.push_back("@param mls - bool: Activates MovingLeastSquares (rad = 0.02, order 2, usRad = 0.005, usStep = 0.003) ");
      helpString.push_back("@param ds - bool: Activates Voxel Grid Downsampling (rad = 0.002). ");
      helpString.push_back("@return true/false on showing the poitnclouddar = ");
    }
    if (functionName=="quit") {
      helpString.push_back("bool quit() ");
      helpString.push_back("@brief quit - quits the module ");
      helpString.push_back("@return true/false on success/failure of extracting features ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}


