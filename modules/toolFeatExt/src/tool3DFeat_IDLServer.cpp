// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <tool3DFeat_IDLServer.h>
#include <yarp/os/idl/WireTypes.h>



class tool3DFeat_IDLServer_getFeats : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3DFeat_IDLServer_getToolTip : public yarp::os::Portable {
public:
  Point3D _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3DFeat_IDLServer_getSamples : public yarp::os::Portable {
public:
  int32_t n;
  double deg;
  bool _return;
  void init(const int32_t n, const double deg);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3DFeat_IDLServer_loadModel : public yarp::os::Portable {
public:
  std::string cloudname;
  bool _return;
  void init(const std::string& cloudname);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3DFeat_IDLServer_setPose : public yarp::os::Portable {
public:
  yarp::sig::Matrix toolPose;
  bool _return;
  void init(const yarp::sig::Matrix& toolPose);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3DFeat_IDLServer_setCanonicalPose : public yarp::os::Portable {
public:
  double deg;
  bool _return;
  void init(const double deg);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3DFeat_IDLServer_bins : public yarp::os::Portable {
public:
  int32_t nbins;
  bool _return;
  void init(const int32_t nbins);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3DFeat_IDLServer_depth : public yarp::os::Portable {
public:
  int32_t maxDepth;
  bool _return;
  void init(const int32_t maxDepth);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3DFeat_IDLServer_setVerbose : public yarp::os::Portable {
public:
  std::string verb;
  bool _return;
  void init(const std::string& verb);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool tool3DFeat_IDLServer_getFeats::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getFeats",1,1)) return false;
  return true;
}

bool tool3DFeat_IDLServer_getFeats::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3DFeat_IDLServer_getFeats::init() {
  _return = false;
}

bool tool3DFeat_IDLServer_getToolTip::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("getToolTip",1,1)) return false;
  return true;
}

bool tool3DFeat_IDLServer_getToolTip::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.read(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3DFeat_IDLServer_getToolTip::init() {
}

bool tool3DFeat_IDLServer_getSamples::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("getSamples",1,1)) return false;
  if (!writer.writeI32(n)) return false;
  if (!writer.writeDouble(deg)) return false;
  return true;
}

bool tool3DFeat_IDLServer_getSamples::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3DFeat_IDLServer_getSamples::init(const int32_t n, const double deg) {
  _return = false;
  this->n = n;
  this->deg = deg;
}

bool tool3DFeat_IDLServer_loadModel::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("loadModel",1,1)) return false;
  if (!writer.writeString(cloudname)) return false;
  return true;
}

bool tool3DFeat_IDLServer_loadModel::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3DFeat_IDLServer_loadModel::init(const std::string& cloudname) {
  _return = false;
  this->cloudname = cloudname;
}

bool tool3DFeat_IDLServer_setPose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setPose",1,1)) return false;
  if (!writer.write(toolPose)) return false;
  return true;
}

bool tool3DFeat_IDLServer_setPose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3DFeat_IDLServer_setPose::init(const yarp::sig::Matrix& toolPose) {
  _return = false;
  this->toolPose = toolPose;
}

bool tool3DFeat_IDLServer_setCanonicalPose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setCanonicalPose",1,1)) return false;
  if (!writer.writeDouble(deg)) return false;
  return true;
}

bool tool3DFeat_IDLServer_setCanonicalPose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3DFeat_IDLServer_setCanonicalPose::init(const double deg) {
  _return = false;
  this->deg = deg;
}

bool tool3DFeat_IDLServer_bins::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("bins",1,1)) return false;
  if (!writer.writeI32(nbins)) return false;
  return true;
}

bool tool3DFeat_IDLServer_bins::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3DFeat_IDLServer_bins::init(const int32_t nbins) {
  _return = false;
  this->nbins = nbins;
}

bool tool3DFeat_IDLServer_depth::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("depth",1,1)) return false;
  if (!writer.writeI32(maxDepth)) return false;
  return true;
}

bool tool3DFeat_IDLServer_depth::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3DFeat_IDLServer_depth::init(const int32_t maxDepth) {
  _return = false;
  this->maxDepth = maxDepth;
}

bool tool3DFeat_IDLServer_setVerbose::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setVerbose",1,1)) return false;
  if (!writer.writeString(verb)) return false;
  return true;
}

bool tool3DFeat_IDLServer_setVerbose::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3DFeat_IDLServer_setVerbose::init(const std::string& verb) {
  _return = false;
  this->verb = verb;
}

tool3DFeat_IDLServer::tool3DFeat_IDLServer() {
  yarp().setOwner(*this);
}
bool tool3DFeat_IDLServer::getFeats() {
  bool _return = false;
  tool3DFeat_IDLServer_getFeats helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool tool3DFeat_IDLServer::getFeats()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
Point3D tool3DFeat_IDLServer::getToolTip() {
  Point3D _return;
  tool3DFeat_IDLServer_getToolTip helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","Point3D tool3DFeat_IDLServer::getToolTip()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3DFeat_IDLServer::getSamples(const int32_t n, const double deg) {
  bool _return = false;
  tool3DFeat_IDLServer_getSamples helper;
  helper.init(n,deg);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool tool3DFeat_IDLServer::getSamples(const int32_t n, const double deg)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3DFeat_IDLServer::loadModel(const std::string& cloudname) {
  bool _return = false;
  tool3DFeat_IDLServer_loadModel helper;
  helper.init(cloudname);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool tool3DFeat_IDLServer::loadModel(const std::string& cloudname)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3DFeat_IDLServer::setPose(const yarp::sig::Matrix& toolPose) {
  bool _return = false;
  tool3DFeat_IDLServer_setPose helper;
  helper.init(toolPose);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool tool3DFeat_IDLServer::setPose(const yarp::sig::Matrix& toolPose)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3DFeat_IDLServer::setCanonicalPose(const double deg) {
  bool _return = false;
  tool3DFeat_IDLServer_setCanonicalPose helper;
  helper.init(deg);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool tool3DFeat_IDLServer::setCanonicalPose(const double deg)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3DFeat_IDLServer::bins(const int32_t nbins) {
  bool _return = false;
  tool3DFeat_IDLServer_bins helper;
  helper.init(nbins);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool tool3DFeat_IDLServer::bins(const int32_t nbins)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3DFeat_IDLServer::depth(const int32_t maxDepth) {
  bool _return = false;
  tool3DFeat_IDLServer_depth helper;
  helper.init(maxDepth);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool tool3DFeat_IDLServer::depth(const int32_t maxDepth)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3DFeat_IDLServer::setVerbose(const std::string& verb) {
  bool _return = false;
  tool3DFeat_IDLServer_setVerbose helper;
  helper.init(verb);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool tool3DFeat_IDLServer::setVerbose(const std::string& verb)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool tool3DFeat_IDLServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "getFeats") {
      bool _return;
      _return = getFeats();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getToolTip") {
      Point3D _return;
      _return = getToolTip();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(3)) return false;
        if (!writer.write(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "getSamples") {
      int32_t n;
      double deg;
      if (!reader.readI32(n)) {
        n = 10;
      }
      if (!reader.readDouble(deg)) {
        deg = 0;
      }
      bool _return;
      _return = getSamples(n,deg);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "loadModel") {
      std::string cloudname;
      if (!reader.readString(cloudname)) {
        cloudname = "cloud.ply";
      }
      bool _return;
      _return = loadModel(cloudname);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setPose") {
      yarp::sig::Matrix toolPose;
      if (!reader.read(toolPose)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setPose(toolPose);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setCanonicalPose") {
      double deg;
      if (!reader.readDouble(deg)) {
        deg = 0;
      }
      bool _return;
      _return = setCanonicalPose(deg);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "bins") {
      int32_t nbins;
      if (!reader.readI32(nbins)) {
        nbins = 2;
      }
      bool _return;
      _return = bins(nbins);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "depth") {
      int32_t maxDepth;
      if (!reader.readI32(maxDepth)) {
        maxDepth = 2;
      }
      bool _return;
      _return = depth(maxDepth);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setVerbose") {
      std::string verb;
      if (!reader.readString(verb)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setVerbose(verb);
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

std::vector<std::string> tool3DFeat_IDLServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("getFeats");
    helpString.push_back("getToolTip");
    helpString.push_back("getSamples");
    helpString.push_back("loadModel");
    helpString.push_back("setPose");
    helpString.push_back("setCanonicalPose");
    helpString.push_back("bins");
    helpString.push_back("depth");
    helpString.push_back("setVerbose");
    helpString.push_back("help");
  }
  else {
    if (functionName=="getFeats") {
      helpString.push_back("bool getFeats() ");
      helpString.push_back("@brief getFeats - Performs 3D feature extraction of the tool in the rotated pose. ");
      helpString.push_back("@return true/false on success/failure of extracting features ");
    }
    if (functionName=="getToolTip") {
      helpString.push_back("Point3D getToolTip() ");
      helpString.push_back("@brief getToolTip - Returns the tooltip (as the center of the edge opposite edge of the hand the bounding box) ");
      helpString.push_back("@return true/false on success/failure of computing the tooltip ");
    }
    if (functionName=="getSamples") {
      helpString.push_back("bool getSamples(const int32_t n = 10, const double deg = 0) ");
      helpString.push_back("@brief getSamples - Generates n poses of the tool around a base orientation deg. ");
      helpString.push_back("@param n - (int) desired number of poses (default = 10) . ");
      helpString.push_back("@param deg - (double) base orientation of the tool pose (default = 0.0). ");
      helpString.push_back("@return true/false on success/failure of generating n samples. ");
    }
    if (functionName=="loadModel") {
      helpString.push_back("bool loadModel(const std::string& cloudname = \"cloud.ply\") ");
      helpString.push_back("@brief loadModel - loads a model from a .pcd or .ply file for further processing. ");
      helpString.push_back("@param cloudname - (string) name of the file to load cloud from (and path from base path if needed) (default = \"cloud.ply\") . ");
      helpString.push_back("@return true/false on success/failure loading the desired cloud. ");
    }
    if (functionName=="setPose") {
      helpString.push_back("bool setPose(const yarp::sig::Matrix& toolPose) ");
      helpString.push_back("@brief setPose - Rotates the tool model according to any given rotation matrix ");
      helpString.push_back("@param rotationMatrix - (yarp::sig::Matrix) rotation matrix to apply to the cloud. ");
      helpString.push_back("@return true/false on success/failure of rotating model according to  matrix. ");
    }
    if (functionName=="setCanonicalPose") {
      helpString.push_back("bool setCanonicalPose(const double deg = 0) ");
      helpString.push_back("@brief setCanonicalPose - Rotates the tool model to canonical orientations, i.e. as a rotation along the longest axis which orients the end effector. ");
      helpString.push_back("@param deg - (double) degrees of rotation along the longest axis. ");
      helpString.push_back("@return true/false on success/failure of rotating model according to orientation. ");
    }
    if (functionName=="bins") {
      helpString.push_back("bool bins(const int32_t nbins = 2) ");
      helpString.push_back("@brief bins - sets the number of bins per angular dimension (yaw-pitch-roll) used to compute the normal histogram. Total number of bins per voxel = bins^3. ");
      helpString.push_back("@param nbins - (int) desired number of bins per angular dimension. (default = 2, i.e. 8 bins per voxel). ");
      helpString.push_back("@return true/false on success/failure of setting number of bins. ");
    }
    if (functionName=="depth") {
      helpString.push_back("bool depth(const int32_t maxDepth = 2) ");
      helpString.push_back("@brief depth - sets the number of times that the bounding box will be iteratively subdivided into octants. Total number of voxels = sum(8^(1:depth)). ");
      helpString.push_back("@param maxDepth - (int) desired number of times that the bounding box will be iteratively subdivided into octants (default = 2, i.e. 72 vox). ");
      helpString.push_back("@return true/false on success/failure of setting maxDepth ");
    }
    if (functionName=="setVerbose") {
      helpString.push_back("bool setVerbose(const std::string& verb) ");
      helpString.push_back("@brief setVerbose - sets verbose of the output on or off. ");
      helpString.push_back("@param verb - (string ON/OFF) desired state of verbose. ");
      helpString.push_back("@return true/false on success/failure of setting verbose. ");
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


