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
  bool _return;
  void init(const double radSearch);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class show3D_IDLServer_addBoundingBox : public yarp::os::Portable {
public:
  bool minBB;
  bool _return;
  void init(const bool minBB);
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
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("addNormals",1,1)) return false;
  if (!writer.writeDouble(radSearch)) return false;
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

void show3D_IDLServer_addNormals::init(const double radSearch) {
  _return = false;
  this->radSearch = radSearch;
}

bool show3D_IDLServer_addBoundingBox::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("addBoundingBox",1,1)) return false;
  if (!writer.writeBool(minBB)) return false;
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

void show3D_IDLServer_addBoundingBox::init(const bool minBB) {
  _return = false;
  this->minBB = minBB;
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
bool show3D_IDLServer::addNormals(const double radSearch) {
  bool _return = false;
  show3D_IDLServer_addNormals helper;
  helper.init(radSearch);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool show3D_IDLServer::addNormals(const double radSearch)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool show3D_IDLServer::addBoundingBox(const bool minBB) {
  bool _return = false;
  show3D_IDLServer_addBoundingBox helper;
  helper.init(minBB);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool show3D_IDLServer::addBoundingBox(const bool minBB)");
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
      if (!reader.readDouble(radSearch)) {
        radSearch = 0.03;
      }
      bool _return;
      _return = addNormals(radSearch);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "addBoundingBox") {
      bool minBB;
      if (!reader.readBool(minBB)) {
        minBB = 0;
      }
      bool _return;
      _return = addBoundingBox(minBB);
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
    helpString.push_back("addBoundingBox");
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
      helpString.push_back("bool addNormals(const double radSearch = 0.03) ");
      helpString.push_back("@brief addNormals - adds Normals to the displayed cloud radSearch is used to find neighboring points. ");
      helpString.push_back("@param radSearch - (double) value (in meters) of the extension of the radius search in order to estimate the surface to compute normals from (default = 0.03). ");
      helpString.push_back("@return true/false on showing the poitncloud ");
    }
    if (functionName=="addBoundingBox") {
      helpString.push_back("bool addBoundingBox(const bool minBB = 0) ");
      helpString.push_back("@brief addBoundingBox - adds the bounding box to the displayed cloud. If minBB is true, it will be the minimum BB, otherwise the axis-aligned one. ");
      helpString.push_back("@param minBB - (bool) true to compute the minimum bounding box, false to compute the axis-aligned bounding box (default = false). ");
      helpString.push_back("@return true/false on showing the poitncloud ");
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


