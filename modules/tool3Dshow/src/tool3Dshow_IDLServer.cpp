// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <tool3Dshow_IDLServer.h>
#include <yarp/os/idl/WireTypes.h>



class tool3Dshow_IDLServer_clearVis : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3Dshow_IDLServer_accumClouds : public yarp::os::Portable {
public:
  bool accum;
  bool _return;
  void init(const bool accum);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3Dshow_IDLServer_showFileCloud : public yarp::os::Portable {
public:
  std::string cloudname;
  bool _return;
  void init(const std::string& cloudname);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3Dshow_IDLServer_addNormals : public yarp::os::Portable {
public:
  double radSearch;
  bool _return;
  void init(const double radSearch);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3Dshow_IDLServer_addBoundingBox : public yarp::os::Portable {
public:
  bool minBB;
  bool _return;
  void init(const bool minBB);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3Dshow_IDLServer_help_commands : public yarp::os::Portable {
public:
  std::string _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3Dshow_IDLServer_quit : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool tool3Dshow_IDLServer_clearVis::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("clearVis",1,1)) return false;
  return true;
}

bool tool3Dshow_IDLServer_clearVis::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3Dshow_IDLServer_clearVis::init() {
  _return = false;
}

bool tool3Dshow_IDLServer_accumClouds::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("accumClouds",1,1)) return false;
  if (!writer.writeBool(accum)) return false;
  return true;
}

bool tool3Dshow_IDLServer_accumClouds::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3Dshow_IDLServer_accumClouds::init(const bool accum) {
  _return = false;
  this->accum = accum;
}

bool tool3Dshow_IDLServer_showFileCloud::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("showFileCloud",1,1)) return false;
  if (!writer.writeString(cloudname)) return false;
  return true;
}

bool tool3Dshow_IDLServer_showFileCloud::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3Dshow_IDLServer_showFileCloud::init(const std::string& cloudname) {
  _return = false;
  this->cloudname = cloudname;
}

bool tool3Dshow_IDLServer_addNormals::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("addNormals",1,1)) return false;
  if (!writer.writeDouble(radSearch)) return false;
  return true;
}

bool tool3Dshow_IDLServer_addNormals::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3Dshow_IDLServer_addNormals::init(const double radSearch) {
  _return = false;
  this->radSearch = radSearch;
}

bool tool3Dshow_IDLServer_addBoundingBox::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("addBoundingBox",1,1)) return false;
  if (!writer.writeBool(minBB)) return false;
  return true;
}

bool tool3Dshow_IDLServer_addBoundingBox::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3Dshow_IDLServer_addBoundingBox::init(const bool minBB) {
  _return = false;
  this->minBB = minBB;
}

bool tool3Dshow_IDLServer_help_commands::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("help_commands",1,2)) return false;
  return true;
}

bool tool3Dshow_IDLServer_help_commands::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3Dshow_IDLServer_help_commands::init() {
  _return = "";
}

bool tool3Dshow_IDLServer_quit::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("quit",1,1)) return false;
  return true;
}

bool tool3Dshow_IDLServer_quit::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3Dshow_IDLServer_quit::init() {
  _return = false;
}

tool3Dshow_IDLServer::tool3Dshow_IDLServer() {
  yarp().setOwner(*this);
}
bool tool3Dshow_IDLServer::clearVis() {
  bool _return = false;
  tool3Dshow_IDLServer_clearVis helper;
  helper.init();
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tool3Dshow_IDLServer::clearVis()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3Dshow_IDLServer::accumClouds(const bool accum) {
  bool _return = false;
  tool3Dshow_IDLServer_accumClouds helper;
  helper.init(accum);
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tool3Dshow_IDLServer::accumClouds(const bool accum)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3Dshow_IDLServer::showFileCloud(const std::string& cloudname) {
  bool _return = false;
  tool3Dshow_IDLServer_showFileCloud helper;
  helper.init(cloudname);
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tool3Dshow_IDLServer::showFileCloud(const std::string& cloudname)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3Dshow_IDLServer::addNormals(const double radSearch) {
  bool _return = false;
  tool3Dshow_IDLServer_addNormals helper;
  helper.init(radSearch);
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tool3Dshow_IDLServer::addNormals(const double radSearch)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3Dshow_IDLServer::addBoundingBox(const bool minBB) {
  bool _return = false;
  tool3Dshow_IDLServer_addBoundingBox helper;
  helper.init(minBB);
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tool3Dshow_IDLServer::addBoundingBox(const bool minBB)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string tool3Dshow_IDLServer::help_commands() {
  std::string _return = "";
  tool3Dshow_IDLServer_help_commands helper;
  helper.init();
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","std::string tool3Dshow_IDLServer::help_commands()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3Dshow_IDLServer::quit() {
  bool _return = false;
  tool3Dshow_IDLServer_quit helper;
  helper.init();
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tool3Dshow_IDLServer::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool tool3Dshow_IDLServer::read(yarp::os::ConnectionReader& connection) {
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
        cloudname = "cloud_merged.ply";
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
    if (tag == "help_commands") {
      std::string _return;
      _return = help_commands();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
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

std::vector<std::string> tool3Dshow_IDLServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("clearVis");
    helpString.push_back("accumClouds");
    helpString.push_back("showFileCloud");
    helpString.push_back("addNormals");
    helpString.push_back("addBoundingBox");
    helpString.push_back("help_commands");
    helpString.push_back("quit");
    helpString.push_back("help");
  }
  else {
    if (functionName=="clearVis") {
      helpString.push_back("bool clearVis() ");
    }
    if (functionName=="accumClouds") {
      helpString.push_back("bool accumClouds(const bool accum = 0) ");
    }
    if (functionName=="showFileCloud") {
      helpString.push_back("bool showFileCloud(const std::string& cloudname = \"cloud_merged.ply\") ");
    }
    if (functionName=="addNormals") {
      helpString.push_back("bool addNormals(const double radSearch = 0.03) ");
    }
    if (functionName=="addBoundingBox") {
      helpString.push_back("bool addBoundingBox(const bool minBB = 0) ");
    }
    if (functionName=="help_commands") {
      helpString.push_back("std::string help_commands() ");
    }
    if (functionName=="quit") {
      helpString.push_back("bool quit() ");
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


