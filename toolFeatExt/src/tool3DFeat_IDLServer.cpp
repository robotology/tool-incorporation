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
  int32_t deg;
  bool _return;
  void init(const int32_t deg);
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

class tool3DFeat_IDLServer_setName : public yarp::os::Portable {
public:
  std::string cloudname;
  bool _return;
  void init(const std::string& cloudname);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class tool3DFeat_IDLServer_help_commands : public yarp::os::Portable {
public:
  bool _return;
  void init();
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
  if (!writer.writeI32(deg)) return false;
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

void tool3DFeat_IDLServer_setCanonicalPose::init(const int32_t deg) {
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

bool tool3DFeat_IDLServer_setName::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("setName",1,1)) return false;
  if (!writer.writeString(cloudname)) return false;
  return true;
}

bool tool3DFeat_IDLServer_setName::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3DFeat_IDLServer_setName::init(const std::string& cloudname) {
  _return = false;
  this->cloudname = cloudname;
}

bool tool3DFeat_IDLServer_help_commands::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("help_commands",1,2)) return false;
  return true;
}

bool tool3DFeat_IDLServer_help_commands::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void tool3DFeat_IDLServer_help_commands::init() {
  _return = false;
}

tool3DFeat_IDLServer::tool3DFeat_IDLServer() {
  yarp().setOwner(*this);
}
bool tool3DFeat_IDLServer::getFeats() {
  bool _return = false;
  tool3DFeat_IDLServer_getFeats helper;
  helper.init();
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tool3DFeat_IDLServer::getFeats()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3DFeat_IDLServer::setPose(const yarp::sig::Matrix& toolPose) {
  bool _return = false;
  tool3DFeat_IDLServer_setPose helper;
  helper.init(toolPose);
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tool3DFeat_IDLServer::setPose(const yarp::sig::Matrix& toolPose)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3DFeat_IDLServer::setCanonicalPose(const int32_t deg) {
  bool _return = false;
  tool3DFeat_IDLServer_setCanonicalPose helper;
  helper.init(deg);
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tool3DFeat_IDLServer::setCanonicalPose(const int32_t deg)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3DFeat_IDLServer::bins(const int32_t nbins) {
  bool _return = false;
  tool3DFeat_IDLServer_bins helper;
  helper.init(nbins);
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tool3DFeat_IDLServer::bins(const int32_t nbins)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3DFeat_IDLServer::depth(const int32_t maxDepth) {
  bool _return = false;
  tool3DFeat_IDLServer_depth helper;
  helper.init(maxDepth);
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tool3DFeat_IDLServer::depth(const int32_t maxDepth)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3DFeat_IDLServer::setVerbose(const std::string& verb) {
  bool _return = false;
  tool3DFeat_IDLServer_setVerbose helper;
  helper.init(verb);
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tool3DFeat_IDLServer::setVerbose(const std::string& verb)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3DFeat_IDLServer::setName(const std::string& cloudname) {
  bool _return = false;
  tool3DFeat_IDLServer_setName helper;
  helper.init(cloudname);
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tool3DFeat_IDLServer::setName(const std::string& cloudname)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool tool3DFeat_IDLServer::help_commands() {
  bool _return = false;
  tool3DFeat_IDLServer_help_commands helper;
  helper.init();
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool tool3DFeat_IDLServer::help_commands()");
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
      int32_t deg;
      if (!reader.readI32(deg)) {
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
    if (tag == "setName") {
      std::string cloudname;
      if (!reader.readString(cloudname)) {
        cloudname = "cloud_merged.ply";
      }
      bool _return;
      _return = setName(cloudname);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help_commands") {
      bool _return;
      _return = help_commands();
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
    helpString.push_back("setPose");
    helpString.push_back("setCanonicalPose");
    helpString.push_back("bins");
    helpString.push_back("depth");
    helpString.push_back("setVerbose");
    helpString.push_back("setName");
    helpString.push_back("help_commands");
    helpString.push_back("help");
  }
  else {
    if (functionName=="getFeats") {
      helpString.push_back("bool getFeats() ");
    }
    if (functionName=="setPose") {
      helpString.push_back("bool setPose(const yarp::sig::Matrix& toolPose) ");
    }
    if (functionName=="setCanonicalPose") {
      helpString.push_back("bool setCanonicalPose(const int32_t deg = 0) ");
    }
    if (functionName=="bins") {
      helpString.push_back("bool bins(const int32_t nbins = 2) ");
    }
    if (functionName=="depth") {
      helpString.push_back("bool depth(const int32_t maxDepth = 2) ");
    }
    if (functionName=="setVerbose") {
      helpString.push_back("bool setVerbose(const std::string& verb) ");
    }
    if (functionName=="setName") {
      helpString.push_back("bool setName(const std::string& cloudname = \"cloud_merged.ply\") ");
    }
    if (functionName=="help_commands") {
      helpString.push_back("bool help_commands() ");
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


