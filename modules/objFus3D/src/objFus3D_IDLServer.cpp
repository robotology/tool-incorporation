// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <objFus3D_IDLServer.h>
#include <yarp/os/idl/WireTypes.h>



class objFus3D_IDLServer_save : public yarp::os::Portable {
public:
  std::string name;
  bool _return;
  void init(const std::string& name);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class objFus3D_IDLServer_track : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class objFus3D_IDLServer_mls : public yarp::os::Portable {
public:
  double rad;
  double usRad;
  double usStep;
  bool _return;
  void init(const double rad, const double usRad, const double usStep);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class objFus3D_IDLServer_ds : public yarp::os::Portable {
public:
  double res;
  bool _return;
  void init(const double res);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class objFus3D_IDLServer_icp : public yarp::os::Portable {
public:
  int32_t maxIt;
  double maxCorr;
  double ranORT;
  double transEps;
  bool _return;
  void init(const int32_t maxIt, const double maxCorr, const double ranORT, const double transEps);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class objFus3D_IDLServer_restart : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class objFus3D_IDLServer_pause : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class objFus3D_IDLServer_verb : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class objFus3D_IDLServer_initAlign : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class objFus3D_IDLServer_quit : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool objFus3D_IDLServer_save::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("save",1,1)) return false;
  if (!writer.writeString(name)) return false;
  return true;
}

bool objFus3D_IDLServer_save::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void objFus3D_IDLServer_save::init(const std::string& name) {
  _return = false;
  this->name = name;
}

bool objFus3D_IDLServer_track::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("track",1,1)) return false;
  return true;
}

bool objFus3D_IDLServer_track::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void objFus3D_IDLServer_track::init() {
  _return = false;
}

bool objFus3D_IDLServer_mls::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(4)) return false;
  if (!writer.writeTag("mls",1,1)) return false;
  if (!writer.writeDouble(rad)) return false;
  if (!writer.writeDouble(usRad)) return false;
  if (!writer.writeDouble(usStep)) return false;
  return true;
}

bool objFus3D_IDLServer_mls::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void objFus3D_IDLServer_mls::init(const double rad, const double usRad, const double usStep) {
  _return = false;
  this->rad = rad;
  this->usRad = usRad;
  this->usStep = usStep;
}

bool objFus3D_IDLServer_ds::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("ds",1,1)) return false;
  if (!writer.writeDouble(res)) return false;
  return true;
}

bool objFus3D_IDLServer_ds::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void objFus3D_IDLServer_ds::init(const double res) {
  _return = false;
  this->res = res;
}

bool objFus3D_IDLServer_icp::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(5)) return false;
  if (!writer.writeTag("icp",1,1)) return false;
  if (!writer.writeI32(maxIt)) return false;
  if (!writer.writeDouble(maxCorr)) return false;
  if (!writer.writeDouble(ranORT)) return false;
  if (!writer.writeDouble(transEps)) return false;
  return true;
}

bool objFus3D_IDLServer_icp::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void objFus3D_IDLServer_icp::init(const int32_t maxIt, const double maxCorr, const double ranORT, const double transEps) {
  _return = false;
  this->maxIt = maxIt;
  this->maxCorr = maxCorr;
  this->ranORT = ranORT;
  this->transEps = transEps;
}

bool objFus3D_IDLServer_restart::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("restart",1,1)) return false;
  return true;
}

bool objFus3D_IDLServer_restart::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void objFus3D_IDLServer_restart::init() {
  _return = false;
}

bool objFus3D_IDLServer_pause::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("pause",1,1)) return false;
  return true;
}

bool objFus3D_IDLServer_pause::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void objFus3D_IDLServer_pause::init() {
  _return = false;
}

bool objFus3D_IDLServer_verb::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("verb",1,1)) return false;
  return true;
}

bool objFus3D_IDLServer_verb::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void objFus3D_IDLServer_verb::init() {
  _return = false;
}

bool objFus3D_IDLServer_initAlign::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("initAlign",1,1)) return false;
  return true;
}

bool objFus3D_IDLServer_initAlign::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void objFus3D_IDLServer_initAlign::init() {
  _return = false;
}

bool objFus3D_IDLServer_quit::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("quit",1,1)) return false;
  return true;
}

bool objFus3D_IDLServer_quit::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void objFus3D_IDLServer_quit::init() {
  _return = false;
}

objFus3D_IDLServer::objFus3D_IDLServer() {
  yarp().setOwner(*this);
}
bool objFus3D_IDLServer::save(const std::string& name) {
  bool _return = false;
  objFus3D_IDLServer_save helper;
  helper.init(name);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool objFus3D_IDLServer::save(const std::string& name)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool objFus3D_IDLServer::track() {
  bool _return = false;
  objFus3D_IDLServer_track helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool objFus3D_IDLServer::track()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool objFus3D_IDLServer::mls(const double rad, const double usRad, const double usStep) {
  bool _return = false;
  objFus3D_IDLServer_mls helper;
  helper.init(rad,usRad,usStep);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool objFus3D_IDLServer::mls(const double rad, const double usRad, const double usStep)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool objFus3D_IDLServer::ds(const double res) {
  bool _return = false;
  objFus3D_IDLServer_ds helper;
  helper.init(res);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool objFus3D_IDLServer::ds(const double res)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool objFus3D_IDLServer::icp(const int32_t maxIt, const double maxCorr, const double ranORT, const double transEps) {
  bool _return = false;
  objFus3D_IDLServer_icp helper;
  helper.init(maxIt,maxCorr,ranORT,transEps);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool objFus3D_IDLServer::icp(const int32_t maxIt, const double maxCorr, const double ranORT, const double transEps)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool objFus3D_IDLServer::restart() {
  bool _return = false;
  objFus3D_IDLServer_restart helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool objFus3D_IDLServer::restart()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool objFus3D_IDLServer::pause() {
  bool _return = false;
  objFus3D_IDLServer_pause helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool objFus3D_IDLServer::pause()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool objFus3D_IDLServer::verb() {
  bool _return = false;
  objFus3D_IDLServer_verb helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool objFus3D_IDLServer::verb()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool objFus3D_IDLServer::initAlign() {
  bool _return = false;
  objFus3D_IDLServer_initAlign helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool objFus3D_IDLServer::initAlign()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool objFus3D_IDLServer::quit() {
  bool _return = false;
  objFus3D_IDLServer_quit helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool objFus3D_IDLServer::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool objFus3D_IDLServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "save") {
      std::string name;
      if (!reader.readString(name)) {
        name = "model";
      }
      bool _return;
      _return = save(name);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "track") {
      bool _return;
      _return = track();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "mls") {
      double rad;
      double usRad;
      double usStep;
      if (!reader.readDouble(rad)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(usRad)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(usStep)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = mls(rad,usRad,usStep);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "ds") {
      double res;
      if (!reader.readDouble(res)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = ds(res);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "icp") {
      int32_t maxIt;
      double maxCorr;
      double ranORT;
      double transEps;
      if (!reader.readI32(maxIt)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(maxCorr)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(ranORT)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(transEps)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = icp(maxIt,maxCorr,ranORT,transEps);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "restart") {
      bool _return;
      _return = restart();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "pause") {
      bool _return;
      _return = pause();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "verb") {
      bool _return;
      _return = verb();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "initAlign") {
      bool _return;
      _return = initAlign();
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

std::vector<std::string> objFus3D_IDLServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("save");
    helpString.push_back("track");
    helpString.push_back("mls");
    helpString.push_back("ds");
    helpString.push_back("icp");
    helpString.push_back("restart");
    helpString.push_back("pause");
    helpString.push_back("verb");
    helpString.push_back("initAlign");
    helpString.push_back("quit");
    helpString.push_back("help");
  }
  else {
    if (functionName=="save") {
      helpString.push_back("bool save(const std::string& name = \"model\") ");
      helpString.push_back("save (string name) - saved the current merged cloud into file of given name ");
      helpString.push_back("@return true/false on success/failure of saving cloud ");
    }
    if (functionName=="track") {
      helpString.push_back("bool track() ");
      helpString.push_back("@brief track - ask the user to select the bounding box and starts tracker on template ");
      helpString.push_back("@return true/false on success/failure of starting tracker ");
    }
    if (functionName=="mls") {
      helpString.push_back("bool mls(const double rad, const double usRad, const double usStep) ");
      helpString.push_back("@brief mls - sets parameters for moving least squares filtering ");
      helpString.push_back("@param ");
      helpString.push_back("@param ");
      helpString.push_back("@param ");
      helpString.push_back("@return true/false on success/failure setting parameters ");
    }
    if (functionName=="ds") {
      helpString.push_back("bool ds(const double res) ");
      helpString.push_back("@brief ds - sets parameters for downsampling ");
      helpString.push_back("@param ");
      helpString.push_back("@return true/false on success/failure setting parameters ");
    }
    if (functionName=="icp") {
      helpString.push_back("bool icp(const int32_t maxIt, const double maxCorr, const double ranORT, const double transEps) ");
      helpString.push_back("@brief icp - sets parameters for iterative closes algorithm ");
      helpString.push_back("@param ");
      helpString.push_back("@param ");
      helpString.push_back("@param ");
      helpString.push_back("@param ");
      helpString.push_back("@return true/false on success/failure of setting parameters ");
    }
    if (functionName=="restart") {
      helpString.push_back("bool restart() ");
      helpString.push_back("@brief restart - Clears all clouds and visualizer, restarts tracker and restarts a new reconstruction. ");
      helpString.push_back("@return true/false on success/failure of cleaning and restarting ");
    }
    if (functionName=="pause") {
      helpString.push_back("bool pause() ");
      helpString.push_back("@brief pause - Pauses recosntruction/merging until it is called again. ");
      helpString.push_back("@return true/false on success/failure of pausing. ");
    }
    if (functionName=="verb") {
      helpString.push_back("bool verb() ");
      helpString.push_back("@brief verb - switches verbose between ON and OFF ");
      helpString.push_back("@return true/false on success/failure of setting verbose ");
    }
    if (functionName=="initAlign") {
      helpString.push_back("bool initAlign() ");
      helpString.push_back("@brief initAlign - Set initial feature based alignment ON/OFF ");
      helpString.push_back("@return true/false on success/failure of (de)activating alingment. ");
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


