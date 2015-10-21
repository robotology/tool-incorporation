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
    }
    if (functionName=="restart") {
      helpString.push_back("bool restart() ");
      helpString.push_back("@brief restart - Clears all clouds and visualizer and restarts a new reconstruction. ");
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


