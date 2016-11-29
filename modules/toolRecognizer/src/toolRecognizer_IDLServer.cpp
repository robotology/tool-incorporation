// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <toolRecognizer_IDLServer.h>
#include <yarp/os/idl/WireTypes.h>



class toolRecognizer_IDLServer_start : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class toolRecognizer_IDLServer_quit : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class toolRecognizer_IDLServer_train : public yarp::os::Portable {
public:
  std::string label;
  int32_t tlx;
  int32_t tly;
  int32_t brx;
  int32_t bry;
  bool _return;
  void init(const std::string& label, const int32_t tlx, const int32_t tly, const int32_t brx, const int32_t bry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class toolRecognizer_IDLServer_recognize : public yarp::os::Portable {
public:
  int32_t tlx;
  int32_t tly;
  int32_t brx;
  int32_t bry;
  std::string _return;
  void init(const int32_t tlx, const int32_t tly, const int32_t brx, const int32_t bry);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class toolRecognizer_IDLServer_burst : public yarp::os::Portable {
public:
  bool burstF;
  bool _return;
  void init(const bool burstF);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool toolRecognizer_IDLServer_start::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("start",1,1)) return false;
  return true;
}

bool toolRecognizer_IDLServer_start::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void toolRecognizer_IDLServer_start::init() {
  _return = false;
}

bool toolRecognizer_IDLServer_quit::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("quit",1,1)) return false;
  return true;
}

bool toolRecognizer_IDLServer_quit::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void toolRecognizer_IDLServer_quit::init() {
  _return = false;
}

bool toolRecognizer_IDLServer_train::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(6)) return false;
  if (!writer.writeTag("train",1,1)) return false;
  if (!writer.writeString(label)) return false;
  if (!writer.writeI32(tlx)) return false;
  if (!writer.writeI32(tly)) return false;
  if (!writer.writeI32(brx)) return false;
  if (!writer.writeI32(bry)) return false;
  return true;
}

bool toolRecognizer_IDLServer_train::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void toolRecognizer_IDLServer_train::init(const std::string& label, const int32_t tlx, const int32_t tly, const int32_t brx, const int32_t bry) {
  _return = false;
  this->label = label;
  this->tlx = tlx;
  this->tly = tly;
  this->brx = brx;
  this->bry = bry;
}

bool toolRecognizer_IDLServer_recognize::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(5)) return false;
  if (!writer.writeTag("recognize",1,1)) return false;
  if (!writer.writeI32(tlx)) return false;
  if (!writer.writeI32(tly)) return false;
  if (!writer.writeI32(brx)) return false;
  if (!writer.writeI32(bry)) return false;
  return true;
}

bool toolRecognizer_IDLServer_recognize::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readString(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void toolRecognizer_IDLServer_recognize::init(const int32_t tlx, const int32_t tly, const int32_t brx, const int32_t bry) {
  _return = "";
  this->tlx = tlx;
  this->tly = tly;
  this->brx = brx;
  this->bry = bry;
}

bool toolRecognizer_IDLServer_burst::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("burst",1,1)) return false;
  if (!writer.writeBool(burstF)) return false;
  return true;
}

bool toolRecognizer_IDLServer_burst::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void toolRecognizer_IDLServer_burst::init(const bool burstF) {
  _return = false;
  this->burstF = burstF;
}

toolRecognizer_IDLServer::toolRecognizer_IDLServer() {
  yarp().setOwner(*this);
}
bool toolRecognizer_IDLServer::start() {
  bool _return = false;
  toolRecognizer_IDLServer_start helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool toolRecognizer_IDLServer::start()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool toolRecognizer_IDLServer::quit() {
  bool _return = false;
  toolRecognizer_IDLServer_quit helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool toolRecognizer_IDLServer::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool toolRecognizer_IDLServer::train(const std::string& label, const int32_t tlx, const int32_t tly, const int32_t brx, const int32_t bry) {
  bool _return = false;
  toolRecognizer_IDLServer_train helper;
  helper.init(label,tlx,tly,brx,bry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool toolRecognizer_IDLServer::train(const std::string& label, const int32_t tlx, const int32_t tly, const int32_t brx, const int32_t bry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
std::string toolRecognizer_IDLServer::recognize(const int32_t tlx, const int32_t tly, const int32_t brx, const int32_t bry) {
  std::string _return = "";
  toolRecognizer_IDLServer_recognize helper;
  helper.init(tlx,tly,brx,bry);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","std::string toolRecognizer_IDLServer::recognize(const int32_t tlx, const int32_t tly, const int32_t brx, const int32_t bry)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool toolRecognizer_IDLServer::burst(const bool burstF) {
  bool _return = false;
  toolRecognizer_IDLServer_burst helper;
  helper.init(burstF);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool toolRecognizer_IDLServer::burst(const bool burstF)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool toolRecognizer_IDLServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "start") {
      bool _return;
      _return = start();
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
    if (tag == "train") {
      std::string label;
      int32_t tlx;
      int32_t tly;
      int32_t brx;
      int32_t bry;
      if (!reader.readString(label)) {
        reader.fail();
        return false;
      }
      if (!reader.readI32(tlx)) {
        tlx = 0;
      }
      if (!reader.readI32(tly)) {
        tly = 0;
      }
      if (!reader.readI32(brx)) {
        brx = 0;
      }
      if (!reader.readI32(bry)) {
        bry = 0;
      }
      bool _return;
      _return = train(label,tlx,tly,brx,bry);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "recognize") {
      int32_t tlx;
      int32_t tly;
      int32_t brx;
      int32_t bry;
      if (!reader.readI32(tlx)) {
        tlx = 0;
      }
      if (!reader.readI32(tly)) {
        tly = 0;
      }
      if (!reader.readI32(brx)) {
        brx = 0;
      }
      if (!reader.readI32(bry)) {
        bry = 0;
      }
      std::string _return;
      _return = recognize(tlx,tly,brx,bry);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "burst") {
      bool burstF;
      if (!reader.readBool(burstF)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = burst(burstF);
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

std::vector<std::string> toolRecognizer_IDLServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("start");
    helpString.push_back("quit");
    helpString.push_back("train");
    helpString.push_back("recognize");
    helpString.push_back("burst");
    helpString.push_back("help");
  }
  else {
    if (functionName=="start") {
      helpString.push_back("bool start() ");
      helpString.push_back("Start the module ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="quit") {
      helpString.push_back("bool quit() ");
      helpString.push_back("Quit the module ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="train") {
      helpString.push_back("bool train(const std::string& label, const int32_t tlx = 0, const int32_t tly = 0, const int32_t brx = 0, const int32_t bry = 0) ");
      helpString.push_back("Command to train tools by their label ");
      helpString.push_back("@return true/false on success/failure to train classifiers. ");
    }
    if (functionName=="recognize") {
      helpString.push_back("std::string recognize(const int32_t tlx = 0, const int32_t tly = 0, const int32_t brx = 0, const int32_t bry = 0) ");
      helpString.push_back("Classifies image into one of the learned tool categories. ");
      helpString.push_back("@return label of recognized tool class. ");
    }
    if (functionName=="burst") {
      helpString.push_back("bool burst(const bool burstF) ");
      helpString.push_back("Checks whether the hand is full or empty ");
      helpString.push_back("@return true/false  corresponding to full or empty hand ");
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


