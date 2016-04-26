// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <ToolFeat3DwithOrient.h>

bool ToolFeat3DwithOrient::read_toolname(yarp::os::idl::WireReader& reader) {
  if (!reader.readString(toolname)) {
    reader.fail();
    return false;
  }
  return true;
}
bool ToolFeat3DwithOrient::nested_read_toolname(yarp::os::idl::WireReader& reader) {
  if (!reader.readString(toolname)) {
    reader.fail();
    return false;
  }
  return true;
}
bool ToolFeat3DwithOrient::read_toolFeats(yarp::os::idl::WireReader& reader) {
  {
    toolFeats.clear();
    uint32_t _size24;
    yarp::os::idl::WireState _etype27;
    reader.readListBegin(_etype27, _size24);
    toolFeats.resize(_size24);
    uint32_t _i28;
    for (_i28 = 0; _i28 < _size24; ++_i28)
    {
      {
        toolFeats[_i28].clear();
        uint32_t _size29;
        yarp::os::idl::WireState _etype32;
        reader.readListBegin(_etype32, _size29);
        toolFeats[_i28].resize(_size29);
        uint32_t _i33;
        for (_i33 = 0; _i33 < _size29; ++_i33)
        {
          if (!reader.readDouble(toolFeats[_i28][_i33])) {
            reader.fail();
            return false;
          }
        }
        reader.readListEnd();
      }
    }
    reader.readListEnd();
  }
  return true;
}
bool ToolFeat3DwithOrient::nested_read_toolFeats(yarp::os::idl::WireReader& reader) {
  {
    toolFeats.clear();
    uint32_t _size34;
    yarp::os::idl::WireState _etype37;
    reader.readListBegin(_etype37, _size34);
    toolFeats.resize(_size34);
    uint32_t _i38;
    for (_i38 = 0; _i38 < _size34; ++_i38)
    {
      {
        toolFeats[_i38].clear();
        uint32_t _size39;
        yarp::os::idl::WireState _etype42;
        reader.readListBegin(_etype42, _size39);
        toolFeats[_i38].resize(_size39);
        uint32_t _i43;
        for (_i43 = 0; _i43 < _size39; ++_i43)
        {
          if (!reader.readDouble(toolFeats[_i38][_i43])) {
            reader.fail();
            return false;
          }
        }
        reader.readListEnd();
      }
    }
    reader.readListEnd();
  }
  return true;
}
bool ToolFeat3DwithOrient::read_orientation(yarp::os::idl::WireReader& reader) {
  if (!reader.read(orientation)) {
    reader.fail();
    return false;
  }
  return true;
}
bool ToolFeat3DwithOrient::nested_read_orientation(yarp::os::idl::WireReader& reader) {
  if (!reader.readNested(orientation)) {
    reader.fail();
    return false;
  }
  return true;
}
bool ToolFeat3DwithOrient::read(yarp::os::idl::WireReader& reader) {
  if (!read_toolname(reader)) return false;
  if (!read_toolFeats(reader)) return false;
  if (!read_orientation(reader)) return false;
  return !reader.isError();
}

bool ToolFeat3DwithOrient::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(3)) return false;
  return read(reader);
}

bool ToolFeat3DwithOrient::write_toolname(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeString(toolname)) return false;
  return true;
}
bool ToolFeat3DwithOrient::nested_write_toolname(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeString(toolname)) return false;
  return true;
}
bool ToolFeat3DwithOrient::write_toolFeats(yarp::os::idl::WireWriter& writer) {
  {
    if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(toolFeats.size()))) return false;
    std::vector<std::vector<double> > ::iterator _iter44;
    for (_iter44 = toolFeats.begin(); _iter44 != toolFeats.end(); ++_iter44)
    {
      {
        if (!writer.writeListBegin(BOTTLE_TAG_DOUBLE, static_cast<uint32_t>((*_iter44).size()))) return false;
        std::vector<double> ::iterator _iter45;
        for (_iter45 = (*_iter44).begin(); _iter45 != (*_iter44).end(); ++_iter45)
        {
          if (!writer.writeDouble((*_iter45))) return false;
        }
        if (!writer.writeListEnd()) return false;
      }
    }
    if (!writer.writeListEnd()) return false;
  }
  return true;
}
bool ToolFeat3DwithOrient::nested_write_toolFeats(yarp::os::idl::WireWriter& writer) {
  {
    if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(toolFeats.size()))) return false;
    std::vector<std::vector<double> > ::iterator _iter46;
    for (_iter46 = toolFeats.begin(); _iter46 != toolFeats.end(); ++_iter46)
    {
      {
        if (!writer.writeListBegin(BOTTLE_TAG_DOUBLE, static_cast<uint32_t>((*_iter46).size()))) return false;
        std::vector<double> ::iterator _iter47;
        for (_iter47 = (*_iter46).begin(); _iter47 != (*_iter46).end(); ++_iter47)
        {
          if (!writer.writeDouble((*_iter47))) return false;
        }
        if (!writer.writeListEnd()) return false;
      }
    }
    if (!writer.writeListEnd()) return false;
  }
  return true;
}
bool ToolFeat3DwithOrient::write_orientation(yarp::os::idl::WireWriter& writer) {
  if (!writer.write(orientation)) return false;
  return true;
}
bool ToolFeat3DwithOrient::nested_write_orientation(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeNested(orientation)) return false;
  return true;
}
bool ToolFeat3DwithOrient::write(yarp::os::idl::WireWriter& writer) {
  if (!write_toolname(writer)) return false;
  if (!write_toolFeats(writer)) return false;
  if (!write_orientation(writer)) return false;
  return !writer.isError();
}

bool ToolFeat3DwithOrient::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  return write(writer);
}
bool ToolFeat3DwithOrient::Editor::write(yarp::os::ConnectionWriter& connection) {
  if (!isValid()) return false;
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(dirty_count+1)) return false;
  if (!writer.writeString("patch")) return false;
  if (is_dirty_toolname) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("toolname")) return false;
    if (!obj->nested_write_toolname(writer)) return false;
  }
  if (is_dirty_toolFeats) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("toolFeats")) return false;
    if (!obj->nested_write_toolFeats(writer)) return false;
  }
  if (is_dirty_orientation) {
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeString("set")) return false;
    if (!writer.writeString("orientation")) return false;
    if (!obj->nested_write_orientation(writer)) return false;
  }
  return !writer.isError();
}
bool ToolFeat3DwithOrient::Editor::read(yarp::os::ConnectionReader& connection) {
  if (!isValid()) return false;
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) return false;
  int len = reader.getLength();
  if (len==0) {
    yarp::os::idl::WireWriter writer(reader);
    if (writer.isNull()) return true;
    if (!writer.writeListHeader(1)) return false;
    writer.writeString("send: 'help' or 'patch (param1 val1) (param2 val2)'");
    return true;
  }
  yarp::os::ConstString tag;
  if (!reader.readString(tag)) return false;
  if (tag=="help") {
    yarp::os::idl::WireWriter writer(reader);
    if (writer.isNull()) return true;
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("many",1, 0)) return false;
    if (reader.getLength()>0) {
      yarp::os::ConstString field;
      if (!reader.readString(field)) return false;
      if (field=="toolname") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("std::string toolname")) return false;
      }
      if (field=="toolFeats") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("std::vector<std::vector<double> >  toolFeats")) return false;
      }
      if (field=="orientation") {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeString("yarp::sig::Matrix orientation")) return false;
      }
    }
    if (!writer.writeListHeader(4)) return false;
    writer.writeString("*** Available fields:");
    writer.writeString("toolname");
    writer.writeString("toolFeats");
    writer.writeString("orientation");
    return true;
  }
  bool nested = true;
  bool have_act = false;
  if (tag!="patch") {
    if ((len-1)%2 != 0) return false;
    len = 1 + ((len-1)/2);
    nested = false;
    have_act = true;
  }
  for (int i=1; i<len; i++) {
    if (nested && !reader.readListHeader(3)) return false;
    yarp::os::ConstString act;
    yarp::os::ConstString key;
    if (have_act) {
      act = tag;
    } else {
      if (!reader.readString(act)) return false;
    }
    if (!reader.readString(key)) return false;
    // inefficient code follows, bug paulfitz to improve it
    if (key == "toolname") {
      will_set_toolname();
      if (!obj->nested_read_toolname(reader)) return false;
      did_set_toolname();
    } else if (key == "toolFeats") {
      will_set_toolFeats();
      if (!obj->nested_read_toolFeats(reader)) return false;
      did_set_toolFeats();
    } else if (key == "orientation") {
      will_set_orientation();
      if (!obj->nested_read_orientation(reader)) return false;
      did_set_orientation();
    } else {
      // would be useful to have a fallback here
    }
  }
  reader.accept();
  yarp::os::idl::WireWriter writer(reader);
  if (writer.isNull()) return true;
  writer.writeListHeader(1);
  writer.writeVocab(VOCAB2('o','k'));
  return true;
}

yarp::os::ConstString ToolFeat3DwithOrient::toString() {
  yarp::os::Bottle b;
  b.read(*this);
  return b.toString();
}
