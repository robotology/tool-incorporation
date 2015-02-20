// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <ToolFeat3D.h>

bool ToolFeat3D::read_toolname(yarp::os::idl::WireReader& reader) {
  if (!reader.readString(toolname)) {
    reader.fail();
    return false;
  }
  return true;
}
bool ToolFeat3D::nested_read_toolname(yarp::os::idl::WireReader& reader) {
  if (!reader.readString(toolname)) {
    reader.fail();
    return false;
  }
  return true;
}
bool ToolFeat3D::read_toolFeats(yarp::os::idl::WireReader& reader) {
  {
    toolFeats.clear();
    uint32_t _size0;
    yarp::os::idl::WireState _etype3;
    reader.readListBegin(_etype3, _size0);
    toolFeats.resize(_size0);
    uint32_t _i4;
    for (_i4 = 0; _i4 < _size0; ++_i4)
    {
      {
        toolFeats[_i4].clear();
        uint32_t _size5;
        yarp::os::idl::WireState _etype8;
        reader.readListBegin(_etype8, _size5);
        toolFeats[_i4].resize(_size5);
        uint32_t _i9;
        for (_i9 = 0; _i9 < _size5; ++_i9)
        {
          if (!reader.readDouble(toolFeats[_i4][_i9])) {
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
bool ToolFeat3D::nested_read_toolFeats(yarp::os::idl::WireReader& reader) {
  {
    toolFeats.clear();
    uint32_t _size10;
    yarp::os::idl::WireState _etype13;
    reader.readListBegin(_etype13, _size10);
    toolFeats.resize(_size10);
    uint32_t _i14;
    for (_i14 = 0; _i14 < _size10; ++_i14)
    {
      {
        toolFeats[_i14].clear();
        uint32_t _size15;
        yarp::os::idl::WireState _etype18;
        reader.readListBegin(_etype18, _size15);
        toolFeats[_i14].resize(_size15);
        uint32_t _i19;
        for (_i19 = 0; _i19 < _size15; ++_i19)
        {
          if (!reader.readDouble(toolFeats[_i14][_i19])) {
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
bool ToolFeat3D::read(yarp::os::idl::WireReader& reader) {
  if (!read_toolname(reader)) return false;
  if (!read_toolFeats(reader)) return false;
  return !reader.isError();
}

bool ToolFeat3D::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListHeader(2)) return false;
  return read(reader);
}

bool ToolFeat3D::write_toolname(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeString(toolname)) return false;
  return true;
}
bool ToolFeat3D::nested_write_toolname(yarp::os::idl::WireWriter& writer) {
  if (!writer.writeString(toolname)) return false;
  return true;
}
bool ToolFeat3D::write_toolFeats(yarp::os::idl::WireWriter& writer) {
  {
    if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(toolFeats.size()))) return false;
    std::vector<std::vector<double> > ::iterator _iter20;
    for (_iter20 = toolFeats.begin(); _iter20 != toolFeats.end(); ++_iter20)
    {
      {
        if (!writer.writeListBegin(BOTTLE_TAG_DOUBLE, static_cast<uint32_t>((*_iter20).size()))) return false;
        std::vector<double> ::iterator _iter21;
        for (_iter21 = (*_iter20).begin(); _iter21 != (*_iter20).end(); ++_iter21)
        {
          if (!writer.writeDouble((*_iter21))) return false;
        }
        if (!writer.writeListEnd()) return false;
      }
    }
    if (!writer.writeListEnd()) return false;
  }
  return true;
}
bool ToolFeat3D::nested_write_toolFeats(yarp::os::idl::WireWriter& writer) {
  {
    if (!writer.writeListBegin(BOTTLE_TAG_LIST, static_cast<uint32_t>(toolFeats.size()))) return false;
    std::vector<std::vector<double> > ::iterator _iter22;
    for (_iter22 = toolFeats.begin(); _iter22 != toolFeats.end(); ++_iter22)
    {
      {
        if (!writer.writeListBegin(BOTTLE_TAG_DOUBLE, static_cast<uint32_t>((*_iter22).size()))) return false;
        std::vector<double> ::iterator _iter23;
        for (_iter23 = (*_iter22).begin(); _iter23 != (*_iter22).end(); ++_iter23)
        {
          if (!writer.writeDouble((*_iter23))) return false;
        }
        if (!writer.writeListEnd()) return false;
      }
    }
    if (!writer.writeListEnd()) return false;
  }
  return true;
}
bool ToolFeat3D::write(yarp::os::idl::WireWriter& writer) {
  if (!write_toolname(writer)) return false;
  if (!write_toolFeats(writer)) return false;
  return !writer.isError();
}

bool ToolFeat3D::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  return write(writer);
}
bool ToolFeat3D::Editor::write(yarp::os::ConnectionWriter& connection) {
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
  return !writer.isError();
}
bool ToolFeat3D::Editor::read(yarp::os::ConnectionReader& connection) {
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
    }
    if (!writer.writeListHeader(3)) return false;
    writer.writeString("*** Available fields:");
    writer.writeString("toolname");
    writer.writeString("toolFeats");
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

yarp::os::ConstString ToolFeat3D::toString() {
  yarp::os::Bottle b;
  b.read(*this);
  return b.toString();
}
