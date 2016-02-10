// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_ToolFeat3D
#define YARP_THRIFT_GENERATOR_STRUCT_ToolFeat3D

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class ToolFeat3D;


class ToolFeat3D : public yarp::os::idl::WirePortable {
public:
  // Fields
  std::string toolname;
  std::vector<std::vector<double> >  toolFeats;

  // Default constructor
  ToolFeat3D() : toolname("") {
  }

  // Constructor with field values
  ToolFeat3D(const std::string& toolname,const std::vector<std::vector<double> > & toolFeats) : toolname(toolname), toolFeats(toolFeats) {
  }

  // Copy constructor
  ToolFeat3D(const ToolFeat3D& __alt) : WirePortable(__alt)  {
    this->toolname = __alt.toolname;
    this->toolFeats = __alt.toolFeats;
  }

  // Assignment operator
  const ToolFeat3D& operator = (const ToolFeat3D& __alt) {
    this->toolname = __alt.toolname;
    this->toolFeats = __alt.toolFeats;
    return *this;
  }

  // read and write structure on a connection
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);

private:
  bool write_toolname(yarp::os::idl::WireWriter& writer);
  bool nested_write_toolname(yarp::os::idl::WireWriter& writer);
  bool write_toolFeats(yarp::os::idl::WireWriter& writer);
  bool nested_write_toolFeats(yarp::os::idl::WireWriter& writer);
  bool read_toolname(yarp::os::idl::WireReader& reader);
  bool nested_read_toolname(yarp::os::idl::WireReader& reader);
  bool read_toolFeats(yarp::os::idl::WireReader& reader);
  bool nested_read_toolFeats(yarp::os::idl::WireReader& reader);

public:

  yarp::os::ConstString toString();

  // if you want to serialize this class without nesting, use this helper
  typedef yarp::os::idl::Unwrapped<ToolFeat3D > unwrapped;

  class Editor : public yarp::os::Wire, public yarp::os::PortWriter {
  public:

    Editor() {
      group = 0;
      obj_owned = true;
      obj = new ToolFeat3D;
      dirty_flags(false);
      yarp().setOwner(*this);
    }

    Editor(ToolFeat3D& obj) {
      group = 0;
      obj_owned = false;
      edit(obj,false);
      yarp().setOwner(*this);
    }

    bool edit(ToolFeat3D& obj, bool dirty = true) {
      if (obj_owned) delete this->obj;
      this->obj = &obj;
      obj_owned = false;
      dirty_flags(dirty);
      return true;
    }

    virtual ~Editor() {
    if (obj_owned) delete obj;
    }

    bool isValid() const {
      return obj!=0/*NULL*/;
    }

    ToolFeat3D& state() { return *obj; }

    void begin() { group++; }

    void end() {
      group--;
      if (group==0&&is_dirty) communicate();
    }
    void set_toolname(const std::string& toolname) {
      will_set_toolname();
      obj->toolname = toolname;
      mark_dirty_toolname();
      communicate();
      did_set_toolname();
    }
    void set_toolFeats(const std::vector<std::vector<double> > & toolFeats) {
      will_set_toolFeats();
      obj->toolFeats = toolFeats;
      mark_dirty_toolFeats();
      communicate();
      did_set_toolFeats();
    }
    void set_toolFeats(int index, const std::vector<double> & elem) {
      will_set_toolFeats();
      obj->toolFeats[index] = elem;
      mark_dirty_toolFeats();
      communicate();
      did_set_toolFeats();
    }
    const std::string& get_toolname() {
      return obj->toolname;
    }
    const std::vector<std::vector<double> > & get_toolFeats() {
      return obj->toolFeats;
    }
    virtual bool will_set_toolname() { return true; }
    virtual bool will_set_toolFeats() { return true; }
    virtual bool did_set_toolname() { return true; }
    virtual bool did_set_toolFeats() { return true; }
    void clean() {
      dirty_flags(false);
    }
    bool read(yarp::os::ConnectionReader& connection);
    bool write(yarp::os::ConnectionWriter& connection);
  private:

    ToolFeat3D *obj;

    bool obj_owned;
    int group;

    void communicate() {
      if (group!=0) return;
      if (yarp().canWrite()) {
        yarp().write(*this);
        clean();
      }
    }
    void mark_dirty() {
      is_dirty = true;
    }
    void mark_dirty_toolname() {
      if (is_dirty_toolname) return;
      dirty_count++;
      is_dirty_toolname = true;
      mark_dirty();
    }
    void mark_dirty_toolFeats() {
      if (is_dirty_toolFeats) return;
      dirty_count++;
      is_dirty_toolFeats = true;
      mark_dirty();
    }
    void dirty_flags(bool flag) {
      is_dirty = flag;
      is_dirty_toolname = flag;
      is_dirty_toolFeats = flag;
      dirty_count = flag ? 2 : 0;
    }
    bool is_dirty;
    int dirty_count;
    bool is_dirty_toolname;
    bool is_dirty_toolFeats;
  };
};

#endif

