// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_VoxFeat
#define YARP_THRIFT_GENERATOR_STRUCT_VoxFeat

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class VoxFeat;


class VoxFeat : public yarp::os::idl::WirePortable {
public:
  // Fields
  std::vector<double>  voxHist;

  // Default constructor
  VoxFeat() {
  }

  // Constructor with field values
  VoxFeat(const std::vector<double> & voxHist) : voxHist(voxHist) {
  }

  // Copy constructor
  VoxFeat(const VoxFeat& __alt) : WirePortable(__alt)  {
    this->voxHist = __alt.voxHist;
  }

  // Assignment operator
  const VoxFeat& operator = (const VoxFeat& __alt) {
    this->voxHist = __alt.voxHist;
    return *this;
  }

  // read and write structure on a connection
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);

private:
  bool write_voxHist(yarp::os::idl::WireWriter& writer);
  bool nested_write_voxHist(yarp::os::idl::WireWriter& writer);
  bool read_voxHist(yarp::os::idl::WireReader& reader);
  bool nested_read_voxHist(yarp::os::idl::WireReader& reader);

public:

  yarp::os::ConstString toString();

  // if you want to serialize this class without nesting, use this helper
  typedef yarp::os::idl::Unwrapped<VoxFeat > unwrapped;

  class Editor : public yarp::os::Wire, public yarp::os::PortWriter {
  public:

    Editor() {
      group = 0;
      obj_owned = true;
      obj = new VoxFeat;
      dirty_flags(false);
      yarp().setOwner(*this);
    }

    Editor(VoxFeat& obj) {
      group = 0;
      obj_owned = false;
      edit(obj,false);
      yarp().setOwner(*this);
    }

    bool edit(VoxFeat& obj, bool dirty = true) {
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

    VoxFeat& state() { return *obj; }

    void begin() { group++; }

    void end() {
      group--;
      if (group==0&&is_dirty) communicate();
    }
    void set_voxHist(const std::vector<double> & voxHist) {
      will_set_voxHist();
      obj->voxHist = voxHist;
      mark_dirty_voxHist();
      communicate();
      did_set_voxHist();
    }
    void set_voxHist(int index, const double elem) {
      will_set_voxHist();
      obj->voxHist[index] = elem;
      mark_dirty_voxHist();
      communicate();
      did_set_voxHist();
    }
    const std::vector<double> & get_voxHist() {
      return obj->voxHist;
    }
    virtual bool will_set_voxHist() { return true; }
    virtual bool did_set_voxHist() { return true; }
    void clean() {
      dirty_flags(false);
    }
    bool read(yarp::os::ConnectionReader& connection);
    bool write(yarp::os::ConnectionWriter& connection);
  private:

    VoxFeat *obj;

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
    void mark_dirty_voxHist() {
      if (is_dirty_voxHist) return;
      dirty_count++;
      is_dirty_voxHist = true;
      mark_dirty();
    }
    void dirty_flags(bool flag) {
      is_dirty = flag;
      is_dirty_voxHist = flag;
      dirty_count = flag ? 1 : 0;
    }
    bool is_dirty;
    int dirty_count;
    bool is_dirty_voxHist;
  };
};

#endif

