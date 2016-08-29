#include <boost/type_index.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>

// Frame timestamp is just an int (POD)
struct FrameTimestamp {
  int timestamp;
};

// String literal suffix to reduce syntax noise. FrameTimestamp{42} can be
// written as 42_ft.
FrameTimestamp operator"" _ft(unsigned long long int t) {
  return FrameTimestamp{static_cast<int>(t)};
}

std::ostream& operator<<(std::ostream& os, FrameTimestamp const& t) {
  os << t.timestamp;
  return os;
}

bool operator==(FrameTimestamp const& l, FrameTimestamp const& r) {
  return l.timestamp == r.timestamp;
}

bool operator!=(FrameTimestamp const& l, FrameTimestamp const& r) {
  return !(l == r);
}

//------------------------------------------------------------------------------

// Allows us to generate lots of different frame names with a static string
// member by doing a using with a tag struct.
template <typename Tag>
struct FrameName {
  static std::string const name;
};

// Frame which can be set at runtime
struct RuntimeFrameName {
  std::string name;
};

// Suffix for allowing less noisy syntax RuntimeFrameName{"foo"} can be written
// as "foo"_frame
RuntimeFrameName operator"" _frame(char const* name, std::size_t) {
  return RuntimeFrameName{name};
}

// Traits for working out if a frame name is not known until runtime.
namespace detail {

template <typename T>
struct is_runtime_frame : std::false_type {};

template <template <typename> class T>
struct is_runtime_frame<T<RuntimeFrameName>> : std::true_type {};

}  // namespace detail

template <typename T>
struct is_runtime_frame {
  static constexpr bool const value =
      detail::is_runtime_frame<std::decay_t<T>>::value;
};

//------------------------------------------------------------------------------

// There are three kinds of frame:
//  - LocalFrame
//  - ExternalFrame
//  - CalibrationFrame
// Each frame is templated on the NameType which can be either known at compile
// time (i.e. FrameName<Tag>) or a RuntimeFrameName. An ExternalFrame represents
// a fixed invariant point. GPS zones fit into this category. A CalibrationFrame
// allows the representation of calibration transforms which have different
// frame composition behaviour. LocalFrames represent any frame which can move.
// These frames have an additional timestamp field which allows us to
// destinguish between two different timepoints of the same physical frame.

template <typename NameType>
struct LocalFrame {
  using FrameNameType = NameType;

  NameType name_;
  FrameTimestamp timestamp_;

  template <bool Enabled = true>
  LocalFrame(FrameTimestamp const& timestamp,
             typename std::enable_if<!is_runtime_frame<NameType>::value &&
                                     Enabled>::type* = nullptr)
      : name_(NameType()), timestamp_(timestamp) {}

  LocalFrame(NameType const& name, FrameTimestamp const& timestamp)
      : name_(name), timestamp_(timestamp) {}

  std::string const& name() const { return name_.name; }

  FrameTimestamp const& timestamp() const { return timestamp_; }
};

using RuntimeLocalFrame = LocalFrame<RuntimeFrameName>;

template <typename NameType>
std::ostream& operator<<(std::ostream& os, LocalFrame<NameType> const& f) {
  os << f.name() << "@" << f.timestamp();
  return os;
}

template <typename NameType>
struct ExternalFrame {
  using FrameNameType = NameType;

  template <bool Enabled = true>
  ExternalFrame(typename std::enable_if<!is_runtime_frame<NameType>::value &&
                                        Enabled>::type* = nullptr)
      : name_(NameType()) {}

  ExternalFrame(NameType name) : name_(std::move(name)) {}

  NameType name_;

  std::string const& name() const { return name_.name; }
};

using RuntimeExternalFrame = ExternalFrame<RuntimeFrameName>;

template <typename NameType>
std::ostream& operator<<(std::ostream& os, ExternalFrame<NameType> const& f) {
  os << f.name();
  return os;
}

template <typename NameType>
struct CalibrationFrame {
  using FrameNameType = NameType;

  NameType name_;

  template <bool Enabled = true>
  CalibrationFrame(typename std::enable_if<!is_runtime_frame<NameType>::value &&
                                           Enabled>::type* = nullptr)
      : name_(NameType()) {}

  CalibrationFrame(NameType name) : name_(std::move(name)) {}

  std::string const& name() const { return name_.name; }
};

using RuntimeCalibrationFrame = CalibrationFrame<RuntimeFrameName>;

template <typename NameType>
std::ostream& operator<<(std::ostream& os,
                         CalibrationFrame<NameType> const& f) {
  os << f.name();
  return os;
}

namespace detail {

template <typename T>
struct is_frame : std::false_type {};

template <typename NameType>
struct is_frame<LocalFrame<NameType>> : std::true_type {};

template <typename NameType>
struct is_frame<ExternalFrame<NameType>> : std::true_type {};

template <typename NameType>
struct is_frame<CalibrationFrame<NameType>> : std::true_type {};

template <typename T>
struct is_local_frame : std::false_type {};

template <typename NameType>
struct is_local_frame<LocalFrame<NameType>> : std::true_type {};

template <typename T>
struct is_world_frame : std::false_type {};

template <typename NameType>
struct is_world_frame<ExternalFrame<NameType>> : std::true_type {};

template <typename T>
struct is_calibration_frame : std::false_type {};

template <typename NameType>
struct is_calibration_frame<CalibrationFrame<NameType>> : std::true_type {};

}  // namespace detail

template <typename T>
struct is_frame {
  static constexpr bool const value = detail::is_frame<std::decay_t<T>>::value;
};

template <typename T>
struct is_local_frame {
  static constexpr bool const value =
      detail::is_local_frame<std::decay_t<T>>::value;
};

template <typename T>
struct is_world_frame {
  static constexpr bool const value =
      detail::is_world_frame<std::decay_t<T>>::value;
};

template <typename T>
struct is_calibration_frame {
  static constexpr bool const value =
      detail::is_calibration_frame<std::decay_t<T>>::value;
};

namespace detail {
// Perform compile time frame checking if possible
template <typename L, typename R>
constexpr bool CheckFramesMatch(L const& l, R const& r) {
  static_assert((is_runtime_frame<L>::value || is_runtime_frame<R>::value ||
                 std::is_same<typename L::FrameNameType,
                              typename R::FrameNameType>::value),
                "Frame mismatch");
  return l.name() == r.name();
}

// Default matching mechanism.
template <typename L, typename R>
struct FramesMatchImpl {
  constexpr bool operator()(L const& l, R const& r) {
    return CheckFramesMatch(l,r);
  }
};

// Specialisation for local frames. Adds timestamp checking
template <typename LFrameTag, typename RFrameTag>
struct FramesMatchImpl<LocalFrame<LFrameTag>, LocalFrame<RFrameTag>> {
  constexpr bool operator()(LocalFrame<LFrameTag> const& l,
                            LocalFrame<RFrameTag> const& r) {
    return l.timestamp_ == r.timestamp_ && CheckFramesMatch(l,r);
  }
};
} // namespace detail

template <typename L, typename R>
constexpr bool FramesMatch(L const& l, R const& r) {
  return detail::FramesMatchImpl<L, R>()(l, r);
}

//------------------------------------------------------------------------------

template <typename DestFrame, typename SrcFrame>
struct Transform {
  using DestFrameType = DestFrame;
  using SrcFrameType = SrcFrame;

  DestFrameType dest_frame_;
  SrcFrameType src_frame_;

  Transform(DestFrame dest_frame, SrcFrame src_frame)
      : dest_frame_(dest_frame), src_frame_(src_frame) {
    static_assert(is_frame<DestFrame>::value, "Invalid destination frame");
    static_assert(is_frame<SrcFrame>::value, "Invalid source frame");

    static_assert((is_calibration_frame<DestFrame>::value &&
                   is_calibration_frame<SrcFrame>::value) ||
                      !(is_calibration_frame<DestFrame>::value ||
                        is_calibration_frame<SrcFrame>::value),
                  "Cannot mix a calibration frame and any other frame type");
  }

  // Enable a transform taking two timestamps if both frames are local
  template <bool Enabled = true>
  Transform(FrameTimestamp dest_time, FrameTimestamp src_time,
            typename std::enable_if<!is_runtime_frame<DestFrame>::value &&
                                    !is_runtime_frame<SrcFrame>::value &&
                                    is_local_frame<DestFrame>::value &&
                                    is_local_frame<SrcFrame>::value &&
                                    Enabled>::type* x = nullptr)
      : Transform(DestFrame(dest_time), SrcFrame(src_time)) {}

  // Enabled only if we can default initialise the frames.
  template <bool Enabled = true>
  Transform(typename std::enable_if<!is_runtime_frame<DestFrame>::value &&
                                    !is_runtime_frame<SrcFrame>::value &&
                                    !is_local_frame<DestFrame>::value &&
                                    !is_local_frame<SrcFrame>::value &&
                                    Enabled>::type* x = nullptr)
      : Transform(DestFrame(), SrcFrame()) {}
};

template <typename DestFrame, typename SrcFrame>
Transform<DestFrame, SrcFrame> MakeTransform(DestFrame dest, SrcFrame src) {
  return Transform<DestFrame, SrcFrame>(dest, src);
}

template <typename DestName, typename SrcName>
Transform<CalibrationFrame<DestName>, CalibrationFrame<SrcName>>
MakeCalibrationTransform(DestName dest, SrcName src) {
  return Transform<CalibrationFrame<DestName>, CalibrationFrame<SrcName>>(dest,
                                                                          src);
}

template <typename DestName, typename SrcName>
Transform<CalibrationFrame<DestName>, CalibrationFrame<SrcName>>
MakeCalibrationTransform() {
  return Transform<CalibrationFrame<DestName>, CalibrationFrame<SrcName>>(
      CalibrationFrame<DestName>(), CalibrationFrame<SrcName>());
}

template <typename DestName, typename SrcName>
Transform<LocalFrame<DestName>, LocalFrame<SrcName>> MakeTimestampedTransform(
    FrameTimestamp dest_time, FrameTimestamp src_time) {
  return Transform<LocalFrame<DestName>, LocalFrame<SrcName>>(
      LocalFrame<DestName>(dest_time), LocalFrame<SrcName>(src_time));
}

template <typename DestName, typename SrcName>
Transform<LocalFrame<DestName>, LocalFrame<SrcName>> MakeTimestampedTransform(
    DestName dest_name, FrameTimestamp dest_time, SrcName src_name,
    FrameTimestamp src_time) {
  return Transform<LocalFrame<DestName>, LocalFrame<SrcName>>(
      LocalFrame<DestName>(dest_name, dest_time),
      LocalFrame<SrcName>(src_name, src_time));
}

template <typename FrameName>
Transform<LocalFrame<FrameName>, LocalFrame<FrameName>> MakeRelativeTransform(
    FrameTimestamp dest_time, FrameTimestamp src_time) {
  return Transform<LocalFrame<FrameName>, LocalFrame<FrameName>>(
      LocalFrame<FrameName>(dest_time), LocalFrame<FrameName>(src_time));
}

template <typename DestFrame, typename SrcFrame>
std::ostream& operator<<(std::ostream& os,
                         Transform<DestFrame, SrcFrame> const& t) {
  os << "G_{" << t.dest_frame_ << "," << t.src_frame_ << "}";
  return os;
}

template <typename LDestFrame, typename LSrcFrame, typename RDestFrame,
          typename RSrcFrame>
struct TransformComposition {
  using ResultType = Transform<LDestFrame, RSrcFrame>;

  ResultType operator()(Transform<LDestFrame, LSrcFrame> const& l,
                        Transform<RDestFrame, RSrcFrame> const& r) {
    return ResultType(l.dest_frame_, r.src_frame_);
  }
};

template <typename LDestTag, typename LSrcTag, typename RDestTag,
          typename RSrcTag>
struct TransformComposition<
    CalibrationFrame<LDestTag>, CalibrationFrame<LSrcTag>,
    CalibrationFrame<RDestTag>, CalibrationFrame<RSrcTag>> {
  using ResultType =
      Transform<CalibrationFrame<LDestTag>, CalibrationFrame<RSrcTag>>;

  ResultType operator()(
      Transform<CalibrationFrame<LDestTag>, CalibrationFrame<LSrcTag>> const& l,
      Transform<CalibrationFrame<RDestTag>, CalibrationFrame<RSrcTag>> const&
          r) {
    return ResultType(l.dest_frame_, r.src_frame_);
  }
};

template <typename LDestTag, typename LSrcTag, typename RDestTag,
          template <typename> class RSrcFrame, typename RSrcTag>
struct TransformComposition<CalibrationFrame<LDestTag>,
                            CalibrationFrame<LSrcTag>, LocalFrame<RDestTag>,
                            RSrcFrame<RSrcTag>> {
  using ResultType = Transform<LocalFrame<LDestTag>, RSrcFrame<RSrcTag>>;

  ResultType operator()(
      Transform<CalibrationFrame<LDestTag>, CalibrationFrame<LSrcTag>> const& l,
      Transform<LocalFrame<RDestTag>, RSrcFrame<RSrcTag>> const& r) {
    LocalFrame<LDestTag> dest_frame(l.dest_frame_.name_,
                                    r.dest_frame_.timestamp_);

    return ResultType(dest_frame, r.src_frame_);
  }
};

template <template <typename> class LDestFrame, typename LDestTag,
          typename LSrcTag, typename RDestTag, typename RSrcTag>
struct TransformComposition<LDestFrame<LDestTag>, LocalFrame<LSrcTag>,
                            CalibrationFrame<RDestTag>,
                            CalibrationFrame<RSrcTag>> {
  using ResultType = Transform<LDestFrame<LDestTag>, LocalFrame<RSrcTag>>;

  ResultType operator()(
      Transform<LDestFrame<LDestTag>, LocalFrame<LSrcTag>> const& l,
      Transform<CalibrationFrame<RDestTag>, CalibrationFrame<RSrcTag>> const&
          r) {
    LocalFrame<RSrcTag> src_frame(r.src_frame_.name_, l.src_frame_.timestamp_);

    return ResultType(l.dest_frame_, src_frame);
  }
};

template <typename LDestFrame, typename LSrcFrame, typename RDestFrame,
          typename RSrcFrame>
auto operator*(Transform<LDestFrame, LSrcFrame> const& l,
               Transform<RDestFrame, RSrcFrame> const& r) ->
    typename TransformComposition<LDestFrame, LSrcFrame, RDestFrame,
                                  RSrcFrame>::ResultType {
  if (!FramesMatch(l.src_frame_, r.dest_frame_)) {
    std::stringstream ss;
    ss << "Error composing " << l << " and " << r;
    throw ss.str();
  }
  return TransformComposition<LDestFrame, LSrcFrame, RDestFrame, RSrcFrame>()(
      l, r);
}

template <typename DestFrame, typename SrcFrame>
auto Invert(Transform<DestFrame, SrcFrame> const& G_dest_src) {
  return Transform<SrcFrame, DestFrame>(G_dest_src.src_frame_,
                                        G_dest_src.dest_frame_);
}

//------------------------------------------------------------------------------

using WorldFrame = FrameName<struct WorldFrameTag>;
template <>
std::string const WorldFrame::name{"world"};

using RobotFrame = FrameName<struct RobotFrameTag>;
template <>
std::string const RobotFrame::name{"robot"};

using RobotCalibrationFrame = CalibrationFrame<RobotFrame>;
using RobotLocalFrame = LocalFrame<RobotFrame>;

using CameraFrame = FrameName<struct CameraFrameTag>;
template <>
std::string const CameraFrame::name{"camera"};
using CameraCalibrationFrame = CalibrationFrame<CameraFrame>;
using CameraLocalFrame = LocalFrame<CameraFrame>;

using LaserFrame = FrameName<struct LaserFrameTag>;
template <>
std::string const LaserFrame::name{"laser"};

template <typename T1, typename T2>
void PrintTransformComposition(T1 const& t1, T2 const& t2) {
  std::cout << t1 << " * " << t2 << " = " << t1 * t2 << " and has type "
            << boost::typeindex::type_id<decltype(t1 * t2)>().pretty_name()
            << "\n";
}

template <typename T1, typename T2, typename T3>
void PrintTransformComposition(T1 const& t1, T2 const& t2, T3 const& t3) {
  std::cout << t1 << " * " << t2 << " * " << t3 << " = " << t1 * t2 * t3
            << " and has type "
            << boost::typeindex::type_id<decltype(t1 * t2 * t3)>().pretty_name()
            << "\n";
}

int main(int argc, const char* argv[]) {

  auto G_robot_camera_calibration =
      MakeCalibrationTransform<RobotFrame, CameraFrame>();
  auto G_laser_robot_calibration =
      MakeCalibrationTransform<LaserFrame, RobotFrame>();
  // ERROR enable_if removes this constructor if using runtime frames
//MakeCalibrationTransform<RuntimeCalibrationFrame, RobotCalibrationFrame>();
  
  // Compose two calibration transforms
  PrintTransformComposition(G_laser_robot_calibration,
                            G_robot_camera_calibration);

  // Lets make a transform chain. Lets do the first transform with an easy
  // helper function. We want both frames to be LocalFrames (have timestamps)
  // and in this case we know that it is a common Dest/Src frame _name_
  // (RobotFrame). Therefore the only interesting information is the timestamps.
  // The helper function constructs the frames for us and then builds the
  // transform.
  auto G_robot_t1_robot_t2 = MakeRelativeTransform<RobotFrame>(1_ft, 2_ft);
  // Under the hood this is essentially calling the constructor with the frame
  // type LocalFrame<RobotFrame>
  auto G_robot_t2_robot_t3 =
      Transform<RobotLocalFrame, RobotLocalFrame>(2_ft, 3_ft);
  // Expanded and removing the all the typedefs and we get this monstrosity.
  // This is why auto is awsome - otherwise this is what you'd be declaring the
  // type as.
  auto G_robot_t3_robot_t4 = Transform<LocalFrame<FrameName<RobotFrameTag>>,
                                       LocalFrame<FrameName<RobotFrameTag>>>(
      LocalFrame<FrameName<RobotFrameTag>>(3_ft),
      LocalFrame<FrameName<RobotFrameTag>>(4_ft));
  // Compose three local transforms
  PrintTransformComposition(G_robot_t1_robot_t2, G_robot_t2_robot_t3,
                            G_robot_t3_robot_t4);

  // Project a relative transform into a different frame using a calibration.
  PrintTransformComposition(Invert(G_robot_camera_calibration),
                            G_robot_t1_robot_t2, G_robot_camera_calibration);

  // Get new calibration frame at runtime.
  // Note, we are making a runtime robot frame. We could use the compile time
  // one but this shows that mixing runtime and compile time frames works fine.
  // We have specified the type here to be expicit about it being a different
  // type for the same frame string, however, we can remove the heavy syntax
  // using the _frame suffix, shown for the second parameter.
  auto G_robot_ins_calibration =
  MakeCalibrationTransform(RuntimeFrameName{"robot"}, "ins"_frame);
  // Compose a transform with runtime frames and one with compile time tag.
  PrintTransformComposition(G_laser_robot_calibration, G_robot_ins_calibration);

  // Composition of bad compile time frames.
  // G_robot_camera_calibration * G_laser_robot_calibration;  // ERROR

  // Compisition of bad frame names, checked at runtime.
  try {
    G_robot_ins_calibration* G_laser_robot_calibration;
  } catch (std::string& ex) {
    std::cout << ex << std::endl;
  }

  // Composition of bad frame timestamps, matching names.
  try {
    G_robot_t2_robot_t3* G_robot_t1_robot_t2;
  } catch (std::string& ex) {
    std::cout << ex << std::endl;
  }

  // The robot in the world frame.
  auto G_world_ins_t3 = MakeTransform(ExternalFrame<WorldFrame>(),
                                      RuntimeLocalFrame("ins"_frame, 3_ft));
  PrintTransformComposition(G_world_ins_t3, Invert(G_robot_ins_calibration));

  // ERROR Cannot create a transform where one frame is Calibration and the
  // other is not.
//  Transform<RuntimeCalibrationFrame, RuntimeLocalFrame>(
//      RuntimeCalibrationFrame("robot"_frame),
//      RuntimeLocalFrame("laser"_frame,1_ft));
  
  // Cannot apply calibration transform to a world frame.
  // NOTE A frame name does not encode the frame type to which it will be
  // applied (as this can be any of the three types). To prevent this at compile
  // time would require separating frame names into external and non-external
  // types. Things were already pretty ugl. I tried inheritance. That went
  // badly. I gave up. Whilst you can create this transform, as soon as you try
  // and compose it with anything which isn't a calibration transform it will go
  // bang.
//  auto G_robot_world_calibration =
//      MakeCalibrationTransform<RobotFrame, WorldFrame>();
//  G_robot_world_calibration* G_world_ins_t3;  // ERROR


  return 0;
}
