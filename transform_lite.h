#pragma once
#ifndef TRANSFORM_LITE_TRANSFORM_LITE_H
#define TRANSFORM_LITE_TRANSFORM_LITE_H

#include <sstream>
#include <string>
#include <type_traits>

#include <boost/utility/string_view.hpp>

#include "constexpr_string.h"

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
  //using TagType = Tag;
  //static std::string const name;
  //static constexpr constexpr_string name;
};

// I had to use a macro. This will define a template speicialisation for the
// a frame known at compilte time. Provde the name you want a type alias to be
// created for and a string
#define DEFINE_FRAME_NAME(FN, string_n)                \
  struct FN##Tag {};                                   \
  template <>                                          \
  struct FrameName<FN##Tag> {                          \
    using TagType = FN##Tag;                           \
    static constexpr constexpr_string name{#string_n}; \
  };                                                   \
  constexpr constexpr_string FrameName<FN##Tag>::name; \
  using FN = FrameName<FN##Tag>;

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
struct is_runtime_frame_name : std::false_type {};

template <>
struct is_runtime_frame_name<RuntimeFrameName> : std::true_type {};

}  // namespace detail

template <typename T>
struct is_runtime_frame_name
    : detail::is_runtime_frame_name<std::remove_cv_t<T>> {};

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

  // The SFINAE trick here is because you cannot enable_if on the a class type.
  // This is because NameType is seen and all the functions are stamped out at
  // that point. It will then fail to get a 'type' and say enbale_if cannot be
  // used. The trick is to add a single layer of indirection by having
  // SFINAENameType the a default of NameType. In general users should never
  // specify function template types, but in the case of constructors they
  // cannot. It's a compiler error. However, by adding the indirection, the
  // compilre waits until the point where the constructor is called to realise
  // that it is going to use the default argument. From here enable_if works as
  // always.
  template <typename SFINAENameType = NameType>
  LocalFrame(
      FrameTimestamp const& timestamp,
      typename std::enable_if<
          !is_runtime_frame_name<SFINAENameType>::value>::type* = nullptr)
      : LocalFrame(NameType(), timestamp) {}

  LocalFrame(NameType const& name, FrameTimestamp const& timestamp)
      : name_(name), timestamp_(timestamp) {}

  boost::string_view name() const { return static_cast<boost::string_view>(name_.name); }

  constexpr FrameTimestamp const& timestamp() const { return timestamp_; }
};

using RuntimeLocalFrame = LocalFrame<RuntimeFrameName>;

template <typename NameType>
std::ostream& operator<<(std::ostream& os, LocalFrame<NameType> const& f) {
  os << f.name() << "@" << f.timestamp();
  return os;
}

//------------------------------------------------------------------------------

template <typename NameType>
struct ExternalFrame {
  using FrameNameType = NameType;

  template <typename SFINAENameType = NameType>
  ExternalFrame(
      typename std::enable_if<
          !is_runtime_frame_name<SFINAENameType>::value>::type* = nullptr)
      : name_(NameType()) {}

  ExternalFrame(NameType name) : name_(std::move(name)) {}

  NameType name_;

  boost::string_view name() const { return static_cast<boost::string_view>(name_.name); }
};

using RuntimeExternalFrame = ExternalFrame<RuntimeFrameName>;

template <typename NameType>
std::ostream& operator<<(std::ostream& os, ExternalFrame<NameType> const& f) {
  os << f.name();
  return os;
}

//------------------------------------------------------------------------------

template <typename NameType>
struct CalibrationFrame {
  using FrameNameType = NameType;

  NameType name_;

  template <typename SFINAENameType = NameType>
  CalibrationFrame(
      typename std::enable_if<
          !is_runtime_frame_name<SFINAENameType>::value>::type* = nullptr)
      : name_(NameType()) {}

  CalibrationFrame(NameType name) : name_(std::move(name)) {}

  boost::string_view name() const { return static_cast<boost::string_view>(name_.name); }
};

using RuntimeCalibrationFrame = CalibrationFrame<RuntimeFrameName>;

template <typename NameType>
std::ostream& operator<<(std::ostream& os,
                         CalibrationFrame<NameType> const& f) {
  os << f.name();
  return os;
}

//------------------------------------------------------------------------------

// Implementation of frame traits.
namespace detail {

template <typename T>
struct is_frame_impl : std::false_type {};

template <typename T>
struct is_frame_impl<LocalFrame<T>> : std::true_type {};

template <typename T>
struct is_frame_impl<ExternalFrame<T>> : std::true_type {};

template <typename T>
struct is_frame_impl<CalibrationFrame<T>> : std::true_type {};

template <typename T>
struct is_local_frame_impl : std::false_type {};

template <typename T>
struct is_local_frame_impl<LocalFrame<T>> : std::true_type {};

template <typename T>
struct is_world_frame_impl : std::false_type {};

template <typename T>
struct is_world_frame_impl<ExternalFrame<T>> : std::true_type {};

template <typename T>
struct is_calibration_frame_impl : std::false_type {};

template <typename T>
struct is_calibration_frame_impl<CalibrationFrame<T>> : std::true_type {};

}  // namespace detail

template <typename T>
struct is_frame : detail::is_frame_impl<typename std::remove_cv_t<T>> {};

template <typename T>
constexpr bool is_frame_v = is_frame<T>::value;

template <typename T>
struct is_local_frame
    : detail::is_local_frame_impl<typename std::remove_cv_t<T>> {};

template <typename T>
constexpr bool is_local_frame_v = is_local_frame<T>::value;

template <typename T>
struct is_world_frame
    : detail::is_world_frame_impl<typename std::remove_cv_t<T>> {};

template <typename T>
constexpr bool is_world_frame_v = is_world_frame<T>::value;

template <typename T>
struct is_calibration_frame
    : detail::is_calibration_frame_impl<typename std::remove_cv_t<T>> {};

template <typename T>
constexpr bool is_calibration_frame_v = is_calibration_frame<T>::value;

template <typename T>
struct is_runtime_frame : is_runtime_frame_name<typename T::FrameNameType> {};

template <typename T>
constexpr bool is_runtime_frame_v = is_runtime_frame<T>::value;

//------------------------------------------------------------------------------

// Implementation of frame matching
namespace detail {

template <typename L, typename R>
struct is_compile_time_checkable;  // : std::true_type {};

template <typename LTag, typename RTag>
struct is_compile_time_checkable<FrameName<LTag>, FrameName<RTag>>
    : std::true_type {};

template <>
struct is_compile_time_checkable<RuntimeFrameName, RuntimeFrameName>
    : std::false_type {};

template <typename LhsFrameName>
struct is_compile_time_checkable<LhsFrameName, RuntimeFrameName>
    : std::false_type {};

template <typename RhsFrameName>
struct is_compile_time_checkable<RuntimeFrameName, RhsFrameName>
    : std::false_type {};

template <typename L, typename R>
constexpr bool is_compile_time_checkable_v =
    is_compile_time_checkable<std::decay_t<L>, std::decay_t<R>>::value;

template <typename L, typename R, bool CheckAtCompileTime>
struct frame_name_equal_impl;

template <typename L, typename R>
struct frame_name_equal_impl<L, R, false> {
  bool operator()(L const& l, R const& r) { return l.name == r.name; }
};

template <typename FrameName>
struct frame_name_equal_impl<FrameName, FrameName, true> {
  constexpr bool operator()(FrameName const&, FrameName const&) { return true; }
};

template <typename L, typename R>
struct frame_name_equal_impl<L, R, true> {
  constexpr bool operator()(L const&, R const&) {
    static_assert(std::is_same<L, R>::value, "frame mismatch");
    return false;
  }
};

}  // namespace detail

template <typename L, typename R>
constexpr bool frame_name_equal(L&& l, R&& r) {
  return detail::frame_name_equal_impl<
      L, R, detail::is_compile_time_checkable_v<L, R>>()(l, r);
}

namespace detail {

template <typename L, typename R>
struct frame_equal_impl;

template <template <typename> class LhsFrameType, typename LhsFrameName,
    template <typename> class RhsFrameType, typename RhsFrameName>
struct frame_equal_impl<LhsFrameType<LhsFrameName>,
                        RhsFrameType<RhsFrameName>> {
  constexpr bool operator()(LhsFrameType<LhsFrameName> const& l,
                            RhsFrameType<RhsFrameName> const& r) {
    return frame_name_equal(l.name_, r.name_);
  }
};

template <typename LhsFrameName, typename RhsFrameName>
struct frame_equal_impl<LocalFrame<LhsFrameName>, LocalFrame<RhsFrameName>> {
  constexpr bool operator()(LocalFrame<LhsFrameName> const& l,
                            LocalFrame<RhsFrameName> const& r) {
    return frame_name_equal(l.name_, r.name_) && l.timestamp() == r.timestamp();
  }
};

} // namespace detail

template <typename L, typename R>
constexpr bool frame_equal(L const& l, R const& r) {
  return detail::frame_equal_impl<L, R>()(l, r);
};

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

  template <typename SFINAEDestFrame = DestFrame,
      typename SFINAESrcFrame = SrcFrame>
  Transform(
      RuntimeFrameName dest_frame_name, FrameTimestamp dest_frame_timestamp,
      RuntimeFrameName src_frame_name, FrameTimestamp src_frame_timestamp,
      typename std::enable_if<is_runtime_frame<SFINAEDestFrame>::value &&
          is_runtime_frame<SFINAESrcFrame>::value>::type* =
      nullptr)
      : Transform(RuntimeLocalFrame(dest_frame_name, dest_frame_timestamp),
                  RuntimeLocalFrame(src_frame_name, src_frame_timestamp)) {}
};

template <typename DestFrame, typename SrcFrame>
std::ostream& operator<<(std::ostream& os,
                         Transform<DestFrame, SrcFrame> const& t) {
  os << "G_{" << t.dest_frame_ << "," << t.src_frame_ << "}";
  return os;
}

//------------------------------------------------------------------------------

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
    FrameTimestamp dest_time, FrameTimestamp src_time);

template <typename FrameName>
Transform<LocalFrame<FrameName>, LocalFrame<FrameName>> MakeRelativeTransform(
    FrameTimestamp dest_time, FrameTimestamp src_time) {
  return Transform<LocalFrame<FrameName>, LocalFrame<FrameName>>(
      LocalFrame<FrameName>(dest_time), LocalFrame<FrameName>(src_time));
}

//------------------------------------------------------------------------------

// Implementation of transform composition
namespace detail {

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

}  // namespace detail

//------------------------------------------------------------------------------

template <typename LDestFrame, typename LSrcFrame, typename RDestFrame,
    typename RSrcFrame>
auto operator*(Transform<LDestFrame, LSrcFrame> const& l,
               Transform<RDestFrame, RSrcFrame> const& r) ->
typename detail::TransformComposition<LDestFrame, LSrcFrame, RDestFrame,
                                      RSrcFrame>::ResultType {
  //if (!FramesMatch(l.src_frame_, r.dest_frame_)) {
  if (!frame_equal(l.src_frame_, r.dest_frame_)) {
    std::stringstream ss;
    ss << "Error composing " << l << " and " << r;
    throw ss.str();
  }
  return detail::TransformComposition<LDestFrame, LSrcFrame, RDestFrame,
                                      RSrcFrame>()(l, r);
}

//------------------------------------------------------------------------------

template <typename DestFrame, typename SrcFrame>
auto Invert(Transform<DestFrame, SrcFrame> const& G_dest_src) {
  return Transform<SrcFrame, DestFrame>(G_dest_src.src_frame_,
                                        G_dest_src.dest_frame_);
}

#endif //TRANSFORM_LITE_TRANSFORM_LITE_H
