#include <iostream>

#include <boost/type_index.hpp>

#include "transformlite/transform_lite.h"

//------------------------------------------------------------------------------

// Define some compile time frames.
DEFINE_FRAME_NAME(WorldFrame, world);
DEFINE_FRAME_NAME(RobotFrame, robot);

//using WorldFrame = FrameName<struct WorldFrameTag>;
//template <>
//std::string const WorldFrame::name{"world"};
//
//using RobotFrame = FrameName<struct RobotFrameTag>;
//template <>
//std::string const RobotFrame::name{"robot"};

using RobotCalibrationFrame = CalibrationFrame<RobotFrame>;
using RobotLocalFrame = LocalFrame<RobotFrame>;

DEFINE_FRAME_NAME(CameraFrame, camera);
//using CameraFrame = FrameName<struct CameraFrameTag>;
//template <>
//std::string const CameraFrame::name{"camera"};
using CameraCalibrationFrame = CalibrationFrame<CameraFrame>;
using CameraLocalFrame = LocalFrame<CameraFrame>;

DEFINE_FRAME_NAME(LaserFrame, laser);
//using LaserFrame = FrameName<struct LaserFrameTag>;
//template <>
//std::string const LaserFrame::name{"laser"};

// Helper functions for demo output
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

//------------------------------------------------------------------------------

template <typename T>
struct TD;

template <bool v>
void print_constexpr() {
  std::cout << std::boolalpha << v << std::noboolalpha << '\n';
}

int main(int argc, const char* argv[]) {
  print_constexpr<frame_name_equal(RobotFrame(), RobotFrame())>();
  //print_constexpr<FramesMatch(RobotFrame(), RobotFrame())>();


  //----------------------------------------------------------------------------

  auto G_robot_camera_calibration =
      MakeCalibrationTransform<RobotFrame, CameraFrame>();
  auto G_laser_robot_calibration =
      MakeCalibrationTransform<LaserFrame, RobotFrame>();
  // ERROR enable_if removes this constructor if using runtime frames
//     MakeCalibrationTransform<RuntimeCalibrationFrame,
//     RobotCalibrationFrame>();

  // Compose two calibration transforms
  PrintTransformComposition(G_laser_robot_calibration,
                            G_robot_camera_calibration);

  // Lets make a transform chain. Lets do the first transform with an easy
  // helper function. We want both frames to be LocalFrames (have
  // timestamps)
  // and in this case we know that it is a common Dest/Src frame _name_
  // (RobotFrame). Therefore the only interesting information is the
  // timestamps.
  // The helper function constructs the frames for us and then builds the
  // transform.
  auto G_robot_t1_robot_t2 = MakeRelativeTransform<RobotFrame>(1_ft, 2_ft);
  // Under the hood this is essentially calling the constructor with the
  // frame
  // type LocalFrame<RobotFrame>
  auto G_robot_t2_robot_t3 =
      Transform<RobotLocalFrame, RobotLocalFrame>(2_ft, 3_ft);
  // Expanded and removing the all the typedefs and we get this monstrosity.
  // This is why auto is awsome - otherwise this is what you'd be declaring
  // the
  // type as.
  auto G_robot_t3_robot_t4 = Transform<LocalFrame<FrameName<RobotFrameTag>>,
                                       LocalFrame<FrameName<RobotFrameTag>>>(
      LocalFrame<FrameName<RobotFrameTag>>(3_ft),
      LocalFrame<FrameName<RobotFrameTag>>(4_ft));
  // Compose three local transforms
  PrintTransformComposition(G_robot_t1_robot_t2, G_robot_t2_robot_t3,
                            G_robot_t3_robot_t4);

  // Project a relative transform into a different frame using a
  // calibration.
  PrintTransformComposition(Invert(G_robot_camera_calibration),
                            G_robot_t1_robot_t2, G_robot_camera_calibration);

  // Get new calibration frame at runtime.
  // Note, we are making a runtime robot frame. We could use the compile
  // time
  // one but this shows that mixing runtime and compile time frames works
  // fine.
  // We have specified the type here to be expicit about it being a
  // different
  // type for the same frame string, however, we can remove the heavy syntax
  // using the _frame suffix, shown for the second parameter.
  auto G_robot_ins_calibration =
      MakeCalibrationTransform(RuntimeFrameName{"robot"}, "ins"_frame);
  // Compose a transform with runtime frames and one with compile time tag.
  PrintTransformComposition(G_laser_robot_calibration, G_robot_ins_calibration);

  // Composition of bad compile time frames.
  //G_robot_camera_calibration * G_laser_robot_calibration;  // ERROR

  // Compisition of bad frame names, checked at runtime.
  try {
    PrintTransformComposition(G_robot_ins_calibration,
                              G_laser_robot_calibration);
  } catch (std::string& ex) {
    std::cout << ex << std::endl;
  }

  // Composition of bad frame timestamps, matching names.
  try {
    PrintTransformComposition(G_robot_t2_robot_t3, G_robot_t1_robot_t2);
  } catch (std::string& ex) {
    std::cout << ex << std::endl;
  }

  // The robot in the world frame.
  auto G_world_ins_t3 = MakeTransform(ExternalFrame<WorldFrame>(),
                                      RuntimeLocalFrame("ins"_frame, 3_ft));
  PrintTransformComposition(G_world_ins_t3, Invert(G_robot_ins_calibration));

  try {
    // Show the use of enable_if constructor for building runtime transform.
    using LeafTransform = Transform<RuntimeLocalFrame, RuntimeLocalFrame>;
    auto G_robot_t1_robot_t3 =
        LeafTransform("robot"_frame, 1_ft, "robot"_frame, 3_ft);
    // Show we can compose runtime and compiletime frames.
    PrintTransformComposition(G_robot_t1_robot_t3, G_robot_t3_robot_t4);

    // Now show this will error at runtime.
    PrintTransformComposition(G_robot_t1_robot_t3, G_laser_robot_calibration);
  } catch (std::string& ex) {
    std::cout << ex << std::endl;
  }

  // ERROR Cannot create a transform where one frame is Calibration and the
  // other is not.
//        Transform<RuntimeCalibrationFrame, RuntimeLocalFrame>(
//            RuntimeCalibrationFrame("robot"_frame),
//            RuntimeLocalFrame("laser"_frame,1_ft));

  // Cannot apply calibration transform to a world frame.
  // NOTE A frame name does not encode the frame type to which it will be
  // applied (as this can be any of the three types). To prevent this at
  // compile
  // time would require separating frame names into external and
  // non-external
  // types. Things were already pretty ugl. I tried inheritance. That went
  // badly. I gave up. Whilst you can create this transform, as soon as you
  // try
  // and compose it with anything which isn't a calibration transform it
  // will go
  // bang.
//        auto G_robot_world_calibration =
//            MakeCalibrationTransform<RobotFrame, WorldFrame>();
//    G_robot_world_calibration* G_world_ins_t3;  // ERROR

  // Disabled through enable_if
  //      auto runtime_local_frame = RuntimeLocalFrame{3_ft};
  //      auto runtime_external_frame = RuntimeExternalFrame{};
  //    RuntimeExternalFrame runtime_external_frame2;
  //    auto runtime_calibration_frame = RuntimeCalibrationFrame{};

  return 0;
}
