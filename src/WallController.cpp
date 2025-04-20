#include "WallController.h"

WallController::WallController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt), controllerDt(dt)
{
  jointIndex = robot().jointIndexByName(jointName);
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  
  solver().addTask(postureTask);

  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});


  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0);
  solver().addTask(comTask);

  // Reduce the posture task stiffness
  postureTask->stiffness(1);
  // In the reset function, reset the task to the current CoM

  // efTask = std::make_shared<mc_tasks::EndEffectorTask>("l_wrist", robots(), 0, 5.0, 500.0);
  // solver().addTask(efTask);

  mc_rtc::log::success("WallController init done ");
}

bool WallController::run()
{
  return mc_control::MCController::run();
}

void WallController::switch_com_target()
{
  // comZero is obtained by doing:
  // comZero = comTask->com();
  // in the reset function
  if(comDown) { comTask->com(comZero - Eigen::Vector3d{0, 0, 0.2}); }
  else { comTask->com(comZero); }
  comDown = !comDown;
}

void WallController::touch_hand()
{
  addContact({robot().name(), "ground", "LeftGripper", "AllGround"});
  addContact({robot().name(), "ground", "RightGripper", "AllGround"});
}

// void WallController::position_right_hand(const Eigen::Vector3d & position)
// {
//   if(!rightGripperTask) return;
  
//   sva::PTransformd targetPose(Eigen::Matrix3d::Identity(), position); // just change position
//   rightGripperTask->target(position);
//   solver().addTask(rightGripperTask);
// }

// void WallController::position_left_hand(const Eigen::Vector3d & position)
// {
//   if(!leftGripperTask) return;

//   sva::PTransformd targetPose(Eigen::Matrix3d::Identity(), position);
//   leftGripperTask->target(position);
//   solver().addTask(leftGripperTask);
// }

void WallController::position_right_hand()
{
  if(!rightGripperTask) return;

  rightGripperTask->target(rightGripperPose);
  solver().addTask(rightGripperTask);
}

void WallController::position_left_hand()
{
  if(!leftGripperTask) return;

  leftGripperTask->target(leftGripperPose);
  solver().addTask(leftGripperTask);
}


void WallController::touch_right_hand()
{
  addContact({robot().name(), "ground", "RightGripper", "AllGround"});
}

void WallController::touch_left_hand()
{
  addContact({robot().name(), "ground", "LeftGripper", "AllGround"});
}

void WallController::remove_right_hand()
{
  solver().removeTask(rightGripperTask);
  removeContact({robot().name(), "ground", "RightGripper", "AllGround"});
}

void WallController::remove_left_hand()
{
  solver().removeTask(leftGripperTask);
  removeContact({robot().name(), "ground", "LeftGripper", "AllGround"});
}


void WallController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);

  rightGripperTask = std::make_shared<mc_tasks::TransformTask>("RightGripper", robots(), 0, 5.0, 500.0);
  leftGripperTask = std::make_shared<mc_tasks::TransformTask>("LeftGripper", robots(), 0, 5.0, 500.0);

  // gui()->addElement(
  //   {"WallController"},
  //   mc_rtc::gui::Transform("Right Gripper Pose",
  //     [this]() { return sva::PTransformd{rightGripperPos}; },
  //     [this](const sva::PTransformd & pose) {
  //       rightGripperPos = pose.translation();
  //     })
  // );
  
  // gui()->addElement(
  //   {"WallController"},
  //   mc_rtc::gui::Transform("Left Gripper Pose",
  //     [this]() { return sva::PTransformd{leftGripperPos}; },
  //     [this](const sva::PTransformd & pose) {
  //       leftGripperPos = pose.translation();
  //     })
  // );
  gui()->addElement(
    {"WallController"},
    mc_rtc::gui::Transform("Right Gripper Pose",
      [this]() { return rightGripperPose; },
      [this](const sva::PTransformd & pose) {
        rightGripperPose = pose;
      })
  );
  
  gui()->addElement(
    {"WallController"},
    mc_rtc::gui::Transform("Left Gripper Pose",
      [this]() { return leftGripperPose; },
      [this](const sva::PTransformd & pose) {
        leftGripperPose = pose;
      })
  );
  

  
  
  gui()->addElement(
    {"WallController"},
    mc_rtc::gui::Button("Position Right Hand", [this]() {
      position_right_hand();
      mc_rtc::log::info("[HAND] Right hand positioned: {}", rightGripperPose.translation().transpose());
    })
  );
  
  gui()->addElement(
    {"WallController"},
    mc_rtc::gui::Button("Position Left Hand", [this]() {
      position_left_hand();
      mc_rtc::log::info("[HAND] Left hand contact positioned: {}", leftGripperPose.translation().transpose());
    })
  );

  gui()->addElement(
    {"WallController"},
    mc_rtc::gui::Button("Add Right Hand Contact", [this]() {
      touch_right_hand();
      mc_rtc::log::info("[HAND] Right hand contact added");
    })
  );
  
  gui()->addElement(
    {"WallController"},
    mc_rtc::gui::Button("Add Left Hand Contact", [this]() {
      touch_left_hand();
      mc_rtc::log::info("[HAND] Left hand contact added");
    })
  );
  
  gui()->addElement(
    {"WallController"},
    mc_rtc::gui::Button("Remove Right Hand Contact", [this]() {
      remove_right_hand();
      mc_rtc::log::info("[HAND] Right hand contact removed");
    })
  );
  
  gui()->addElement(
    {"WallController"},
    mc_rtc::gui::Button("Remove Left Hand Contact", [this]() {
      remove_left_hand();
      mc_rtc::log::info("[HAND] Left hand contact removed");
    })
  );
  
  comTask->reset();
  comZero = comTask->com();
  // efTask->reset();
}

CONTROLLER_CONSTRUCTOR("WallController", WallController)


