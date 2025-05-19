#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/TransformTask.h>
#include <mc_solver/ContactConstraint.h>






#include "api.h"

struct WallController_DLLAPI WallController : public mc_control::MCController
{
  WallController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  void switch_com_target();

  void touch_hand();

  // In WallController class (WallController.h)

  // void position_right_hand(const Eigen::Vector3d & position);
  // void position_left_hand(const Eigen::Vector3d & position);
  void position_right_hand();
  void position_left_hand();

  void touch_right_hand();
  void touch_left_hand();
  void remove_right_hand();
  void remove_left_hand();
  void monitorContactForce(const std::string & contactName, double muThreshold, double fmax, double fmin);

  private:
    mc_rtc::Configuration config_;
    std::string jointName = "NECK_Y";
    int jointIndex = 0;
    bool goingLeft = true;

    std::shared_ptr<mc_tasks::CoMTask> comTask;
    Eigen::Vector3d comZero;
    bool comDown = true;

    std::shared_ptr<mc_tasks::EndEffectorTask> efTask;


    std::shared_ptr<mc_tasks::TransformTask> rightGripperTask;
    std::shared_ptr<mc_tasks::TransformTask> leftGripperTask;

    
    bool handContactAdded = false;
    double elapsedTime = 0.0; // in seconds


    double controllerDt = 0.0;

    // Eigen::Vector3d rightGripperPos = Eigen::Vector3d::Zero();
    // Eigen::Vector3d leftGripperPos = Eigen::Vector3d::Zero();

    sva::PTransformd rightGripperPose = sva::PTransformd::Identity();
    sva::PTransformd leftGripperPose = sva::PTransformd::Identity();

    sva::ForceVecd wrench = robot().surfaceWrench("RightGripper");

    // Extract force (first 3 components) and torque (last 3)
    Eigen::Vector3d force = wrench.force();   // Newtons
    Eigen::Vector3d torque = wrench.couple(); // Nm

    

    
};