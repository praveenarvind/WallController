#include "WallController.h"
#include <mc_solver/QPSolver.h>
#include <mc_rbdyn/Contact.h>



WallController::WallController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt), controllerDt(dt)
{
  jointIndex = robot().jointIndexByName(jointName);
  config_.load(config);
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(dynamicsConstraint);
  
  solver().addTask(postureTask);
  
  addContact({robot().name(), "ground", "RightFoot", "AllGround"});
  addContact({robot().name(), "ground", "LeftFoot", "AllGround"});

  comTask = std::make_shared<mc_tasks::CoMTask>(robots(), 0, 10.0, 1000.0);
  solver().addTask(comTask);

  postureTask->stiffness(1);


  mc_rtc::log::success("WallController init done ");
}

bool WallController::run()
{
  bool success = mc_control::MCController::run();

  for(const auto & contact : contacts())
    {
      const auto & sName = contact.r1Surface;
      // Check if the robot owns this surface
      if(robot().hasSurface(sName))
      {

        if(sName == "LeftFoot"){
          monitorContactForce("LeftFoot", 0.5, 1000.0, 100.0);
        }
        if(sName == "RightFoot"){
          monitorContactForce("RightFoot", 0.5, 1000.0, 100.0);

        }
        if(sName == "LeftGripper"){
          monitorContactForce("LeftGripper", 0.5, 2000.0, 10.0);

        }
        if(sName == "RightGripper"){
          monitorContactForce("RightGripper", 0.5, 2000.0, 10.0);

        }
        
      }
    }
  
  

  return success;
}

void WallController::monitorContactForce(const std::string & contactName, double muThreshold, double fmax, double fmin)
{
  sva::ForceVecd wrenchAtContact = sva::ForceVecd::Zero();

  for(const auto & contact : solver().contacts())
  {
    if(contact.r1Surface()->name() == contactName || contact.r2Surface()->name() == contactName)
    {
      wrenchAtContact = solver().desiredContactForce(contact);
      break;
    }
    // else{
    //   return;
    // }
  }

  const Eigen::Vector3d & force = wrenchAtContact.force();
  mc_rtc::log::info("Force on {}: fx = {}, fy = {}, fz = {}, f = {}", 
                    contactName, force.x(), force.y(), force.z(), force.norm());

  // Remove contact if force magnitude outside bounds
  if(force.norm() > fmax || force.norm() < fmin)
  {
    mc_rtc::log::warning("[{}] Removing contact due to force magnitude {:.2f}", contactName, force.norm());
    removeContact({robot().name(), "ground", contactName, "AllGround"});
    return;
  }

  // Remove contact if force direction is pulling
  if(force.z() < 0)
  {
    mc_rtc::log::warning("[{}] Removing contact due to negative Z force: {}", contactName, force.z());
    removeContact({robot().name(), "ground", contactName, "AllGround"});
    return;
  }

  // Compute effective friction
  double mu = std::sqrt(force.x() * force.x() + force.y() * force.y()) / std::abs(force.z());

  if(mu > muThreshold)
  {
    mc_rtc::log::warning("[{}] Removing contact due to excessive μ = {:.2f} (threshold = {:.2f})", contactName, mu, muThreshold);
    removeContact({robot().name(), "ground", contactName, "AllGround"});
  }
}

void WallController::switch_com_target()
{
  if(comDown) { comTask->com(comZero - Eigen::Vector3d{0, 0, 0.2}); }
  else { comTask->com(comZero); }
  comDown = !comDown;
}

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

  gui()->addElement(
    {"WallController"},
    mc_rtc::gui::Button("Print Information", [this]() {
      // ───── Robot-Level Info ─────
    mc_rtc::log::info("Robot name: {}", robot().name());
    mc_rtc::log::info("Robot mass: {} kg", robot().mass());

// ───── Active Contacts Info ─────
    for(const auto & contact : contacts())
    {
      const auto & sName = contact.r1Surface;
      // Check if the robot owns this surface
      if(robot().hasSurface(sName))
      {
        auto pose = robot().surfacePose(sName);
        Eigen::Vector3d pos = pose.translation();
        Eigen::Matrix3d rot = pose.rotation();
        Eigen::Vector3d euler = rot.eulerAngles(0, 1, 2); // XYZ Euler
      
        mc_rtc::log::info("Contact Surface: {}", sName);
        mc_rtc::log::info("  ↳ Position (x, y, z): {}", pos.transpose());
        mc_rtc::log::info("  ↳ Orientation (Euler XYZ, rad): {}", euler.transpose());
      }
    }})

  
  );

  gui()->addElement(
  {"WallController"},
  mc_rtc::gui::Button("Log LeftFoot QP Force", [this]()
  {
    auto desiredContactName = "LeftFoot";
   sva::ForceVecd wrenchAtContact;
   
   for(const auto & contact : solver().contacts())
   {
    if (contact.r1Surface()->name() == desiredContactName || contact.r2Surface()->name() == desiredContactName )
      {
        wrenchAtContact = solver().desiredContactForce(contact);
      }

    }
    
    mc_rtc::log::info("The wrench at contact {} is {}", desiredContactName, wrenchAtContact);
    mc_rtc::log::info("Torque components: tx = {}, ty = {}, tz = {}", 
                  wrenchAtContact.couple().x(), 
                  wrenchAtContact.couple().y(), 
                  wrenchAtContact.couple().z());
    mc_rtc::log::info("Force components: fx = {}, fy = {}, fz = {}, f = {}",
                  wrenchAtContact.force().x(),
                  wrenchAtContact.force().y(),
                  wrenchAtContact.force().z(),
                  wrenchAtContact.force().norm());

                  

   })
   
  );

  comTask->reset();
  comZero = comTask->com();
  // efTask->reset();
  // Imagine a robot foot on the ground:

  //   μ: how slippery the floor is (high μ = good grip)

  //   fmin: the robot must press down at least a little to keep the foot planted

  //   fmax: the leg can only push so hard before breaking or slipping
}

CONTROLLER_CONSTRUCTOR("WallController", WallController)


