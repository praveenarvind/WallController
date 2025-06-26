#include "WallController.h"
#include "polytope/robustStabilityPolytope.h"
#include "problemDescriptor/problemDescriptor.h"
#include "wrapper/glpk_wrapper.h"
#include "problemDescriptor/contactSet.h"
#include "PointProjector.h"

#include <mc_solver/QPSolver.h>
#include <mc_rbdyn/Contact.h>


WallController::WallController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt), controllerDt(dt), lowPassPolyCenter_(dt, cutoffPeriodPolyCenter_), stabilityCoM_(dt)
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

  if(checkContactFlag)
{
  // std::vector<std::string> contactNames;

  // for(const auto & contact : contacts())
  // {
  //   const auto & sName = contact.r1Surface;
  //   if(robot().hasSurface(sName))
  //   {
  //     contactNames.push_back(sName);
  //   }
  // }

  // for(const auto & name : contactNames)
  // {
  //   if(name == "RightGripper") monitorContactForce("RightGripper", 1, 150.0, 10.0);
  //   if(name == "LeftGripper")  monitorContactForce("LeftGripper",  1, 2000.0, 10.0);
  //   if(name == "RightFoot")    monitorContactForce("RightFoot",    0.7, 350.0, 100.0);
  //   if(name == "LeftFoot")     monitorContactForce("LeftFoot",     0.7, 400.0, 100.0);
  // }
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
    //removeContact({robot().name(), "ground", contactName, "AllGround"});
    //return;
  }

  // Remove contact if force direction is pulling
  if((force.z() < 0 && (contactName == "RightFoot" || contactName == "LeftFoot")) ||
   (force.z() > 0 && (contactName == "RightGripper" || contactName == "LeftGripper")))
  {
    mc_rtc::log::warning("[{}] Removing contact due to negative Z force: {}", contactName, force.z());
    //removeContact({robot().name(), "ground", contactName, "AllGround"});
    //return;
  }

  // Compute effective friction
  double mu = std::sqrt(force.x() * force.x() + force.y() * force.y()) / std::abs(force.z());
  mc_rtc::log::info("[{}] μ = {:.2f} (threshold = {:.2f})", contactName, mu, muThreshold);
    

  if(mu > muThreshold)
  {
    mc_rtc::log::warning("[{}] Removing contact due to excessive μ = {:.2f} (threshold = {:.2f})", contactName, mu, muThreshold);
    //removeContact({robot().name(), "ground", contactName, "AllGround"});
    //return;
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

  // Set position
  Eigen::Vector3d position(0.605258, -0.261046, 1.13794);

  // Set quaternion and convert to rotation matrix
  Eigen::Quaterniond quat(-0.0051878911697347905,
                           0.705439037763330,
                          0.01813315373065657,
                          0.7094415531541714);
  quat.normalize(); // always normalize for safety

  // Build the full pose
  sva::PTransformd targetPose(quat.toRotationMatrix(), position);

  // Apply the pose
  rightGripperTask->target(targetPose);
  solver().addTask(rightGripperTask);

  mc_rtc::log::info("[HAND] Right hand positioned to predefined pose.");


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
  removeContact({robot().name(), "ground", "RightGripper", "AllGround"});
}

void WallController::remove_left_hand()
{
  solver().removeTask(leftGripperTask);
  removeContact({robot().name(), "ground", "LeftGripper", "AllGround"});
}

void WallController::updateCoM(){
    mc_rtc::log::info("=== Updating StabilityCoM ===");
    stabilityCoM_.buildContactSet(robot());
    stabilityCoM_.addDefaultCoMAccelerations();
    stabilityCoM_.computeSafeCoM(robot(), *comTask);
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
      solver().removeTask(rightGripperTask);
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
      updateCoM();
      mc_rtc::log::info("com updte");
    })

  
  );

  gui()->addElement(
  {"WallController"},
  mc_rtc::gui::Button("Log LeftFoot QP Force", [this]()
  {
    auto desiredContactName = "RightGripper";
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

  gui()->addElement(
  {"WallController"},
  mc_rtc::gui::Button("Run Contact Force Check", [this]()
  {
    checkContactFlag = true;
    mc_rtc::log::success("Started contact force monitoring.");
    std::vector<std::string> contactNames;

  for(const auto & contact : contacts())
  {
    const auto & sName = contact.r1Surface;
    if(robot().hasSurface(sName))
    {
      contactNames.push_back(sName);
    }
  }

  for(const auto & name : contactNames)
  {
    if(name == "RightGripper") monitorContactForce("RightGripper", 1, 150.0, 10.0);
    if(name == "LeftGripper")  monitorContactForce("LeftGripper",  1, 2000.0, 10.0);
    if(name == "RightFoot")    monitorContactForce("RightFoot",    0.7, 350.0, 100.0);
    if(name == "LeftFoot")     monitorContactForce("LeftFoot",     0.7, 400.0, 100.0);
  }
  })
);

gui()->addElement(
  {"WallController"},
  mc_rtc::gui::Button("Stop Contact Force Check", [this]()
  {
    checkContactFlag = false;
    mc_rtc::log::success("Stop contact force monitoring.");
  })
);





gui()->addElement(
  {"WallController"},
  mc_rtc::gui::Button("Print Stability Polytope", [this]() {
   updateCoM();
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


