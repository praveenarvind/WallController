#include "WallController.h"
#include "polytope/robustStabilityPolytope.h"
#include "problemDescriptor/problemDescriptor.h"
#include "wrapper/glpk_wrapper.h"
#include "problemDescriptor/contactSet.h"
#include "PointProjector.h"

#include <mc_solver/QPSolver.h>
#include <mc_rbdyn/Contact.h>


WallController::WallController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt), controllerDt(dt), lowPassPolyCenter_(dt, cutoffPeriodPolyCenter_)
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
mc_rtc::log::info("==== Starting Static Polytope Computation ====");

      // 1. Create ContactSet just once
      auto pd = std::make_shared<ContactSet>(false);

      // 2. Set robot mass and friction sides
      double robot_mass = robot().mass();
      mc_rtc::log::info("Robot mass: {}", robot_mass);
      pd->mass(robot_mass);
      pd->setFrictionSides(6);

      // 3. Add contacts from robot
      size_t n_contacts = contacts().size();
      mc_rtc::log::info("contacts().size() = {}", n_contacts);

      int ptCount = 1;
      int n_contacts_added = 0;
      for(const auto & contact : contacts())
      {
        const auto & sName = contact.r1Surface;
        mc_rtc::log::info("Inspecting contact #{}: surface name = {}", ptCount, sName);

        if(robot().hasSurface(sName))
        {
          auto pose = robot().surfacePose(sName);
          Eigen::Matrix4d homTrans = Eigen::Matrix4d::Identity();
          homTrans.block<3,3>(0,0) = pose.rotation();
          homTrans.block<3,1>(0,3) = pose.translation();

          double friction = 1.0;
          double fmax = 350.0;
          double fmin = 100.0;

          std::string ptName = "contact_" + std::to_string(ptCount);

            if(sName == "RightGripper") // gripper
            {
              friction = 1.0;
              fmax = 150.0;
              fmin = 10.0;
            }
            else if(sName == "LeftGripper")
            {
              friction = 1.0;
              fmax = 2000.0;
              fmin = 10.0;
            }
            else if(sName == "RightFoot")
            {
              friction = 0.7;
              fmax = 350.0;
              fmin = 100.0;
            }
            else if(sName == "LeftFoot")
            {
              friction = 0.7;
              fmax = 350.0;
              fmin = 100.0;
            }


          pd->addContact(ptName, homTrans, friction, fmax, fmin);
          mc_rtc::log::success("Added contact {}: surface={}, pose=[{:.3f}, {:.3f}, {:.3f}]", 
                               ptName, sName, pose.translation().x(), pose.translation().y(), pose.translation().z());
          ++n_contacts_added;
          ++ptCount;
        }
        else
        {
          mc_rtc::log::warning("Robot does NOT have surface: {} (skipping)", sName);
        }
        
      }

      if(n_contacts_added == 0)
      {
        mc_rtc::log::error("No contacts were added to the ContactSet! Static polytope will be empty.");
        return; // Early exit to avoid computing empty polytope
      }
      else
      {
        mc_rtc::log::success("Total contacts added to ContactSet: {}", n_contacts_added);
      }

      // 4. Add CoM accelerations
      mc_rtc::log::info("Adding CoM accelerations...");
      std::vector<Eigen::Vector3d> acc_list = {
        {0.0, 0.0, -9.81},
        {0.6, 0.0, -9.81},
        {0.0, 0.6, -9.81},
        {-0.6, 0.0, -9.81},
        {0.0, -0.6, -9.81}
      };
      for(const auto& a : acc_list)
      {
        pd->addCoMAcc(a);
        mc_rtc::log::info("Added CoM acceleration: [{:.3f}, {:.3f}, {:.3f}]", a.x(), a.y(), a.z());
      }

      // 5. Create and compute the static polytope
      mc_rtc::log::info("Creating StaticStabilityPolytope...");
      double maxError = 1e-4;

      RobustStabilityPolytope polytope(pd, 20, maxError, ::GLPK);

      polytope.initSolver();
      bool ok = polytope.computeProjectionStabilityPolyhedron();
      polytope.endSolver();

      if(!ok){
        mc_rtc::log::error("Comp Stab Fail");
        return;
      }

        Eigen::Vector3d bary = polytope.baryPoint();
  // Optional: smooth it (up to you)
  lowPassPolyCenter_.update(bary);
  bary = lowPassPolyCenter_.eval();

  // 4b) Blend with desiredCoM_
  double blendCoef = 0.5; // tune this as you like
  Eigen::Vector3d filteredCoM = (1.0 - blendCoef) * robot().com() + blendCoef * bary;

  // 4c) Adjust safe Z
  double nominalZ = robot().com().z(); // or any nominal you trust
  filteredCoM.z() = std::max(filteredCoM.z(), nominalZ);

  // 4d) Adjust safe X (optional: use midpoint of feet)
  Eigen::Vector3d leftFoot = robot().frame("LeftFoot").position().translation();
  Eigen::Vector3d rightFoot = robot().frame("RightFoot").position().translation();
  double midX = 0.5 * (leftFoot.x() + rightFoot.x());
  filteredCoM.x() = midX;

  // === 5) Project this filtered CoM ===
  PointProjector projector;
  projector.setPolytope(std::make_shared<RobustStabilityPolytope>(polytope));
  projector.setPoint(filteredCoM);
  projector.project();
  Eigen::Vector3d safeCoM = projector.projectedPoint();

  // === 6) Send to CoMTask ===
  comTask->com(safeCoM);

      // Eigen::Vector3d desiredCoM = desiredCoM;

      // PointProjector projector;
      // projector.setPolytope(std::make_shared<RobustStabilityPolytope>(polytope)); // wrap in shared_ptr
      // projector.setPoint(desiredCoM);
      // projector.project();

      // Eigen::Vector3d safeCoM = projector.projectedPoint();

      // //datastore().call<void, const Eigen::Vector3d &>("RobotStabilizer::setCoMTarget", safeCoM);
      // comTask->com(safeCoM);
      // mc_rtc::log::info("[WallController] Desired CoM projected to safe CoM: {} -> {}", desiredCoM.transpose(), safeCoM.transpose());

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


