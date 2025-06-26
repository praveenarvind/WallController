#include "StabilityCoM.h"
#include <mc_rtc/logging.h>

StabilityCoM::StabilityCoM(double dt)
: lowPassFilter_(dt, 1.0)
{
  pd_ = std::make_shared<ContactSet>(false);
}

void StabilityCoM::buildContactSet(const mc_rbdyn::Robot & robot)
{
  pd_->mass(robot.mass());
  pd_->setFrictionSides(6);

  // This logic is SAME AS YOUR ORIGINAL:
  std::vector<std::string> surfaces = {"RightFoot", "LeftFoot", "RightGripper", "LeftGripper"};

  int ptCount = 1;
  int n_contacts_added = 0;

  for(const auto & sName : surfaces)
  {
    if(!robot.hasSurface(sName))
    {
      mc_rtc::log::warning("Surface {} not found, skipping", sName);
      continue;
    }

    auto pose = robot.surfacePose(sName);
    Eigen::Matrix4d homTrans = Eigen::Matrix4d::Identity();
    homTrans.block<3,3>(0,0) = pose.rotation();
    homTrans.block<3,1>(0,3) = pose.translation();

    double friction = 1.0, fmax = 350.0, fmin = 100.0;

    if(sName == "RightGripper") { fmax = 150.0; fmin = 10.0; }
    else if(sName == "LeftGripper") { fmax = 2000.0; fmin = 10.0; }
    else if(sName == "RightFoot" || sName == "LeftFoot")
    {
      friction = 0.7;
      fmax = 350.0;
      fmin = 100.0;
    }

    pd_->addContact("contact_" + std::to_string(ptCount), homTrans, friction, fmax, fmin);
    ++ptCount;
    ++n_contacts_added;
  }

  if(n_contacts_added == 0)
  {
    mc_rtc::log::error("No contacts added — polytope will be empty");
  }
}


void StabilityCoM::addDefaultCoMAccelerations()
{
  std::vector<Eigen::Vector3d> acc_list = {
    {0.0, 0.0, -9.81},
    {0.6, 0.0, -9.81},
    {0.0, 0.6, -9.81},
    {-0.6, 0.0, -9.81},
    {0.0, -0.6, -9.81}
  };

  for(const auto & a : acc_list)
  {
    pd_->addCoMAcc(a);
  }
}

void StabilityCoM::computeSafeCoM(const mc_rbdyn::Robot & robot, mc_tasks::CoMTask & comTask)
{
  //pd_ = Contact; 20: Directions (How many different ways you check to see how far the robot can safely lean or push)
  //1e-4: Tolerance; GLPK: Solver
  polytope_ = std::make_shared<RobustStabilityPolytope>(pd_, 20, 1e-4, ::GLPK);
  polytope_->initSolver();
  bool ok = polytope_->computeProjectionStabilityPolyhedron();
  polytope_->endSolver();

  if(!ok)
  {
    mc_rtc::log::error("Stability polytope computation failed");
    return;
  }
  
  //Gets the barycenter of the polytope (average center of mass of polytope points)
  Eigen::Vector3d bary = polytope_->baryPoint();
  //Feed the barypoint to a lowpass filter
  lowPassFilter_.update(bary);
  //gets the smoothed lowpaass filter
  bary = lowPassFilter_.eval();

  double blendCoef = 0.5;
  Eigen::Vector3d filteredCoM = (1.0 - blendCoef) * robot.com() + blendCoef * bary;

  filteredCoM.z() = std::max(filteredCoM.z(), robot.com().z());

  Eigen::Vector3d leftFoot = robot.frame("LeftFoot").position().translation();
  Eigen::Vector3d rightFoot = robot.frame("RightFoot").position().translation();
  filteredCoM.x() = 0.5 * (leftFoot.x() + rightFoot.x());


  PointProjector projector;
  //Turn polytope_ into a 3D shape that can be use for projection.
  projector.setPolytope(polytope_);
  //Sets which point you want to project onto the shape
  projector.setPoint(filteredCoM);
  //Finds the closest point on the shape to your point. Saves that result.
  projector.project();
  //Point on the polytope
  Eigen::Vector3d safeCoM = projector.projectedPoint();

  comTask.com(safeCoM);
}
