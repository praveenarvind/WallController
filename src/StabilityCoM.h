#pragma once

#include "problemDescriptor/contactSet.h"
#include "polytope/robustStabilityPolytope.h"
#include "PointProjector.h"

#include <mc_rbdyn/Robot.h>
#include <mc_filter/LowPass.h>
#include <memory>

#include <mc_control/Contact.h>
#include <unordered_set>

#include <mc_tasks/CoMTask.h>

struct StabilityCoM
{
  StabilityCoM(double dt);

  void buildContactSet(const mc_rbdyn::Robot & robot);
  void addDefaultCoMAccelerations();
  void computeSafeCoM(const mc_rbdyn::Robot & robot, mc_tasks::CoMTask & comTask);

private:
  std::shared_ptr<ContactSet> pd_;
  std::shared_ptr<RobustStabilityPolytope> polytope_;
  mc_filter::LowPass<Eigen::Vector3d> lowPassFilter_;
};
