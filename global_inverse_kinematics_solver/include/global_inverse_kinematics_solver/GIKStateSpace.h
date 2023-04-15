#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKSTATESPACE_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKSTATESPACE_H

#include <ompl_near_projection/NearProjectedStateSpace.h>
#include <global_inverse_kinematics_solver/CnoidStateSpace.h>
#include <cnoid/Body>
#include <ik_constraint2/ik_constraint2.h>

namespace global_inverse_kinematics_solver{

  ompl_near_projection::NearProjectedStateSpacePtr createStateSpace(const std::vector<cnoid::LinkPtr>& variables, std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints);
};

#endif
