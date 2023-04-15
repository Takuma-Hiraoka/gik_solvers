#include <global_inverse_kinematics_solver/GIKStateSpace.h>
#include <global_inverse_kinematics_solver/GIKConstraint.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace global_inverse_kinematics_solver{

  ompl_near_projection::NearProjectedStateSpacePtr createStateSpace(const std::vector<cnoid::LinkPtr>& variables, std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints){
    ompl::base::StateSpacePtr ambientSpace = createAmbientSpace(variables);
    GIKConstraintPtr gikConstraint = std::make_shared<GIKConstraint>(ambientSpace, constraints);
    return std::make_shared<ompl_near_projection::NearProjectedStateSpace>(ambientSpace, gikConstraint);
  }
};
