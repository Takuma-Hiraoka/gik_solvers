#include <global_inverse_kinematics_solver/GIKConstraint.h>

namespace global_inverse_kinematics_solver{
  bool GIKConstraint::project(ompl::base::State *state) const{
    return projectNear(state, state);
  }

  bool GIKConstraint::projectNear(ompl::base::State *state, const ompl::base::State *near) const{

    

    return true;
  }
};
