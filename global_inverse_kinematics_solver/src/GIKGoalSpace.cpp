#include <global_inverse_kinematics_solver/GIKGoalSpace.h>
#include <global_inverse_kinematics_solver/GIKConstraint.h>

namespace global_inverse_kinematics_solver{

  bool GIKGoalSpace::isSatisfied(const ompl::base::State *st, double *distance) const {
    // TODO goalのみ見る(constraintは見ない)の方が高速だが...
    ompl::base::ConstrainedStateSpacePtr goal_s = std::dynamic_pointer_cast<ompl::base::ConstrainedStateSpace>(goalSpace_);
    if(goal_s == nullptr){
      return NearGoalSpace::isSatisfied(st, distance);
    }

    const GIKConstraintPtr constraint = std::dynamic_pointer_cast<GIKConstraint>(goal_s->getConstraint());
    if(constraint == nullptr){
      return NearGoalSpace::isSatisfied(st, distance);
    }

    return constraint->isSatisfied(st, distance);
  }

  bool GIKGoalSpace::sampleTo(ompl::base::State *state, const ompl::base::State *source) const {
    GIKStateSpacePtr goalSpaceNear = std::static_pointer_cast<GIKStateSpace>(goalSpace_); // goal spaceは、GIKStateSpaceであるという想定. 本当はGoalSpace::setSpace時にcastして保管しておきたいのだが、GoalSpace::setSpaceがvirtual関数として宣言されていないのでできなかった.
    bool ret = goalSpaceNear->getGIKConstraint()->projectNearValidWithNominal(state, source); // goal projection時についでにnominal-poseに近づけることで、tree全体としてnonimalposeに近づけて、逆運動学をときやすくする.
    si_->enforceBounds(state);
    return ret;
  }

};
