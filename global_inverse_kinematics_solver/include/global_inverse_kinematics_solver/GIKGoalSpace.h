#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKGOALSPACE_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKGOALSPACE_H

#include <ompl_near_projection/ompl_near_projection.h>
#include <ik_constraint2/ik_constraint2.h>
#include <global_inverse_kinematics_solver/GIKStateSpace.h>

namespace global_inverse_kinematics_solver{
  OMPL_CLASS_FORWARD(GIKGoalSpace); // *Ptrを定義. (shared_ptr)

  class GIKGoalSpace : public ompl_near_projection::NearGoalSpace{
  public:
    GIKGoalSpace(const ompl::base::SpaceInformationPtr &si) : NearGoalSpace(si) {}

    virtual bool isSatisfied(const ompl::base::State *st, double *distance) const override;

    bool sampleTo(ompl::base::State *state, const ompl::base::State *source) const override{
      GIKStateSpacePtr goalSpaceNear = std::static_pointer_cast<GIKStateSpace>(goalSpace_); // goal spaceは、GIKStateSpaceであるという想定. 本当はGoalSpace::setSpace時にcastして保管しておきたいのだが、GoalSpace::setSpaceがvirtual関数として宣言されていないのでできなかった.
      bool ret = goalSpaceNear->getGIKConstraint()->projectNearValidWithNominal(state, source);
      si_->enforceBounds(state);

    }

  };
};

#endif
