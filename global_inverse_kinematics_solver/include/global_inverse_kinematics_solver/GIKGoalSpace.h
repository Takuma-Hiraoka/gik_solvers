#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKGOALSPACE_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKGOALSPACE_H

#include <ompl_near_projection/ompl_near_projection.h>
#include <ik_constraint2/ik_constraint2.h>

namespace global_inverse_kinematics_solver{

  class GIKGoalSpace : public ompl_near_projection::NearGoalSpace{
  public:
    virtual bool isSatisfied(const ompl::base::State *st, double *distance) const override;
  };
};

#endif
