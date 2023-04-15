#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKSTATESPACE_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKSTATESPACE_H

#include <ompl_near_projection/NearProjectedStateSpace.h>
#include <global_inverse_kinematics_solver/CnoidStateSpace.h>
#include <cnoid/Body>
#include <ik_constraint2/ik_constraint2.h>

namespace global_inverse_kinematics_solver{
  OMPL_CLASS_FORWARD(GIKStateSpace); // *Ptrを定義 (shared_ptr)

  class GIKStateSpace : public ompl_near_projection::NearProjectedStateSpace {
  public:
    GIKStateSpace(const ompl::base::StateSpacePtr &ambientSpace, const ompl_near_projection::NearConstraintPtr &constraint) :
      NearProjectedStateSpace(ambientSpace, constraint)
    {
    }

    void setup() override;
  };
};

#endif
