#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKSTATESPACE_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKSTATESPACE_H

#include <ompl_near_projection/NearProjectedStateSpace.h>
#include <global_inverse_kinematics_solver/CnoidStateSpace.h>
#include <global_inverse_kinematics_solver/GIKConstraint.h>
#include <cnoid/Body>
#include <ik_constraint2/ik_constraint2.h>

namespace global_inverse_kinematics_solver{
  OMPL_CLASS_FORWARD(GIKStateSpace); // *Ptrを定義 (shared_ptr)

  class GIKStateSpace : public ompl_near_projection::NearProjectedStateSpace {
  public:
    GIKStateSpace(const ompl::base::StateSpacePtr &ambientSpace, const GIKConstraintPtr &constraint) :
      NearProjectedStateSpace(ambientSpace, constraint),
      gikConstraint_(constraint)
    {
    }

    void setup() override;

    const GIKConstraintPtr getGIKConstraint() const
    {
      return gikConstraint_;
    }

    // 各要素ごとにfromからの変位がmaxDistance以下になる範囲内でtoに近づくstateを返す.
    virtual void elementWiseDistanceLimit(const ompl::base::State *from, const ompl::base::State *to, double maxDistance, ompl::base::State *state) override;

  protected:
    const GIKConstraintPtr gikConstraint_;
  };
};

#endif
