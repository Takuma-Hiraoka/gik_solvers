#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKSTATESPACE_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKSTATESPACE_H

#include <ompl_near_projection/NearProjectedStateSpace.h>
#include <global_inverse_kinematics_solver/CnoidStateSpace.h>
#include <cnoid/Body>
#include <ik_constraint2/ik_constraint2.h>

namespace global_inverse_kinematics_solver{
  OMPL_CLASS_FORWARD(GIKStateSpace); // GIKStateSpacePtrを定義 (shared_ptr)

  class GIKStateSpace : public ompl_near_projection::NearProjectedStateSpace {
  public:
    GIKStateSpace(const ompl::base::StateSpacePtr &ambientSpace, const ompl_near_projection::NearConstraintPtr &constraint, const std::vector<cnoid::LinkPtr>& variables) :
      NearProjectedStateSpace(ambientSpace, constraint),
      variables_(variables)
    {
    }

    const std::vector<cnoid::LinkPtr>& variables() const {return variables_;}
    static void state2Variables (const ompl::base::State *state, const ompl::base::StateSpacePtr space, const std::vector<cnoid::LinkPtr>& variables);

  protected:
    static void state2VariablesImpl(const ompl::base::State *state, const ompl::base::StateSpacePtr space, const std::vector<cnoid::LinkPtr>& variables, int& variablesIndex);
    const std::vector<cnoid::LinkPtr>& variables_;
  };

  GIKStateSpacePtr createGIKStateSpace(const std::vector<cnoid::LinkPtr>& variables, std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints);
};

#endif
