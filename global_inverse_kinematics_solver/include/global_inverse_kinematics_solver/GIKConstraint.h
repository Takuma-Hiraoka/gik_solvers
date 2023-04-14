#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKCONSTRAINT_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKCONSTRAINT_H

#include <ompl_near_projection/ompl_near_projection.h>
#include <ik_constraint2/ik_constraint2.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>

namespace global_inverse_kinematics_solver{
  OMPL_CLASS_FORWARD(GIKConstraint); // GIKConstraintPtrを定義. (shared_ptr)

  class GIKConstraint : public ompl_near_projection::NearConstraint{
  public:
    GIKConstraint(const ompl::base::StateSpacePtr ambientSpace, const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints) :
      NearConstraint(0,0,0),
      ambientSpace_(ambientSpace),
      constraints_(constraints),
      ikConstraints_(constraints)
    {
      param_.we = 1e2;
      param_.maxIteration = 200;
      ikConstraints_.push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >());
    }

    // Constraintクラスでpure virtual関数として定義されているので適当に実態を定義する
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override {return ;}

    virtual bool project(ompl::base::State *state) const override;
    virtual bool projectNear(ompl::base::State *state, const ompl::base::State *near) const override;

    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints() const {return constraints_; }
  protected:
    const ompl::base::StateSpacePtr ambientSpace_;
    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints_;
    mutable std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks_;
    prioritized_inverse_kinematics_solver2::IKParam param_;

    // constraintsの末尾にJointAngleConstraintを加えたもの
    mutable std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > ikConstraints_;
  };
};

#endif
