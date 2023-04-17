#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKPROJECTIONEVALUATOR_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKPROJECTIONEVALUATOR_H

#include <global_inverse_kinematics_solver/CnoidStateSpace.h>
#include <ompl/base/ProjectionEvaluator.h>

namespace global_inverse_kinematics_solver{
  OMPL_CLASS_FORWARD(GIKProjectionEvaluator); // *Ptrを定義. (shared_ptr)

  class GIKProjectionEvaluator : public ompl::base::ProjectionEvaluator{
  public:
    GIKProjectionEvaluator(const ompl::base::StateSpacePtr &space) :
      ompl::base::ProjectionEvaluator(space),
      variables_(getLinks(space)),
      bodies_(getBodies(variables_))
    {
    }
    virtual unsigned int getDimension(void) const override;
    virtual void defaultCellSizes(void) override;
    virtual void project(const ompl::base::State *st, Eigen::Ref<Eigen::VectorXd> projection) const override;

    cnoid::LinkPtr& parentLink() { return parentLink_; }
    const cnoid::LinkPtr& parentLink() const { return parentLink_; }
    cnoid::Position& localPos() { return localPos_; }
    const cnoid::Position& localPos() const { return localPos_; }

  protected:
    const std::vector<cnoid::LinkPtr> variables_;
    const std::set<cnoid::BodyPtr> bodies_;

    cnoid::LinkPtr parentLink_;
    cnoid::Position localPos_;
  };


};

#endif
