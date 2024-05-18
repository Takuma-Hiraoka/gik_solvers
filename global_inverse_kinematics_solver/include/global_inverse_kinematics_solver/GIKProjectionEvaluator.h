#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKPROJECTIONEVALUATOR_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKPROJECTIONEVALUATOR_H

#include <global_inverse_kinematics_solver/CnoidStateSpace.h>
#include <ompl/base/ProjectionEvaluator.h>

namespace global_inverse_kinematics_solver{
  OMPL_CLASS_FORWARD(GIKProjectionEvaluator); // *Ptrを定義. (shared_ptr)

  class GIKProjectionEvaluator : public ompl::base::ProjectionEvaluator{
  public:
    GIKProjectionEvaluator(const ompl::base::StateSpacePtr &space, std::shared_ptr<UintQueue>& modelQueue, const std::vector<std::vector<cnoid::LinkPtr> >& variables) :
      ompl::base::ProjectionEvaluator(space),
      modelQueue_(modelQueue),
      variables_(variables)
    {
      for(int i=0;i<variables_.size();i++){
        bodies_.push_back(getBodies(variables_[i]));
      }
    }
    virtual unsigned int getDimension(void) const override;
    virtual void defaultCellSizes(void) override;
    virtual void project(const ompl::base::State *st, Eigen::Ref<Eigen::VectorXd> projection) const override;

    std::vector<cnoid::LinkPtr>& parentLink() { return parentLink_; }
    const std::vector<cnoid::LinkPtr>& parentLink() const { return parentLink_; }
    cnoid::Isometry3& localPos() { return localPos_; }
    const cnoid::Isometry3& localPos() const { return localPos_; }

  protected:
    // model queueで管理.
    mutable std::shared_ptr<UintQueue> modelQueue_;
    const std::vector<std::vector<cnoid::LinkPtr> > variables_;
    std::vector<std::set<cnoid::BodyPtr> > bodies_;
    std::vector<cnoid::LinkPtr> parentLink_;

    cnoid::Isometry3 localPos_;
  };


};

#endif
