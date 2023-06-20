#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKGOALSPACE_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_GIKGOALSPACE_H

#include <ompl_near_projection/ompl_near_projection.h>
#include <ik_constraint2/ik_constraint2.h>
#include <global_inverse_kinematics_solver/GIKStateSpace.h>

namespace global_inverse_kinematics_solver{
  OMPL_CLASS_FORWARD(GIKGoalSpace); // *Ptrを定義. (shared_ptr)

  class GIKGoalSpace : public ompl_near_projection::NearGoalSpace{
  public:
    GIKGoalSpace(const ompl::base::SpaceInformationPtr &si, const ompl::base::StateSpacePtr ambientSpace, std::shared_ptr<UintQueue>& modelQueue, const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints, const std::vector<std::vector<cnoid::LinkPtr> >& variables, const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& goals, const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& nominals, const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& rejections) :
      NearGoalSpace(si),
      modelQueue_(modelQueue),
      variables_(variables),
      goals_(goals)
    {
      for(int i=0;i<variables_.size();i++){
        bodies_.push_back(getBodies(variables_[i]));
      }

      std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > > goalConstraints = constraints;
      for(int i=0;i<goalConstraints.size();i++){
        goalConstraints[i].push_back(goals[i]);
      }
      GIKConstraintPtr goalGIKConstraint = std::make_shared<GIKConstraint>(ambientSpace, modelQueue, goalConstraints, variables, rejections);
      goalGIKConstraint->nominalConstraints() = nominals;
      goalStateSpace_ = std::make_shared<GIKStateSpace>(ambientSpace, goalGIKConstraint);
    }

    virtual bool isSatisfied(const ompl::base::State *st, double *distance) const override;
    virtual double distanceGoal(const ompl::base::State *st) const override;

    bool sampleTo(ompl::base::State *state, const ompl::base::State *source, double* distance = nullptr) const override;

    void setViewer(const std::shared_ptr<choreonoid_viewer::Viewer>& viewer) { goalStateSpace_->getGIKConstraint()->viewer() = viewer;}
    void setDrawLoop(const unsigned int& drawLoop) { goalStateSpace_->getGIKConstraint()->drawLoop() = drawLoop; }
    void setNearMaxError(const double& nearMaxError) { goalStateSpace_->getGIKConstraint()->nearMaxError() = nearMaxError; }
    void setParam(const prioritized_inverse_kinematics_solver2::IKParam& param) { goalStateSpace_->getGIKConstraint()->param() = param; }

 protected:
    GIKStateSpacePtr goalStateSpace_;
    mutable std::shared_ptr<UintQueue> modelQueue_;
    const std::vector<std::vector<cnoid::LinkPtr> > variables_;
    const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > goals_;
    std::vector<std::set<cnoid::BodyPtr> > bodies_;

  };
};

#endif
