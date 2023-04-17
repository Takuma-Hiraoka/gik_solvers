#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_H

#include <global_inverse_kinematics_solver/CnoidStateSpace.h>
#include <global_inverse_kinematics_solver/GIKConstraint.h>
#include <global_inverse_kinematics_solver/GIKGoalSpace.h>
#include <global_inverse_kinematics_solver/GIKStateSpace.h>
#include <global_inverse_kinematics_solver/GIKProjectionEvaluator.h>
#include <choreonoid_viewer/choreonoid_viewer.h>

namespace global_inverse_kinematics_solver{
  class GIKParam {
  public:
    int debugLevel = 0;
    double timeout = 10.0;
    double delta = 0.05;
    double range = 0.1;

    cnoid::LinkPtr projectLink = nullptr;
    cnoid::Position projectLocalPose = cnoid::Position::Identity();

    std::shared_ptr<choreonoid_viewer::Viewer> viewer = nullptr;
  };

  // goalsはconstraintsを含まない. 実際のgoalは、constraintsの末尾にgoalsが追加されたものになる
  // pathは、freeJointはx y z qx qy qz qwの順
  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& goals,
                const GIKParam& param = GIKParam(),
                std::shared_ptr<std::vector<std::vector<double> > > path = nullptr);

  void frame2Variables(const std::vector<double>& frame, const std::vector<cnoid::LinkPtr>& variables);
}

#endif
