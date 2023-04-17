#include <global_inverse_kinematics_solver/GIKProjectionEvaluator.h>

namespace global_inverse_kinematics_solver{

  unsigned int GIKProjectionEvaluator::getDimension(void) const {
    return 3;
  }

  void GIKProjectionEvaluator::defaultCellSizes(void) {
    cellSizes_.resize(3);
    cellSizes_[0] = 0.1;
    cellSizes_[1] = 0.1;
    cellSizes_[2] = 0.1;
  }

  void GIKProjectionEvaluator::project(const ompl::base::State *st, Eigen::Ref<Eigen::VectorXd> projection) const {

    state2Link(space_, st);
    for(std::set<cnoid::BodyPtr>::const_iterator it=bodies_.begin(); it != bodies_.end(); it++){
      (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
      (*it)->calcCenterOfMass();
    }

    cnoid::Vector3 targetp = parentLink_ ? parentLink_->T() * localPos_.translation() : localPos_.translation();
    projection = targetp;
  }

};

