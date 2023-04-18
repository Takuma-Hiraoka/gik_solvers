#include <global_inverse_kinematics_solver/GIKConstraint.h>

namespace global_inverse_kinematics_solver{
  bool GIKConstraint::project(ompl::base::State *state) const{
    return projectNear(state, state);
  }

  bool GIKConstraint::projectNear(ompl::base::State *state, const ompl::base::State *near) const{
    ompl::base::WrapperStateSpace::StateType* wrapperState = static_cast<ompl::base::WrapperStateSpace::StateType*>(state); // stateは, このConstraintをもったWrapperStateSpaceのstateであるはずなので.
    state2Link(ambientSpace_, wrapperState->getState(), variables_); // spaceとstateの空間をそろえる

    {
      // setup nearConstraints
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nearConstraints = ikConstraints_.back();
      nearConstraints.resize(variables_.size());
      for(int i=0;i<variables_.size();i++){
        if(variables_[i]->isRevoluteJoint() || variables_[i]->isPrismaticJoint()){
          std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::dynamic_pointer_cast<ik_constraint2::JointAngleConstraint>(nearConstraints[i]);
          if(constraint == nullptr) {
            nearConstraints[i] = constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
          }
          constraint->joint() = variables_[i];
          constraint->targetq() = variables_[i]->q();
          constraint->precision() = 1e10; // always satisfied

        }else if(variables_[i]->isFreeJoint()) {
          std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::dynamic_pointer_cast<ik_constraint2::PositionConstraint>(nearConstraints[i]);
          if(constraint == nullptr) {
            nearConstraints[i] = constraint = std::make_shared<ik_constraint2::PositionConstraint>();
          }
          constraint->A_link() = variables_[i];
          constraint->B_localpos() = variables_[i]->T();
          constraint->precision() = 1e10; // always satisfied
        }else{
          std::cerr << "[GIKConstraint::projectNear] something is wrong" << std::endl;
        }
      }
    }

    const ompl::base::WrapperStateSpace::StateType* wrapperNear = static_cast<const ompl::base::WrapperStateSpace::StateType*>(near); // stateは, このConstraintをもったWrapperStateSpaceのstateであるはずなので.
    state2Link(ambientSpace_, wrapperNear->getState(), variables_); // spaceとstateの空間をそろえる

    bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables_,
                                                                      ikConstraints_,
                                                                      tasks_,
                                                                      param_);

    link2State(variables_, ambientSpace_, wrapperState->getState()); // spaceとstateの空間をそろえる

    return solved;
  }

  double GIKConstraint::distance (const ompl::base::State *state) const {
    const ompl::base::WrapperStateSpace::StateType* wrapperState = static_cast<const ompl::base::WrapperStateSpace::StateType*>(state); // stateは, このConstraintをもったWrapperStateSpaceのstateであるはずなので.
    state2Link(ambientSpace_, wrapperState->getState(), variables_); // spaceとstateの空間をそろえる

    for(std::set<cnoid::BodyPtr>::const_iterator it=bodies_.begin(); it != bodies_.end(); it++){
      (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
      (*it)->calcCenterOfMass();
    }

    double squaredDistance = 0.0;

    for(size_t i=0;i<constraints_.size();i++){
      for(size_t j=0;j<constraints_[i].size();j++){
        constraints_[i][j]->updateBounds();
        squaredDistance += std::pow(constraints_[i][j]->distance(), 2.0);
      }
    }

    return std::sqrt(squaredDistance);
  }
  bool GIKConstraint::isSatisfied (const ompl::base::State *state) const {
    const ompl::base::WrapperStateSpace::StateType* wrapperState = static_cast<const ompl::base::WrapperStateSpace::StateType*>(state); // stateは, このConstraintをもったWrapperStateSpaceのstateであるはずなので.
    state2Link(ambientSpace_, wrapperState->getState(), variables_); // spaceとstateの空間をそろえる

    for(std::set<cnoid::BodyPtr>::const_iterator it=bodies_.begin(); it != bodies_.end(); it++){
      (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
      (*it)->calcCenterOfMass();
    }

    bool satisfied = true;
    for(size_t i=0;i<constraints_.size();i++){
      for(size_t j=0;j<constraints_[i].size();j++){
        constraints_[i][j]->updateBounds();
        if(!constraints_[i][j]->isSatisfied()) {
          if(viewer_ == nullptr) return false;
          satisfied = false;
        }
      }
    }

    if(viewer_ != nullptr){
      loopCount_++;
      if(loopCount_%drawLoop_==0){
        std::vector<cnoid::SgNodePtr> markers;
        for(int j=0;j<constraints_.size();j++){
          for(int k=0;k<constraints_[j].size(); k++){
            const std::vector<cnoid::SgNodePtr>& marker = constraints_[j][k]->getDrawOnObjects();
            std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
          }
        }
        viewer_->drawOn(markers);
        viewer_->drawObjects(true);
      }
    }

    return satisfied;
  }
  bool GIKConstraint::isSatisfied (const ompl::base::State *state, double *distance) const {
    if(!distance) return isSatisfied(state);

    const ompl::base::WrapperStateSpace::StateType* wrapperState = static_cast<const ompl::base::WrapperStateSpace::StateType*>(state); // stateは, このConstraintをもったWrapperStateSpaceのstateであるはずなので.
    state2Link(ambientSpace_, wrapperState->getState(), variables_); // spaceとstateの空間をそろえる

    for(std::set<cnoid::BodyPtr>::const_iterator it=bodies_.begin(); it != bodies_.end(); it++){
      (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
      (*it)->calcCenterOfMass();
    }

    double squaredDistance = 0.0;
    bool isSatisfied = true;

    for(size_t i=0;i<constraints_.size();i++){
      for(size_t j=0;j<constraints_[i].size();j++){
        constraints_[i][j]->updateBounds();
        if(!constraints_[i][j]->isSatisfied()) isSatisfied = false;
        squaredDistance += std::pow(constraints_[i][j]->distance(), 2.0);
      }
    }

    *distance = std::sqrt(squaredDistance);

    if(viewer_ != nullptr){
      loopCount_++;
      if(loopCount_%drawLoop_==0){
        std::vector<cnoid::SgNodePtr> markers;
        for(int j=0;j<constraints_.size();j++){
          for(int k=0;k<constraints_[j].size(); k++){
            const std::vector<cnoid::SgNodePtr>& marker = constraints_[j][k]->getDrawOnObjects();
            std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
          }
        }
        viewer_->drawOn(markers);
        viewer_->drawObjects(true);
      }
    }

    return isSatisfied;

  }

};
