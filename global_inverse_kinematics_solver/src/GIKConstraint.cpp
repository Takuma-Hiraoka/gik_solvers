#include <global_inverse_kinematics_solver/GIKConstraint.h>

namespace global_inverse_kinematics_solver{
  bool GIKConstraint::project(ompl::base::State *state) const{
    std::cerr << "GIKConstraint::project" << std::endl;
    return projectNearValid(state, state);
  }

  bool GIKConstraint::projectNearValid(ompl::base::State *state, const ompl::base::State *near) const{
    const unsigned int m = modelQueue_->pop();

    ompl::base::WrapperStateSpace::StateType* wrapperState = static_cast<ompl::base::WrapperStateSpace::StateType*>(state); // stateは, このConstraintをもったWrapperStateSpaceのstateであるはずなので.
    state2Link(ambientSpace_, wrapperState->getState(), variables_[m]); // spaceとstateの空間をそろえる

    {
      // setup nearConstraints
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nearConstraints = ikConstraints_[m].back();
      nearConstraints.resize(variables_[m].size());
      for(int i=0;i<variables_[m].size();i++){
        if(variables_[m][i]->isRevoluteJoint() || variables_[m][i]->isPrismaticJoint()){
          std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::dynamic_pointer_cast<ik_constraint2::JointAngleConstraint>(nearConstraints[i]);
          if(constraint == nullptr) {
            nearConstraints[i] = constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
          }
          constraint->joint() = variables_[m][i];
          constraint->targetq() = variables_[m][i]->q();
          constraint->precision() = 1e10; // always satisfied

        }else if(variables_[m][i]->isFreeJoint()) {
          std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::dynamic_pointer_cast<ik_constraint2::PositionConstraint>(nearConstraints[i]);
          if(constraint == nullptr) {
            nearConstraints[i] = constraint = std::make_shared<ik_constraint2::PositionConstraint>();
          }
          constraint->A_link() = variables_[m][i];
          constraint->B_localpos() = variables_[m][i]->T();
          constraint->precision() = 1e10; // always satisfied
        }else{
          std::cerr << "[GIKConstraint::projectNear] something is wrong" << std::endl;
        }
      }
    }

    const ompl::base::WrapperStateSpace::StateType* wrapperNear = static_cast<const ompl::base::WrapperStateSpace::StateType*>(near); // stateは, このConstraintをもったWrapperStateSpaceのstateであるはずなので.
    state2Link(ambientSpace_, wrapperNear->getState(), variables_[m]); // spaceとstateの空間をそろえる

    bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables_[m],
                                                                      ikConstraints_[m],
                                                                      tasks_[m],
                                                                      param_);

    link2State(variables_[m], ambientSpace_, wrapperState->getState()); // spaceとstateの空間をそろえる

    modelQueue_->push(m);

    return solved;
  }

  bool GIKConstraint::projectNearValidWithNominal(ompl::base::State *state, const ompl::base::State *near) const{
    const unsigned int m = modelQueue_->pop();

    ompl::base::WrapperStateSpace::StateType* wrapperState = static_cast<ompl::base::WrapperStateSpace::StateType*>(state); // stateは, このConstraintをもったWrapperStateSpaceのstateであるはずなので.
    state2Link(ambientSpace_, wrapperState->getState(), variables_[m]); // spaceとstateの空間をそろえる

    {
      // setup nearConstraints
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominalConstraints = ikConstraints_[m].back();
      nominalConstraints.resize(nominalConstraints_[m].size());
      for(int i=0;i<nominalConstraints_[m].size(); i++){
        nominalConstraints[i] = nominalConstraints_[m][i];
      }
    }

    const ompl::base::WrapperStateSpace::StateType* wrapperNear = static_cast<const ompl::base::WrapperStateSpace::StateType*>(near); // stateは, このConstraintをもったWrapperStateSpaceのstateであるはずなので.
    state2Link(ambientSpace_, wrapperNear->getState(), variables_[m]); // spaceとstateの空間をそろえる

    bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables_[m],
                                                                      ikConstraints_[m],
                                                                      tasks_[m],
                                                                      param_);

    link2State(variables_[m], ambientSpace_, wrapperState->getState()); // spaceとstateの空間をそろえる

    modelQueue_->push(m);

    return solved;
  }

  double GIKConstraint::distance (const ompl::base::State *state) const {
    const unsigned int m = modelQueue_->pop();
    const ompl::base::WrapperStateSpace::StateType* wrapperState = static_cast<const ompl::base::WrapperStateSpace::StateType*>(state); // stateは, このConstraintをもったWrapperStateSpaceのstateであるはずなので.
    state2Link(ambientSpace_, wrapperState->getState(), variables_[m]); // spaceとstateの空間をそろえる

    for(std::set<cnoid::BodyPtr>::const_iterator it=bodies_[m].begin(); it != bodies_[m].end(); it++){
      (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
      (*it)->calcCenterOfMass();
    }

    double squaredDistance = 0.0;

    for(size_t i=0;i<constraints_[m].size();i++){
      for(size_t j=0;j<constraints_[m][i].size();j++){
        constraints_[m][i][j]->updateBounds();
        squaredDistance += std::pow(constraints_[m][i][j]->distance(), 2.0);
      }
    }
    modelQueue_->push(m);
    return std::sqrt(squaredDistance);
  }
  bool GIKConstraint::isSatisfied (const ompl::base::State *state) const {
    const unsigned int m = modelQueue_->pop();
    const ompl::base::WrapperStateSpace::StateType* wrapperState = static_cast<const ompl::base::WrapperStateSpace::StateType*>(state); // stateは, このConstraintをもったWrapperStateSpaceのstateであるはずなので.
    state2Link(ambientSpace_, wrapperState->getState(), variables_[m]); // spaceとstateの空間をそろえる

    for(std::set<cnoid::BodyPtr>::const_iterator it=bodies_[m].begin(); it != bodies_[m].end(); it++){
      (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
      (*it)->calcCenterOfMass();
    }

    bool satisfied = true;
    for(size_t i=0;i<constraints_[m].size();i++){
      for(size_t j=0;j<constraints_[m][i].size();j++){
        constraints_[m][i][j]->updateBounds();
        if(!constraints_[m][i][j]->isSatisfied()) {
          if(viewer_ == nullptr || m!=0) {
            modelQueue_->push(m);
            return false;
          }
          satisfied = false;
        }
      }
    }

    if(viewer_ != nullptr && m==0){
      loopCount_++;
      if(loopCount_%drawLoop_==0){
        std::vector<cnoid::SgNodePtr> markers;
        for(int j=0;j<constraints_[m].size();j++){
          for(int k=0;k<constraints_[m][j].size(); k++){
            const std::vector<cnoid::SgNodePtr>& marker = constraints_[m][j][k]->getDrawOnObjects();
            std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
          }
        }
        viewer_->drawOn(markers);
        viewer_->drawObjects(true);
      }
    }
    modelQueue_->push(m);
    return satisfied;
  }
  bool GIKConstraint::isSatisfied (const ompl::base::State *state, double *distance) const {
    if(!distance) return isSatisfied(state);
    const unsigned int m = modelQueue_->pop();

    const ompl::base::WrapperStateSpace::StateType* wrapperState = static_cast<const ompl::base::WrapperStateSpace::StateType*>(state); // stateは, このConstraintをもったWrapperStateSpaceのstateであるはずなので.
    state2Link(ambientSpace_, wrapperState->getState(), variables_[m]); // spaceとstateの空間をそろえる

    for(std::set<cnoid::BodyPtr>::const_iterator it=bodies_[m].begin(); it != bodies_[m].end(); it++){
      (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
      (*it)->calcCenterOfMass();
    }

    double squaredDistance = 0.0;
    bool isSatisfied = true;

    for(size_t i=0;i<constraints_[m].size();i++){
      for(size_t j=0;j<constraints_[m][i].size();j++){
        constraints_[m][i][j]->updateBounds();
        if(!constraints_[m][i][j]->isSatisfied()) isSatisfied = false;
        squaredDistance += std::pow(constraints_[m][i][j]->distance(), 2.0);
      }
    }

    *distance = std::sqrt(squaredDistance);

    if(viewer_ != nullptr && m==0){
      loopCount_++;
      if(loopCount_%drawLoop_==0){
        std::vector<cnoid::SgNodePtr> markers;
        for(int j=0;j<constraints_[m].size();j++){
          for(int k=0;k<constraints_[m][j].size(); k++){
            const std::vector<cnoid::SgNodePtr>& marker = constraints_[m][j][k]->getDrawOnObjects();
            std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
          }
        }
        viewer_->drawOn(markers);
        viewer_->drawObjects(true);
      }
    }
    modelQueue_->push(m);
    return isSatisfied;

  }

};
