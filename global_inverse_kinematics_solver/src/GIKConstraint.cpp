#include <global_inverse_kinematics_solver/GIKConstraint.h>

namespace global_inverse_kinematics_solver{
  bool GIKConstraint::project(ompl::base::State *state) const{
    std::cerr << "GIKConstraint::project" << std::endl;
    return projectNearValid(state, state);
  }

  bool GIKConstraint::projectNearValid(ompl::base::State *state, const ompl::base::State *near, double* distance) const{
    const unsigned int m = modelQueue_->pop();

    state2Link(stateSpace_, state, variables_[m]); // spaceとstateの空間をそろえる

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
          constraint->maxError() = nearMaxError_;

        }else if(variables_[m][i]->isFreeJoint()) {
          std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::dynamic_pointer_cast<ik_constraint2::PositionConstraint>(nearConstraints[i]);
          if(constraint == nullptr) {
            nearConstraints[i] = constraint = std::make_shared<ik_constraint2::PositionConstraint>();
          }
          constraint->A_link() = variables_[m][i];
          constraint->B_localpos() = variables_[m][i]->T();
          constraint->precision() = 1e10; // always satisfied
          constraint->maxError() << nearMaxError_, nearMaxError_, nearMaxError_, nearMaxError_, nearMaxError_, nearMaxError_;
        }else{
          std::cerr << "[GIKConstraint::projectNear] something is wrong" << std::endl;
        }
      }
    }

    state2Link(stateSpace_, near, variables_[m]); // spaceとstateの空間をそろえる

    std::shared_ptr<std::vector<std::vector<double> > > path;
    ompl_near_projection::NearProjectedStateSpace::StateType* tmp_state = dynamic_cast<ompl_near_projection::NearProjectedStateSpace::StateType*>(state);
    if(tmp_state) path = std::make_shared<std::vector<std::vector<double> > >();
    bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables_[m],
                                                                      ikConstraints_[m],
                                                                      rejections_[m],
                                                                      tasks_[m],
                                                                      param_,
                                                                      path);

    link2State(variables_[m], stateSpace_, state); // spaceとstateの空間をそろえる

    if(tmp_state) {
      for(int i=0;i<tmp_state->intermediateStates.size();i++) stateSpace_->freeState(tmp_state->intermediateStates[i]);
      tmp_state->intermediateStates.clear();
      for(int i=0;i+1<path->size();i++){ // 終点を含まない
        ompl::base::State* st = stateSpace_->allocState();
        frame2State(path->at(i), stateSpace_, st);
        tmp_state->intermediateStates.push_back(st);
      }
    }

    if(distance != nullptr){
      double squaredDistance = 0.0;
      for(size_t i=0;i<constraints_[m].size();i++){
        for(size_t j=0;j<constraints_[m][i].size();j++){
          //constraints_[m][i][j]->updateBounds();
          squaredDistance += std::pow(constraints_[m][i][j]->distance(), 2.0);
        }
      }
      *distance = std::sqrt(squaredDistance);
    }

    if(viewer_ != nullptr && m==0){
      loopCount_++;
      if(loopCount_%drawLoop_==0){
        std::vector<cnoid::SgNodePtr> markers;
        for(int j=0;j<constraints_[m].size();j++){
          for(int k=0;k<constraints_[m][j].size(); k++){constraints_[m][j][k]->updateBounds();
            const std::vector<cnoid::SgNodePtr>& marker = constraints_[m][j][k]->getDrawOnObjects();
            std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
          }
        }
        viewer_->drawOn(markers);
        viewer_->drawObjects(true);
      }
    }

    modelQueue_->push(m);

    return solved;
  }

  bool GIKConstraint::projectNearValidWithNominal(ompl::base::State *state, const ompl::base::State *near, double* distance) const{
    const unsigned int m = modelQueue_->pop();

    state2Link(stateSpace_, state, variables_[m]); // spaceとstateの空間をそろえる

    {
      // setup nearConstraints
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominalConstraints = ikConstraints_[m].back();
      nominalConstraints.resize(nominalConstraints_[m].size());
      for(int i=0;i<nominalConstraints_[m].size(); i++){
        nominalConstraints[i] = nominalConstraints_[m][i];
      }
    }

    state2Link(stateSpace_, near, variables_[m]); // spaceとstateの空間をそろえる

    std::shared_ptr<std::vector<std::vector<double> > > path;
    ompl_near_projection::NearProjectedStateSpace::StateType* tmp_state = dynamic_cast<ompl_near_projection::NearProjectedStateSpace::StateType*>(state);
    if(tmp_state) path = std::make_shared<std::vector<std::vector<double> > >();
    bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables_[m],
                                                                      ikConstraints_[m],
                                                                      rejections_[m],
                                                                      tasks_[m],
                                                                      param_,
                                                                      path);

    link2State(variables_[m], stateSpace_, state); // spaceとstateの空間をそろえる

    if(tmp_state) {
      for(int i=0;i<tmp_state->intermediateStates.size();i++) stateSpace_->freeState(tmp_state->intermediateStates[i]);
      tmp_state->intermediateStates.clear();
      for(int i=0;i+1<path->size();i++){ // 終点を含まない
        ompl::base::State* st = stateSpace_->allocState();
        frame2State(path->at(i), stateSpace_, st);
        tmp_state->intermediateStates.push_back(st);
      }
    }

    if(distance != nullptr){
      double squaredDistance = 0.0;
      for(size_t i=0;i<constraints_[m].size();i++){
        for(size_t j=0;j<constraints_[m][i].size();j++){
          //constraints_[m][i][j]->updateBounds();
          squaredDistance += std::pow(constraints_[m][i][j]->distance(), 2.0);
        }
      }
      *distance = std::sqrt(squaredDistance);
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
    for(size_t i=0;i<rejections_[m].size();i++){
      rejections_[m][i]->updateBounds();
      squaredDistance += std::pow(rejections_[m][i]->distance(), 2.0);
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
    for(size_t i=0;i<rejections_[m].size();i++){
      rejections_[m][i]->updateBounds();
      if(!rejections_[m][i]->isSatisfied()) {
        if(viewer_ == nullptr || m!=0) {
          modelQueue_->push(m);
          return false;
        }
        satisfied = false;
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
    for(size_t i=0;i<rejections_[m].size();i++){
      rejections_[m][i]->updateBounds();
      if(!rejections_[m][i]->isSatisfied()) isSatisfied = false;
      squaredDistance += std::pow(rejections_[m][i]->distance(), 2.0);
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

  class JointAngle {
  public:
    cnoid::Position T;
    double q;
  };

  bool GIKConstraint2::projectNearValid(ompl::base::State *state, const ompl::base::State *near, double* distance) const{
    const unsigned int m = modelQueue_->pop();

    state2Link(stateSpace_, near, variables_[m]); // spaceとstateの空間をそろえる

    ompl::base::State* current = stateSpace_->allocState();
    link2State(variables_[m], stateSpace_, current); // current->intermediateStatesが空であるように
    std::vector<ompl::base::State* > intermediateStates;
    double prevDist = stateSpace_->distance(current, state);

    std::vector<JointAngle> prevAngle(variables_[m].size());
    for(int k=0;k<variables_[m].size();k++){
      if(variables_[m][k]->isRevoluteJoint() || variables_[m][k]->isPrismaticJoint()) {
        prevAngle[k].q = variables_[m][k]->q();
      }else if(variables_[m][k]->isFreeJoint()) {
        prevAngle[k].T = variables_[m][k]->T();
      }
    }

    for(int loop=0;loop<50;loop++){
      double prev = variables_[m][1]->q();
      state2Link(stateSpace_, state, variables_[m]); // spaceとstateの空間をそろえる
      double tgt = variables_[m][1]->q();

      for(int k=0;k<variables_[m].size();k++){
        if(variables_[m][k]->isRevoluteJoint() || variables_[m][k]->isPrismaticJoint()) {
          variables_[m][k]->q() = std::min(std::max(variables_[m][k]->q(), prevAngle[k].q - this->projectionRange), prevAngle[k].q + this->projectionRange);
        }else if(variables_[m][k]->isFreeJoint()) {
          for(int j=0;j<3;j++){
            variables_[m][k]->p()[j] = std::min(std::max(variables_[m][k]->p()[j], prevAngle[k].T.translation()[j] - this->projectionRange), prevAngle[k].T.translation()[j] + this->projectionRange);
          }
          cnoid::AngleAxisd w(prevAngle[k].T.linear().transpose() * variables_[m][k]->R());
          variables_[m][k]->R() = prevAngle[k].T.linear() * cnoid::AngleAxisd(std::min(std::max(w.angle(), - this->projectionRange), this->projectionRange), w.axis());
        }
      }

      {
        // setup nearConstraints
        std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nearConstraints = ikConstraints_[m].back();
        nearConstraints.clear();
      }
      std::shared_ptr<std::vector<std::vector<double> > > path;
      bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables_[m],
                                                                        ikConstraints_[m],
                                                                        tasks_[m],
                                                                        param_,
                                                                        path);

      for(int k=0;k<variables_[m].size();k++){
        if(variables_[m][k]->isRevoluteJoint() || variables_[m][k]->isPrismaticJoint()) {
          prevAngle[k].q = variables_[m][k]->q();
        }else if(variables_[m][k]->isFreeJoint()) {
          prevAngle[k].T = variables_[m][k]->T();
        }
      }

      link2State(variables_[m], stateSpace_, current); // spaceとstateの空間をそろえる

      bool reject = false;
      for(size_t i=0;i<rejections_[m].size();i++){
        rejections_[m][i]->updateBounds();
        if(!rejections_[m][i]->isSatisfied()) {
          reject = true;
          break;
        }
      }
      if(reject) break;

      double dist = stateSpace_->distance(current, state);

      if(dist < prevDist - this->projectionTrapThre) { // srは0.01, jは0.03
        ompl::base::State* st = stateSpace_->allocState();
        stateSpace_->copyState(st, current); // current->intermediateStatesが空であるように
        intermediateStates.push_back(st);

        prevDist = dist;
      }else{
        break;
      }
    }

    if(intermediateStates.size() > 0){
      stateSpace_->copyState(state, intermediateStates.back());
    }else{
      stateSpace_->copyState(state, near);
    }
    ompl_near_projection::NearProjectedStateSpace::StateType* tmp_state = dynamic_cast<ompl_near_projection::NearProjectedStateSpace::StateType*>(state);
    if(tmp_state) {
      for(int i=0;i<tmp_state->intermediateStates.size();i++) stateSpace_->freeState(tmp_state->intermediateStates[i]);
      tmp_state->intermediateStates.clear();
      for(int i=0;i+1<intermediateStates.size();i++){ // 終点を含まない
        ompl::base::State* st = stateSpace_->allocState();
        stateSpace_->copyState(st, intermediateStates[i]);
        tmp_state->intermediateStates.push_back(st);
      }
    }

    if(distance != nullptr){
      state2Link(stateSpace_, state, variables_[m]); // spaceとstateの空間をそろえる
      for(std::set<cnoid::BodyPtr>::const_iterator it=bodies_[m].begin(); it != bodies_[m].end(); it++){
        (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
        (*it)->calcCenterOfMass();
      }

      double squaredDistance = 0.0;
      for(size_t i=0;i<constraints_[m].size();i++){
        for(size_t j=0;j<constraints_[m][i].size();j++){
          //constraints_[m][i][j]->updateBounds();
          squaredDistance += std::pow(constraints_[m][i][j]->distance(), 2.0);
        }
      }
      *distance = std::sqrt(squaredDistance);
    }

    if(viewer_ != nullptr && m==0){
      loopCount_++;
      if(loopCount_%drawLoop_==0){
        std::vector<cnoid::SgNodePtr> markers;
        for(int j=0;j<constraints_[m].size();j++){
          for(int k=0;k<constraints_[m][j].size(); k++){constraints_[m][j][k]->updateBounds();
            const std::vector<cnoid::SgNodePtr>& marker = constraints_[m][j][k]->getDrawOnObjects();
            std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
          }
        }
        viewer_->drawOn(markers);
        viewer_->drawObjects(true);
      }
    }

    stateSpace_->freeState(current);
    for(int i=0;i<intermediateStates.size();i++){
      stateSpace_->freeState(intermediateStates[i]);
    }

    modelQueue_->push(m);

    return true;
  }

};
