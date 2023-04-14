#include <global_inverse_kinematics_solver/CnoidStateSpace.h>

namespace global_inverse_kinematics_solver{

  void CnoidRealVectorStateSpace::StateType::link2State(){
    for(int i=0;i<links_.size();i++){
      values[i] = links_[i]->q();
    }
  }
  void CnoidRealVectorStateSpace::StateType::state2Link(){
    for(int i=0;i<links_.size();i++){
      links_[i]->q() = values[i];
    }
  }
  ompl::base::State *CnoidRealVectorStateSpace::allocState() const {
    auto *rstate = new StateType(links_);
    rstate->values = new double[dimension_];
    return rstate;
  }
  void CnoidSE3StateSpace::StateType::link2State(){
    setXYZ(link_->p()[0], link_->p()[1], link_->p()[2]);
    cnoid::Quaternion q(link_->R());
    rotation().w = q.w();
    rotation().x = q.x();
    rotation().y = q.y();
    rotation().z = q.z();
  }
  void CnoidSE3StateSpace::StateType::state2Link(){
    link_->p()[0] = getX();
    link_->p()[1] = getY();
    link_->p()[2] = getZ();
    link_->R() = cnoid::Quaternion(rotation().w,
                                   rotation().x,
                                   rotation().y,
                                   rotation().z).toRotationMatrix();
  }
  ompl::base::State *CnoidSE3StateSpace::allocState() const {
    auto *state = new StateType(link_);
    allocStateComponents(state);
    return state;
  }

  void state2Link(const ompl::base::State *state){
    const ompl::base::WrapperStateSpace::StateType* wrapperState = std::dynamic_cast<ompl::base::WrapperStateSpace::StateType>(state);
    if(wrapperState != nullptr) return state2Link(state->getState());
    const CnoidRealVectorStateSpace::StateType* realVectorState = std::dynamic_cast<CnoidRealVectorStateSpace::StateType>(state);
    if(realVectorState != nullptr) return state->state2Link();
    const CnoidSE3StateSpace::StateType* se3State = std::dynamic_cast<CnoidRealVectorStateSpace::StateType>(state);
    if(se3State != nullptr) return state->state2Link();
    const ompl::base::CompoundStateSpace::StateType* compoundState = std::dynamic_cast<ompl::base::CompoundStateSpace::StateType>(state);
    if(compoundState != nullptr) {
      for(int i=0;i<compoundStateSpace->getSubspaceCount();i++){
        state2VariablesImpl((*compoundState)[i], compoundStateSpace->getSubspace(i), variables, variablesIndex);
      }
    }
  }
  void link2State(const ompl::base::State *state){
  }

  ompl::base::StateSpacePtr createAmbientSpace(const std::vector<cnoid::LinkPtr>& variables){
    ompl::base::StateSpacePtr ambientSpace = nullptr;
    std::vector<cnoid::LinkPtr> realVectorVariables;
    for(int i=0;i<variables.size();i++){
      if(variables[i]->isRevoluteJoint() || variables[i]->isPrismaticJoint()) {
        realVectorVariables.push_back(variables[i]);
      }else if(variables[i]->isFreeJoint()) {
        if(realVectorVariables.size() > 0){
          std::shared_ptr<CnoidRealVectorStateSpace> realVectorStateSpace = std::make_shared<CnoidRealVectorStateSpace>(realVectorVariables, realVectorVariables.size());
          ambientSpace = ambientSpace + realVectorStateSpace;
        }
        std::shared_ptr<CnoidSE3StateSpace> se3StateSpace = std::make_shared<CnoidSE3StateSpace>(variables[i]);
        ambientSpace = ambientSpace + se3StateSpace;
      }
    }
    if(realVectorVariables.size() > 0){
      std::shared_ptr<CnoidRealVectorStateSpace> realVectorStateSpace = std::make_shared<CnoidRealVectorStateSpace>(realVectorVariables, realVectorVariables.size());
      ambientSpace = ambientSpace + realVectorStateSpace;
    }

    if(ambientSpace == nullptr){
      ambientSpace = std::make_shared<CnoidRealVectorStateSpace>(std::vector<cnoid::LinkPtr>(),0);
    }

    return ambientSpace;
  }


};

