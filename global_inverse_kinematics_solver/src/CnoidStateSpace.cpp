#include <global_inverse_kinematics_solver/CnoidStateSpace.h>

namespace global_inverse_kinematics_solver{

  void CnoidRealVectorStateSpace::link2State(ompl::base::State* state){
    ompl::base::RealVectorStateSpace::StateType* realVectorState = static_cast<ompl::base::RealVectorStateSpace::StateType*>(state);
    for(int i=0;i<links_.size();i++){
      realVectorState->values[i] = links_[i]->q();
    }
  }
  void CnoidRealVectorStateSpace::state2Link(const ompl::base::State* state){
    const ompl::base::RealVectorStateSpace::StateType* realVectorState = static_cast<const ompl::base::RealVectorStateSpace::StateType*>(state);
    for(int i=0;i<links_.size();i++){
      links_[i]->q() = realVectorState->values[i];
    }
  }
  void CnoidSE3StateSpace::link2State(ompl::base::State* state){
    ompl::base::SE3StateSpace::StateType* se3State = static_cast<ompl::base::SE3StateSpace::StateType*>(state);
    se3State->setXYZ(link_->p()[0], link_->p()[1], link_->p()[2]);
    cnoid::Quaternion q(link_->R());
    se3State->rotation().w = q.w();
    se3State->rotation().x = q.x();
    se3State->rotation().y = q.y();
    se3State->rotation().z = q.z();
  }
  void CnoidSE3StateSpace::state2Link(const ompl::base::State* state){
    const ompl::base::SE3StateSpace::StateType* se3State = static_cast<const ompl::base::SE3StateSpace::StateType*>(state);
    link_->p()[0] = se3State->getX();
    link_->p()[1] = se3State->getY();
    link_->p()[2] = se3State->getZ();
    link_->R() = cnoid::Quaternion(se3State->rotation().w,
                                   se3State->rotation().x,
                                   se3State->rotation().y,
                                   se3State->rotation().z).toRotationMatrix();
  }

  void state2Link(const ompl::base::StateSpacePtr& space, const ompl::base::State *state){

    // space, stateのどちらか一方がambientSpace, もう一方がそのWrapperStateSpaceでも良いようにしている
    const ompl::base::WrapperStateSpacePtr wrapperStateSpace = std::dynamic_pointer_cast<ompl::base::WrapperStateSpace>(space);
    if(wrapperStateSpace != nullptr){
      state2Link(wrapperStateSpace->getSpace(), state);
      return;
    }
    const ompl::base::WrapperStateSpace::StateType* wrapperState = dynamic_cast<const ompl::base::WrapperStateSpace::StateType*>(state);
    if(wrapperState) {
      state2Link(space, wrapperState->getState());
      return;
    }

    const CnoidRealVectorStateSpacePtr realVectorStateSpace = std::dynamic_pointer_cast<CnoidRealVectorStateSpace>(space);
    if(realVectorStateSpace != nullptr) {
      const ompl::base::RealVectorStateSpace::StateType* realVectorState = static_cast<const ompl::base::RealVectorStateSpace::StateType*>(state);
      realVectorStateSpace->state2Link(realVectorState);
      return;
    }
    const CnoidSE3StateSpacePtr se3StateSpace = std::dynamic_pointer_cast<CnoidSE3StateSpace>(space);
    if(se3StateSpace != nullptr) {
      const ompl::base::SE3StateSpace::StateType* se3State = static_cast<const ompl::base::SE3StateSpace::StateType*>(state);
      se3StateSpace->state2Link(se3State);
      return ;
    }
    const std::shared_ptr<ompl::base::CompoundStateSpace> compoundStateSpace = std::dynamic_pointer_cast<ompl::base::CompoundStateSpace>(space);
    if(compoundStateSpace != nullptr) {
      const ompl::base::CompoundStateSpace::StateType* compoundState = dynamic_cast<const ompl::base::CompoundStateSpace::StateType*>(state);
      for(int i=0;i<compoundStateSpace->getSubspaceCount();i++){
        state2Link(compoundStateSpace->getSubspace(i), (*compoundState)[i]);
      }
      return;
    }
  }
  void link2State(const ompl::base::StateSpacePtr& space, ompl::base::State *state){

    // space, stateのどちらか一方がambientSpace, もう一方がそのWrapperStateSpaceでも良いようにしている
    const ompl::base::WrapperStateSpacePtr wrapperStateSpace = std::dynamic_pointer_cast<ompl::base::WrapperStateSpace>(space);
    if(wrapperStateSpace != nullptr){
      link2State(wrapperStateSpace->getSpace(), state);
      return;
    }
    ompl::base::WrapperStateSpace::StateType* wrapperState = dynamic_cast<ompl::base::WrapperStateSpace::StateType*>(state);
    if(wrapperState){
      link2State(space, wrapperState->getState());
      return;
    }

    const CnoidRealVectorStateSpacePtr realVectorStateSpace = std::dynamic_pointer_cast<CnoidRealVectorStateSpace>(space);
    if(realVectorStateSpace != nullptr) {
      ompl::base::RealVectorStateSpace::StateType* realVectorState = static_cast<ompl::base::RealVectorStateSpace::StateType*>(state);
      realVectorStateSpace->link2State(realVectorState);
      return;
    }
    const CnoidSE3StateSpacePtr se3StateSpace = std::dynamic_pointer_cast<CnoidSE3StateSpace>(space);
    if(se3StateSpace != nullptr) {
      ompl::base::SE3StateSpace::StateType* se3State = static_cast<ompl::base::SE3StateSpace::StateType*>(state);
      se3StateSpace->link2State(se3State);
      return ;
    }
    const std::shared_ptr<ompl::base::CompoundStateSpace> compoundStateSpace = std::dynamic_pointer_cast<ompl::base::CompoundStateSpace>(space);
    if(compoundStateSpace != nullptr) {
      ompl::base::CompoundStateSpace::StateType* compoundState = dynamic_cast<ompl::base::CompoundStateSpace::StateType*>(state);
      for(int i=0;i<compoundStateSpace->getSubspaceCount();i++){
        link2State(compoundStateSpace->getSubspace(i), (*compoundState)[i]);
      }
      return;
    }
  }

  std::vector<cnoid::LinkPtr> getLinks(const ompl::base::StateSpacePtr& space){
    std::vector<cnoid::LinkPtr> links;
    getLinks(space, links);
    return links;
  }

  void getLinks(const ompl::base::StateSpacePtr& space, std::vector<cnoid::LinkPtr>& links){
    const ompl::base::WrapperStateSpacePtr wrapperStateSpace = std::dynamic_pointer_cast<ompl::base::WrapperStateSpace>(space);
    if(wrapperStateSpace != nullptr){
      getLinks(wrapperStateSpace->getSpace(), links);
      return;
    }
    const CnoidRealVectorStateSpacePtr realVectorStateSpace = std::dynamic_pointer_cast<CnoidRealVectorStateSpace>(space);
    if(realVectorStateSpace != nullptr) {
      const std::vector<cnoid::LinkPtr>& tmp_links = realVectorStateSpace->links();
      std::copy(tmp_links.begin(), tmp_links.end(), std::back_inserter(links));
      return;
    }
    const CnoidSE3StateSpacePtr se3StateSpace = std::dynamic_pointer_cast<CnoidSE3StateSpace>(space);
    if(se3StateSpace != nullptr) {
      const cnoid::LinkPtr& tmp_link = se3StateSpace->link();
      links.push_back(tmp_link);
      return ;
    }
    const std::shared_ptr<ompl::base::CompoundStateSpace> compoundStateSpace = std::dynamic_pointer_cast<ompl::base::CompoundStateSpace>(space);
    if(compoundStateSpace != nullptr) {
      for(int i=0;i<compoundStateSpace->getSubspaceCount();i++){
        getLinks(compoundStateSpace->getSubspace(i), links);
      }
      return;
    }
  }

  std::set<cnoid::BodyPtr> getBodies(const std::vector<cnoid::LinkPtr>& links){
    std::set<cnoid::BodyPtr> bodies;
    for(size_t i=0;i<links.size();i++){
      if(links[i]->body()) bodies.insert(links[i]->body());
    }
    return bodies;
  }

  ompl::base::StateSpacePtr createAmbientSpace(const std::vector<cnoid::LinkPtr>& variables){
    ompl::base::StateSpacePtr ambientSpace = nullptr;
    std::vector<cnoid::LinkPtr> realVectorVariables;
    std::vector<double> low, high; // boundが無いとエラーになるので、一応関節角度上下限 or rootLinkの現在地+-1mで与えている
    for(int i=0;i<variables.size();i++){
      if(variables[i]->isRevoluteJoint() || variables[i]->isPrismaticJoint()) {
        realVectorVariables.push_back(variables[i]);
        low.push_back(variables[i]->q_lower());
        high.push_back(variables[i]->q_upper());
      }else if(variables[i]->isFreeJoint()) {
        if(realVectorVariables.size() > 0){
          std::shared_ptr<CnoidRealVectorStateSpace> realVectorStateSpace = std::make_shared<CnoidRealVectorStateSpace>(realVectorVariables, realVectorVariables.size());
          ompl::base::RealVectorBounds bounds(realVectorVariables.size());
          bounds.low = low;
          bounds.high = high;
          realVectorStateSpace->setBounds(bounds);
          ambientSpace = ambientSpace + realVectorStateSpace;
          realVectorVariables.clear();
          low.clear();
          high.clear();
        }
        std::shared_ptr<CnoidSE3StateSpace> se3StateSpace = std::make_shared<CnoidSE3StateSpace>(variables[i]);
        ompl::base::RealVectorBounds bounds(3);
        bounds.low = std::vector<double>{variables[i]->p()[0]-1.0,variables[i]->p()[1]-1.0,variables[i]->p()[2]-1.0};
        bounds.high = std::vector<double>{variables[i]->p()[0]+1.0,variables[i]->p()[1]+1.0,variables[i]->p()[2]+1.0};
        se3StateSpace->setBounds(bounds);
        ambientSpace = ambientSpace + se3StateSpace;
      }
    }
    if(realVectorVariables.size() > 0){
      std::shared_ptr<CnoidRealVectorStateSpace> realVectorStateSpace = std::make_shared<CnoidRealVectorStateSpace>(realVectorVariables, realVectorVariables.size());
      ompl::base::RealVectorBounds bounds(realVectorVariables.size());
      bounds.low = low;
      bounds.high = high;
      realVectorStateSpace->setBounds(bounds);
      ambientSpace = ambientSpace + realVectorStateSpace;
      realVectorVariables.clear();
      low.clear();
      high.clear();
    }

    if(ambientSpace == nullptr){
      ambientSpace = std::make_shared<CnoidRealVectorStateSpace>(std::vector<cnoid::LinkPtr>(),0);
    }

    ambientSpace->registerDefaultProjection(std::make_shared<DummyProjectionEvaluator>(ambientSpace)); // WrapperStateSpace::setup()のときに、ambientSpaceのdefaultProjectionが無いとエラーになる. CompoundStateSpaceにはDefaultProjectionが無いので、とりあえず適当に与えておく

    return ambientSpace;
  }


};

