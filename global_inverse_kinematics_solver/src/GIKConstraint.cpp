#include <global_inverse_kinematics_solver/GIKConstraint.h>

namespace global_inverse_kinematics_solver{
  bool GIKConstraint::project(ompl::base::State *state) const{
    return projectNear(state, state);
  }

  bool GIKConstraint::projectNear(ompl::base::State *state, const ompl::base::State *near) const{

    state2Link(ambientSpace_, state);

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

    state2Link(ambientSpace_, near);

    bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables_,
                                                                      ikConstraints_,
                                                                      tasks_,
                                                                      param_);

    link2State(ambientSpace_, state);

    return solved;
  }

  double GIKConstraint::distance (const ompl::base::State *state) const {
    state2Link(ambientSpace_, state);
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
    state2Link(ambientSpace_, state);
    for(std::set<cnoid::BodyPtr>::const_iterator it=bodies_.begin(); it != bodies_.end(); it++){
      (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
      (*it)->calcCenterOfMass();
    }

    for(size_t i=0;i<constraints_.size();i++){
      for(size_t j=0;j<constraints_[i].size();j++){
        constraints_[i][j]->updateBounds();
        if(!constraints_[i][j]->isSatisfied()) return false;
      }
    }
    return true;
  }
  bool GIKConstraint::isSatisfied (const ompl::base::State *state, double *distance) const {
    if(!distance) return isSatisfied(state);

    state2Link(ambientSpace_, state);
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

    return isSatisfied;

  }

};
