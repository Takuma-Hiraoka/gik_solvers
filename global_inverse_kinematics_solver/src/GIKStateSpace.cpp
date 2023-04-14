#include <global_inverse_kinematics_solver/GIKStateSpace.h>
#include <global_inverse_kinematics_solver/GIKConstraint.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace global_inverse_kinematics_solver{

  void GIKStateSpace::state2Variables (const ompl::base::State *state, const ompl::base::StateSpacePtr space, const std::vector<cnoid::LinkPtr>& variables){
    int variablesIndex = 0;
    return state2VariablesImpl(state, space, variables, variablesIndex);
  }

  void GIKStateSpace::state2VariablesImpl (const ompl::base::State *state, const ompl::base::StateSpacePtr space, const std::vector<cnoid::LinkPtr>& variables, int& variablesIndex){

    const ompl::base::WrapperStateSpacePtr& wrapperStateSpace = std::dynamic_pointer_cast<ompl::base::WrapperStateSpace>(space);
    if(wrapperStateSpace != nullptr){
      state2VariablesImpl(state, wrapperStateSpace->getSpace(), variables, variablesIndex);
      return;
    }

    const std::shared_ptr<ompl::base::RealVectorStateSpace>& realVectorStateSpace = std::dynamic_pointer_cast<ompl::base::RealVectorStateSpace>(space);
    if(realVectorStateSpace != nullptr){
      const ompl::base::RealVectorStateSpace::StateType* realVectorState = state->as<ompl::base::RealVectorStateSpace::StateType>();
      for(int i=0;i<realVectorStateSpace->getDimension(); i++){
        variables[variablesIndex]->q() = (*realVectorState)[i];
        variablesIndex++;
      }
      return;
    }

    const std::shared_ptr<ompl::base::SE3StateSpace>& se3StateSpace = std::dynamic_pointer_cast<ompl::base::SE3StateSpace>(space);
    if(se3StateSpace != nullptr){
      const ompl::base::SE3StateSpace::StateType* se3State = state->as<ompl::base::SE3StateSpace::StateType>();
      variables[variablesIndex]->p()[0] = se3State->getX();
      variables[variablesIndex]->p()[1] = se3State->getY();
      variables[variablesIndex]->p()[2] = se3State->getZ();
      variables[variablesIndex]->R() = cnoid::Quaternion(se3State->rotation().w,
                                                         se3State->rotation().x,
                                                         se3State->rotation().y,
                                                         se3State->rotation().z).toRotationMatrix();
      variablesIndex++;
      return;
    }

    const std::shared_ptr<ompl::base::CompoundStateSpace>& compoundStateSpace = std::dynamic_pointer_cast<ompl::base::CompoundStateSpace>(space);
    if(compoundStateSpace != nullptr){
      const ompl::base::CompoundStateSpace::StateType* compoundState = state->as<ompl::base::CompoundStateSpace::StateType>();
      for(int i=0;i<compoundStateSpace->getSubspaceCount();i++){
        state2VariablesImpl((*compoundState)[i], compoundStateSpace->getSubspace(i), variables, variablesIndex);
      }
      return;
    }
  }

  GIKStateSpacePtr createGIKStateSpace(const std::vector<cnoid::LinkPtr>& variables, std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints){
    ompl::base::StateSpacePtr ambientSpace = createAmbientSpace(variables);
    GIKConstraintPtr gikConstraint = std::make_shared<GIKConstraint>(ambientSpace, constraints);
    return std::make_shared<GIKStateSpace>(ambientSpace, gikConstraint, variables);
  }
};
