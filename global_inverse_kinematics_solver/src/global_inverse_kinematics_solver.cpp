#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>

namespace global_inverse_kinematics_solver{
  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& goals,
                const GIKParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path){

    ompl::base::StateSpacePtr ambientSpace = createAmbientSpace(variables);
    GIKConstraintPtr gikConstraint = std::make_shared<GIKConstraint>(ambientSpace, constraints);
    GIKStateSpacePtr stateSpace = std::make_shared<GIKStateSpace>(ambientSpace, gikConstraint);
    stateSpace->setDelta(param.delta); // この距離内のstateは、中間のconstraintチェック無しで遷移可能
    ompl::base::ConstrainedSpaceInformationPtr spaceInformation = std::make_shared<ompl::base::ConstrainedSpaceInformation>(stateSpace);
    spaceInformation->setStateValidityChecker(std::make_shared<ompl::base::AllValidStateValidityChecker>(spaceInformation)); // validは全てconstraintでチェックするので、StateValidityCheckerは全てvalidでよい
    ompl::geometric::SimpleSetup simpleSetup(spaceInformation);

    ompl::base::ScopedState<> start(stateSpace);
    link2State(stateSpace, start.get());
    simpleSetup.setStartState(start);

    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > goalConstraints = constraints;
    std::copy(goals.begin(), goals.end(), std::back_inserter(goalConstraints));
    GIKConstraintPtr goalGIKConstraint = std::make_shared<GIKConstraint>(ambientSpace, goalConstraints);
    GIKStateSpacePtr goalStateSpace = std::make_shared<GIKStateSpace>(ambientSpace, goalGIKConstraint);
    GIKGoalSpacePtr goal = std::make_shared<GIKGoalSpace>(spaceInformation);
    goal->setSpace(goalStateSpace);
    simpleSetup.setGoal(goal);

    //std::shared_ptr<ompl_near_projection::geometric::NearKPIECE1> planner = std::make_shared<ompl_near_projection::geometric::NearKPIECE1>(simpleSetup.getSpaceInformation());
    std::shared_ptr<ompl_near_projection::geometric::NearEST> planner = std::make_shared<ompl_near_projection::geometric::NearEST>(spaceInformation);
    //planner->setProjectionEvaluator("pos");
    planner->setRange(param.range); // This parameter greatly influences the runtime of the algorithm. It represents the maximum length of a motion to be added in the tree of motions.
    simpleSetup.setPlanner(planner);


    simpleSetup.setup();


    // attempt to solve the problem within one second of planning time
    ompl::base::PlannerStatus solved = simpleSetup.solve(param.timeout);

    if(param.debugLevel > 0){
      simpleSetup.getSolutionPath().print(std::cout);
    }

    if(path != nullptr){
      path->resize(simpleSetup.getSolutionPath().getStateCount());
      for(int i=0;i<simpleSetup.getSolutionPath().getStateCount();i++){
        //stateSpace->getDimension()は,SO3StateSpaceが3を返してしまう(実際はquaternionで4)ので、使えない
        ompl::base::ScopedState<> state(stateSpace);
        state = simpleSetup.getSolutionPath().getState(i);
        std::vector<double> values = state.reals();
        path->at(i).resize(values.size());
        for(int j=0;j<values.size();j++){
          path->at(i)[j] = state[j];
        }
      }
    }

    return solved == ompl::base::PlannerStatus::EXACT_SOLUTION;
  }

  void frame2Variables(const std::vector<double>& frame, const std::vector<cnoid::LinkPtr>& variables){
    int index = 0;
    for(int i=0;i<variables.size();i++){
      if(variables[i]->isRevoluteJoint() || variables[i]->isPrismaticJoint()) {
        variables[i]->q() = frame[index];
        index+=1;
      }else if(variables[i]->isFreeJoint()) {
        variables[i]->p()[0] = frame[index+0];
        variables[i]->p()[1] = frame[index+1];
        variables[i]->p()[2] = frame[index+2];
        variables[i]->R() = cnoid::Quaternion(frame[index+6],
                                              frame[index+3],
                                              frame[index+4],
                                              frame[index+5]).toRotationMatrix();
        index+=7;
      }
    }
  }
}
