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
    GIKConstraintPtr gikConstraint = std::make_shared<GIKConstraint>(ambientSpace, constraints, variables);
    GIKStateSpacePtr stateSpace = std::make_shared<GIKStateSpace>(ambientSpace, gikConstraint);
    stateSpace->setDelta(param.delta); // この距離内のstateは、中間のconstraintチェック無しで遷移可能
    ompl::base::ConstrainedSpaceInformationPtr spaceInformation = std::make_shared<ompl::base::ConstrainedSpaceInformation>(stateSpace);
    spaceInformation->setStateValidityChecker(std::make_shared<ompl::base::AllValidStateValidityChecker>(spaceInformation)); // validは全てconstraintでチェックするので、StateValidityCheckerは全てvalidでよい
    spaceInformation->setup(); // ここでsetupを呼ばないと、stateSpaceがsetupされないのでlink2State等ができない

    ompl::geometric::SimpleSetup simpleSetup(spaceInformation);

    ompl::base::ScopedState<> start(stateSpace);
    link2State(variables, stateSpace, start.get());
    simpleSetup.setStartState(start);

    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > goalConstraints = constraints;
    std::copy(goals.begin(), goals.end(), std::back_inserter(goalConstraints));
    GIKConstraintPtr goalGIKConstraint = std::make_shared<GIKConstraint>(ambientSpace, goalConstraints, variables);
    goalGIKConstraint->viewer() = param.viewer;
    GIKStateSpacePtr goalStateSpace = std::make_shared<GIKStateSpace>(ambientSpace, goalGIKConstraint);
    GIKGoalSpacePtr goal = std::make_shared<GIKGoalSpace>(spaceInformation);
    goal->setSpace(goalStateSpace);
    simpleSetup.setGoal(goal);

    if(param.projectLink != nullptr){
      std::shared_ptr<ompl_near_projection::geometric::NearKPIECE1> planner = std::make_shared<ompl_near_projection::geometric::NearKPIECE1>(simpleSetup.getSpaceInformation());
      GIKProjectionEvaluatorPtr proj = std::make_shared<GIKProjectionEvaluator>(stateSpace, variables);
      proj->parentLink() = param.projectLink;
      proj->localPos() = param.projectLocalPose;
      proj->setCellSizes(std::vector<double>(proj->getDimension(), param.projectCellSize));
      planner->setProjectionEvaluator(proj);
      planner->setRange(param.range); // This parameter greatly influences the runtime of the algorithm. It represents the maximum length of a motion to be added in the tree of motions.
      planner->setGoalBias(param.goalBias);
      simpleSetup.setPlanner(planner);
    }else{
      std::shared_ptr<ompl_near_projection::geometric::NearEST> planner = std::make_shared<ompl_near_projection::geometric::NearEST>(spaceInformation);
      planner->setRange(param.range); // This parameter greatly influences the runtime of the algorithm. It represents the maximum length of a motion to be added in the tree of motions.
      planner->setGoalBias(param.goalBias);
      simpleSetup.setPlanner(planner);
    }

    // attempt to solve the problem within one second of planning time
    simpleSetup.setup();
    ompl::base::PlannerStatus solved = simpleSetup.solve(param.timeout);

    ompl::geometric::PathGeometric solutionPath = simpleSetup.getSolutionPath();

    if(param.debugLevel > 0){
      std::cerr << solutionPath.check() << std::endl;
      solutionPath.print(std::cout);
    }

    if(path != nullptr){
      simpleSetup.simplifySolution();
      solutionPath = simpleSetup.getSolutionPath();

      if(param.debugLevel > 0){
        std::cerr << solutionPath.check() << std::endl;
        solutionPath.print(std::cout);
      }

      solutionPath.interpolate();
      if(param.debugLevel > 0){
        std::cerr << solutionPath.check() << std::endl;
        solutionPath.print(std::cout);
      }

      // 途中の軌道をpathに入れて返す
      path->resize(solutionPath.getStateCount());
      for(int i=0;i<solutionPath.getStateCount();i++){
        //stateSpace->getDimension()は,SO3StateSpaceが3を返してしまう(実際はquaternionで4)ので、使えない
        state2Frame(stateSpace, solutionPath.getState(i), path->at(i));
      }
    }

    // goal stateをvariablesに反映して返す
    state2Link(stateSpace, solutionPath.getState(solutionPath.getStateCount()-1), variables);
    std::set<cnoid::BodyPtr> bodies = getBodies(variables);
    for(std::set<cnoid::BodyPtr>::const_iterator it=bodies.begin(); it != bodies.end(); it++){
      (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
      (*it)->calcCenterOfMass();
    }

    return solved == ompl::base::PlannerStatus::EXACT_SOLUTION;
  }
}
