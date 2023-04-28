#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>


namespace global_inverse_kinematics_solver{
  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& goals,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
                const GIKParam& param,
                std::shared_ptr<std::vector<std::vector<double> > > path){
    return solveGIK(variables, constraints, std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >{goals}, nominals, param, std::vector<std::shared_ptr<std::vector<std::vector<double> > > >{path});
  }

  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& goals,
                const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& nominals,
                const GIKParam& param,
                const std::vector<std::shared_ptr<std::vector<std::vector<double> > > >& path){
    std::vector<std::vector<cnoid::LinkPtr> > variabless{variables};
    std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > > constraintss{constraints};
    std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > > goalss{goals};
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > nominalss{nominals};
    std::shared_ptr<UintQueue> modelQueue = std::make_shared<UintQueue>();
    GIKParam param2(param);
    modelQueue->push(0);

    if(param.threads >= 2){
      std::set<cnoid::BodyPtr> bodies = getBodies(variables);
      for(int i=1;i<param.threads;i++){
        modelQueue->push(i);
        std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
        for(std::set<cnoid::BodyPtr>::iterator it = bodies.begin(); it != bodies.end(); it++){
          modelMap[*it] = (*it)->clone();
        }
        variabless.push_back(std::vector<cnoid::LinkPtr>(variables.size()));
        for(int v=0;v<variables.size();v++){
          variabless.back()[v] = modelMap[variables[v]->body()]->link(variables[v]->index());
        }
        constraintss.push_back(std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >(constraints.size()));
        for(int j=0;j<constraints.size();j++){
          constraintss.back()[j].resize(constraints[j].size());
          for(int k=0;k<constraints[j].size();k++){
            constraintss.back()[j][k] = constraints[j][k]->clone(modelMap);
          }
        }
        goalss.push_back(std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >(goals.size()));
        for(int j=0;j<goals.size();j++){
          goalss.back()[j].resize(goals[j].size());
          for(int k=0;k<goals[j].size();k++){
            goalss.back()[j][k] = goals[j][k]->clone(modelMap);
          }
        }
        nominalss.push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(nominals.size()));
        for(int j=0;j<nominals.size();j++){
          nominalss.back()[j] = nominals[j]->clone(modelMap);
        }
        if(param.projectLink.size() == 1){
          param2.projectLink.push_back(modelMap[param.projectLink[0]->body()]->link(param.projectLink[0]->index()));
        }
      }
    }

    return solveGIK(variabless,
                    constraintss,
                    goalss,
                    nominalss,
                    modelQueue,
                    param2,
                    path);
  }

  bool solveGIK(const std::vector<std::vector<cnoid::LinkPtr> >& variables, // 0: modelQueue, 1: variables
                const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints, // 0: modelQueue, 1: constriant priority 2: constraints
                const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& goals, // 0: modelQueue. 1: goalSpace(OR). 2: goals(AND).
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& nominals, // 0: modelQueue, 1: nominals
                std::shared_ptr<UintQueue> modelQueue,
                const GIKParam& param,
                const std::vector<std::shared_ptr<std::vector<std::vector<double> > > >& path){
    if((variables.size() == 0) ||
       (variables.size() != constraints.size()) ||
       (constraints.size() != goals.size()) ||
       (goals.size() != nominals.size())
       ){
      std::cerr << "[solveGIK] size mismatch" << std::endl;
      return false;
    }

    bool calculate_path = true;
    if(goals.size() == 0 ||
       goals[0].size() != path.size()) calculate_path=false;
    for(int i=0;i<path.size();i++) {
      if(path[i] == nullptr) {
        calculate_path = false;
        break;
      }
    }

    ompl::base::StateSpacePtr ambientSpace = createAmbientSpace(variables[0]);
    GIKConstraintPtr gikConstraint = std::make_shared<GIKConstraint>(ambientSpace, modelQueue, constraints, variables);
    gikConstraint->viewer() = param.viewer;
    gikConstraint->drawLoop() = param.drawLoop;
    gikConstraint->param() = param.pikParam;
    gikConstraint->nearMaxError() = param.nearMaxError;
    GIKStateSpacePtr stateSpace = std::make_shared<GIKStateSpace>(ambientSpace, gikConstraint);
    stateSpace->setDelta(param.delta); // この距離内のstateは、中間のconstraintチェック無しで遷移可能
    ompl_near_projection::NearConstrainedSpaceInformationPtr spaceInformation = std::make_shared<ompl_near_projection::NearConstrainedSpaceInformation>(stateSpace);
    spaceInformation->setStateValidityChecker(std::make_shared<ompl::base::AllValidStateValidityChecker>(spaceInformation)); // validは全てconstraintでチェックするので、StateValidityCheckerは全てvalidでよい
    spaceInformation->setup(); // ここでsetupを呼ばないと、stateSpaceがsetupされないのでlink2State等ができない

    ompl_near_projection::NearProblemDefinitionPtr problemDefinition = std::make_shared<ompl_near_projection::NearProblemDefinition>(spaceInformation);

    ompl::base::ScopedState<> start(stateSpace);
    link2State(variables[0], stateSpace, start.get());
    problemDefinition->clearStartStates();
    problemDefinition->addStartState(start);

    std::vector<ompl_near_projection::NearGoalSpacePtr> goalSpaces;
    for(int i=0;i<goals[0].size();i++){
      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > goal_(goals.size()); // 0: modelQueue, q: goals(AND)
      for(int m=0;m<goals.size();m++) goal_[m] = goals[m][i];
      GIKGoalSpacePtr goal = std::make_shared<GIKGoalSpace>(spaceInformation, ambientSpace, modelQueue, constraints, variables, goal_, nominals);
      goal->setViewer(param.viewer);
      goal->setDrawLoop(param.drawLoop);
      goal->setParam(param.pikParam);
      goal->setNearMaxError(param.nearMaxError);
      goalSpaces.push_back(goal);
    }
    problemDefinition->setGoals(goalSpaces);

    ompl::base::PlannerPtr planner;

    if(param.projectLink.size() == variables.size()){
      GIKProjectionEvaluatorPtr proj = std::make_shared<GIKProjectionEvaluator>(stateSpace, modelQueue, variables);
      proj->parentLink() = param.projectLink;
      proj->localPos() = param.projectLocalPose;
      proj->setCellSizes(std::vector<double>(proj->getDimension(), param.projectCellSize));

      if(param.threads <= 1){
        std::shared_ptr<ompl_near_projection::geometric::NearKPIECE1> planner_ = std::make_shared<ompl_near_projection::geometric::NearKPIECE1>(spaceInformation);
        planner_->setProjectionEvaluator(proj);
        planner_->setRange(param.range); // This parameter greatly influences the runtime of the algorithm. It represents the maximum length of a motion to be added in the tree of motions.
        planner_->setGoalBias(param.goalBias);

        planner = planner_;

      }else{
        std::shared_ptr<ompl_near_projection::geometric::pNearKPIECE1> planner_ = std::make_shared<ompl_near_projection::geometric::pNearKPIECE1>(spaceInformation);
        planner_->setProjectionEvaluator(proj);
        planner_->setThreadCount(param.threads);
        planner_->setRange(param.range); // This parameter greatly influences the runtime of the algorithm. It represents the maximum length of a motion to be added in the tree of motions.
        planner_->setGoalBias(param.goalBias);
        planner = planner_;

      }
    }else{
      std::shared_ptr<ompl_near_projection::geometric::NearEST> planner_ = std::make_shared<ompl_near_projection::geometric::NearEST>(spaceInformation);
      planner_->setRange(param.range); // This parameter greatly influences the runtime of the algorithm. It represents the maximum length of a motion to be added in the tree of motions.
      planner_->setGoalBias(param.goalBias);
      planner = planner_;
    }

    if(!spaceInformation->isSetup()) spaceInformation->setup();
    planner->setProblemDefinition(problemDefinition);
    if(!planner->isSetup()) planner->setup();

    // たまに、サンプル時のIKと補間時のIKが微妙に違うことが原因で、解のpathの補間に失敗する場合があるので、成功するまでとき直す. path.check()と、interpolateやsimplifyのIKは同じものなので、最初のpath.check()が成功すれば、simplifyやinterpolate後のpathも必ずcheck()が成功すると想定.
    ompl::base::PlannerStatus solved;

    //planner->clear();
    //problemDefition->clearSolutionPaths();

    {
      ompl::time::point start = ompl::time::now();
      solved = planner->solve(param.timeout);
      double planTime = ompl::time::seconds(ompl::time::now() - start);
      if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
        OMPL_INFORM("Solution found in %f seconds", planTime);
      else
        OMPL_INFORM("No solution found after %f seconds", planTime);
    }

    // IKの途中経過を使っているので、EXACT_SOLUTIONなら、必ずinterpolateできている.
    // 一時的にstateが最適化の誤差で微妙に!isSatisfiedになっていたり、deltaを上回った距離になっていることがあって、それは許容したい.
    // solutionPath.check()は行わない

    // goal stateをvariablesに反映して返す.
    if(problemDefinition->hasSolution()) {
      const ompl::geometric::PathGeometricPtr solutionPath = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(problemDefinition->getSolutionPath());
      state2Link(stateSpace, solutionPath->getState(solutionPath->getStateCount()-1), variables[0]);
      std::set<cnoid::BodyPtr> bodies = getBodies(variables[0]);
      for(std::set<cnoid::BodyPtr>::const_iterator it=bodies.begin(); it != bodies.end(); it++){
        (*it)->calcForwardKinematics(false); // 疎な軌道生成なので、velocityはチェックしない
        (*it)->calcCenterOfMass();
      }
    }

    if(calculate_path){
      for(int i=0;i<path.size();i++){
        ompl::geometric::PathGeometricPtr solutionPath = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(problemDefinition->getSolutionPathForEachGoal()[i]);
        if(param.debugLevel > 1){
          solutionPath->print(std::cout);
        }

        ompl::geometric::PathSimplifierPtr pathSimplifier = std::make_shared<ompl::geometric::PathSimplifier>(spaceInformation);
        ompl::time::point start = ompl::time::now();
        std::size_t numStates = solutionPath->getStateCount();
        gikConstraint->viewer() = nullptr; // simplifySolution()中は描画しない
        pathSimplifier->simplify(*solutionPath, param.timeout);
        gikConstraint->viewer() = param.viewer; // simplifySolution()中は描画しない
        double simplifyTime = ompl::time::seconds(ompl::time::now() - start);
        OMPL_INFORM("Path simplification took %f seconds and changed from %d to %d states",
                    simplifyTime, numStates, solutionPath->getStateCount());

        if(param.debugLevel > 1){
          solutionPath->print(std::cout);
        }

        solutionPath->interpolate();
        if(param.debugLevel > 1){
          solutionPath->print(std::cout);
        }

        // 途中の軌道をpathに入れて返す
        path[i]->resize(solutionPath->getStateCount());
        for(int j=0;j<solutionPath->getStateCount();j++){
          //stateSpace->getDimension()は,SO3StateSpaceが3を返してしまう(実際はquaternionで4)ので、使えない
          state2Frame(stateSpace, solutionPath->getState(j), path[i]->at(j));
        }
      }
    }

    return solved == ompl::base::PlannerStatus::EXACT_SOLUTION;
  }
}
