#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>

namespace global_inverse_kinematics_solver{
  bool solveGIK(const std::vector<cnoid::LinkPtr>& variables,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
                const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& goals){

    ompl::base::StateSpacePtr ambientSpace = createAmbientSpace(variables);
    GIKConstraintPtr gikConstraint = std::make_shared<GIKConstraint>(ambientSpace, constraints);
    ompl_near_projection::NearProjectedStateSpacePtr stateSpace = std::make_shared<ompl_near_projection::NearProjectedStateSpace>(ambientSpace, gikConstraint);
    ompl::base::ConstrainedSpaceInformationPtr spaceInformation = std::make_shared<ompl::base::ConstrainedSpaceInformation>(stateSpace);
    ompl::geometric::SimpleSetup simpleSetup(spaceInformation);

    ompl::base::ScopedState<> start(stateSpace);
    link2State(stateSpace, start.get());
    simpleSetup.setStartState(start);

    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > goalConstraints = constraints;
    std::copy(goals.begin(), goals.end(), std::back_inserter(goalConstraints));
    GIKConstraintPtr goalGIKConstraint = std::make_shared<GIKConstraint>(ambientSpace, goalConstraints);
    ompl_near_projection::NearProjectedStateSpacePtr goalStateSpace = std::make_shared<ompl_near_projection::NearProjectedStateSpace>(ambientSpace, goalGIKConstraint);
    GIKGoalSpacePtr goal = std::make_shared<GIKGoalSpace>(spaceInformation);
    goal->setSpace(goalStateSpace);
    simpleSetup.setGoal(goal);

    std::shared_ptr<ompl_near_projection::geometric::NearKPIECE1> planner = std::make_shared<ompl_near_projection::geometric::NearKPIECE1>(simpleSetup.getSpaceInformation());
    //planner->setProjectionEvaluator("pos");
    planner->setRange(0.1); // This parameter greatly influences the runtime of the algorithm. It represents the maximum length of a motion to be added in the tree of motions.
    simpleSetup.setPlanner(planner);

    simpleSetup.setup();
    simpleSetup.print();

    // attempt to solve the problem within one second of planning time
    ompl::base::PlannerStatus solved = simpleSetup.solve(10.0);

    simpleSetup.getSolutionPath().print(std::cout);

    return solved;
  }
}
