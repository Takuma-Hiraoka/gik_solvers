#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <ik_constraint2/PositionConstraint.h>
#include <ik_constraint2/COMConstraint.h>

#include <cnoid/BodyLoader>
#include <ros/package.h>

int main(void){
  // load robot
  std::string modelfile = ros::package::getPath("choreonoid") + "/share/model/SR1/SR1.body";
  cnoid::BodyLoader bodyLoader;
  cnoid::BodyPtr robot = bodyLoader.load(modelfile);

  // reset manip pose
  robot->rootLink()->p() = cnoid::Vector3(0,0,0.6);
  robot->rootLink()->v().setZero();
  robot->rootLink()->R() = cnoid::Matrix3::Identity();
  robot->rootLink()->w().setZero();
  std::vector<double> reset_manip_pose{
    0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// rleg
      0.523599, 0.0, 0.0, -1.74533, 0.15708, -0.113446, 0.637045,// rarm
      0.0, -0.349066, 0.0, 0.820305, -0.471239, 0.0,// lleg
      0.523599, 0.0, 0.0, -1.74533, -0.15708, -0.113446, -0.637045,// larm
      0.0, 0.0, 0.0};

  for(int j=0; j < robot->numJoints(); ++j){
    robot->joint(j)->q() = reset_manip_pose[j];
  }
  robot->calcForwardKinematics();
  robot->calcCenterOfMass();

  // setup constraints
  std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
  // joint limit
  for(int i=0;i<robot->numJoints();i++){
    std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
    constraint->joint() = robot->joint(i);
    constraints0.push_back(constraint);
  }

  std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1;
  {
    // task: rleg to current position
    std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
    constraint->A_link() = robot->link("RLEG_ANKLE_R");
    constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.04);
    constraint->B_link() = nullptr;
    constraint->B_localpos() = robot->link("RLEG_ANKLE_R")->T() * constraint->A_localpos();
    constraints1.push_back(constraint);
  }
  {
    // task: lleg to current position
    std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
    constraint->A_link() = robot->link("LLEG_ANKLE_R");
    constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.04);
    constraint->B_link() = nullptr;
    constraint->B_localpos() = robot->link("LLEG_ANKLE_R")->T() * constraint->A_localpos();
    constraints1.push_back(constraint);
  }
  {
    // task: COM to current position
    std::shared_ptr<ik_constraint2::COMConstraint> constraint = std::make_shared<ik_constraint2::COMConstraint>();
    constraint->A_robot() = robot;
    constraint->B_localp() = robot->centerOfMass();
    constraints1.push_back(constraint);
  }

  std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0,constraints1};

  // setup goals
  std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > goal0;
  {
    // task: rarm to target.
    std::shared_ptr<ik_constraint2::PositionConstraint> goal = std::make_shared<ik_constraint2::PositionConstraint>();
    goal->A_link() = robot->link("RARM_WRIST_R");
    goal->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.02);
    goal->B_link() = nullptr;
    goal->B_localpos().translation() = cnoid::Vector3(0.3,-0.2,0.8);
    goal->B_localpos().linear() = cnoid::Matrix3(cnoid::AngleAxis(-1.5,cnoid::Vector3(0,1,0)));
    goal0.push_back(goal);
  }

  std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > goals{goal0};

  std::vector<cnoid::LinkPtr> variables;
  variables.push_back(robot->rootLink());
  for(size_t i=0;i<robot->numJoints();i++){
    variables.push_back(robot->joint(i));
  }

  for(size_t i=0;i<constraints.size();i++){
    for(size_t j=0;j<constraints[i].size();j++){
      constraints[i][j]->debugLevel() = 0;//debug
    }
  }
  bool solved = global_inverse_kinematics_solver::solveGIK(variables,
                                                           constraints,
                                                           goals);

  std::cerr << "solved: " << solved << std::endl;

  for(size_t i=0;i<constraints.size();i++){
    for(size_t j=0;j<constraints[i].size();j++){
      constraints[i][j]->debugLevel() = 0;//not debug
      constraints[i][j]->updateBounds();
      if(constraints[i][j]->isSatisfied()) std::cerr << "constraint " << i << " " << j << ": satisfied"<< std::endl;
      else std::cerr << "constraint " << i << " " << j << ": NOT satisfied"<< std::endl;
    }
  }

  for(size_t i=0;i<goals.size();i++){
    for(size_t j=0;j<goals[i].size();j++){
      goals[i][j]->debugLevel() = 0;//not debug
      goals[i][j]->updateBounds();
      if(goals[i][j]->isSatisfied()) std::cerr << "goal " << i << " " << j << ": satisfied"<< std::endl;
      else std::cerr << "goal " << i << " " << j << ": NOT satisfied"<< std::endl;
    }
  }
}
