#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <cnoid/SceneMarkers>
#include <iostream>
#include <ros/package.h>

#include <global_inverse_kinematics_solver/global_inverse_kinematics_solver.h>
#include <ik_constraint2/ik_constraint2.h>
#include <ik_constraint2_vclip/ik_constraint2_vclip.h>

namespace global_inverse_kinematics_solver_sample{
  void sample2_desk(){
    cnoid::BodyLoader bodyLoader;

    // load robot
    std::string modelfile = ros::package::getPath("choreonoid") + "/share/model/SR1/SR1.body";
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

    // load desk
    std::string deskModelfile = ros::package::getPath("global_inverse_kinematics_solver_sample") + "/models/desk.wrl";
    cnoid::BodyPtr desk = bodyLoader.load(deskModelfile);
    desk->rootLink()->p() = cnoid::Vector3(0.8,0.0,0.71);
    desk->calcForwardKinematics();
    desk->calcCenterOfMass();

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(std::vector<cnoid::BodyPtr>{robot, desk});

    // setup constraints
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;

    // task: joint limit
    for(int i=0;i<robot->numJoints();i++){
      std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
      constraint->joint() = robot->joint(i);
      constraints0.push_back(constraint);
    }

    {
      // task: self collision
      std::vector<std::vector<std::string> > pairs{
        std::vector<std::string>{"LARM_SHOULDER_R","WAIST"},
        std::vector<std::string>{"LARM_ELBOW","WAIST"},
        std::vector<std::string>{"LARM_WRIST_R","WAIST"},
        std::vector<std::string>{"RARM_SHOULDER_R","WAIST"},
        std::vector<std::string>{"RARM_ELBOW","WAIST"},
        std::vector<std::string>{"RARM_WRIST_R","WAIST"},
        std::vector<std::string>{"LARM_SHOULDER_R","WAIST_R"},
        std::vector<std::string>{"LARM_ELBOW","WAIST_R"},
        std::vector<std::string>{"LARM_WRIST_R","WAIST_R"},
        std::vector<std::string>{"RARM_SHOULDER_R","WAIST_R"},
        std::vector<std::string>{"RARM_ELBOW","WAIST_R"},
        std::vector<std::string>{"RARM_WRIST_R","WAIST_R"},
        std::vector<std::string>{"LARM_SHOULDER_R","CHEST"},
        std::vector<std::string>{"LARM_ELBOW","CHEST"},
        std::vector<std::string>{"LARM_WRIST_R","CHEST"},
        std::vector<std::string>{"RARM_SHOULDER_R","CHEST"},
        std::vector<std::string>{"RARM_ELBOW","CHEST"},
        std::vector<std::string>{"RARM_WRIST_R","CHEST"}
      };
      for(int i=0;i<pairs.size();i++){
        std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = std::make_shared<ik_constraint2_vclip::VclipCollisionConstraint>();
        constraint->A_link() = robot->link(pairs[i][0]);
        constraint->B_link() = robot->link(pairs[i][1]);
        constraint->tolerance() = 0.03;
        constraints0.push_back(constraint);
      }
    }

    {
      // task: env collision
      for(int i=0;i<robot->numLinks();i++){
        std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = std::make_shared<ik_constraint2_vclip::VclipCollisionConstraint>();
        constraint->A_link() = robot->link(i);
        constraint->B_link() = desk->rootLink();
        constraint->tolerance() = 0.03;
        constraints0.push_back(constraint);
      }
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
    std::shared_ptr<ik_constraint2::PositionConstraint> goalRaw;
    {
      // task: rarm to target.
      std::shared_ptr<ik_constraint2::PositionConstraint> goal = std::make_shared<ik_constraint2::PositionConstraint>();
      goal->A_link() = robot->link("RARM_WRIST_R");
      goal->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.02);
      goal->B_link() = nullptr;
      goal->B_localpos().translation() = cnoid::Vector3(0.3,-0.2,0.4); // below desk
      goal->B_localpos().linear() = cnoid::Matrix3(cnoid::AngleAxis(-1.5,cnoid::Vector3(0,1,0)));
      goal0.push_back(goal);

      goalRaw = goal;
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
    global_inverse_kinematics_solver::GIKParam param;
    param.debugLevel=1;
    param.range = 0.3; // 0.2よりも0.3の方が速い
    param.delta = 0.2; // 大きければ大きいほど速いが、干渉計算の正確さが犠牲になる
    param.timeout = 30.0;
    param.projectLink = goalRaw->A_link();
    param.projectLocalPose = goalRaw->A_localpos();
    param.projectCellSize = 0.15; // 0.05よりも0.1の方が速い. 0.3よりも0.2の方が速い
    param.viewer = viewer;
    std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();
    bool solved = global_inverse_kinematics_solver::solveGIK(variables,
                                                             constraints,
                                                             goals,
                                                             param,
                                                             path);
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

    // main loop
    for(int i=0;i<path->size();i++){
      global_inverse_kinematics_solver::frame2Variables(path->at(i),variables);
      robot->calcForwardKinematics(false);
      robot->calcCenterOfMass();

      std::vector<cnoid::SgNodePtr> markers;
      for(int j=0;j<constraints.size();j++){
        for(int k=0;k<constraints[j].size(); k++){
          constraints[j][k]->debugLevel() = 0;
          constraints[j][k]->updateBounds();
          const std::vector<cnoid::SgNodePtr>& marker = constraints[j][k]->getDrawOnObjects();
          std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
        }
      }
      for(int j=0;j<goals.size();j++){
        for(int k=0;k<goals[j].size(); k++){
          goals[j][k]->debugLevel() = 0;
          goals[j][k]->updateBounds();
          const std::vector<cnoid::SgNodePtr>& marker = goals[j][k]->getDrawOnObjects();
          std::copy(marker.begin(), marker.end(), std::back_inserter(markers));
        }
      }
      viewer->drawOn(markers);
      viewer->drawObjects();

      std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }

  }

}
