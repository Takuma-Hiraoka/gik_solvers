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
  void sample4_jaxon(){
    cnoid::BodyLoader bodyLoader;

    // load robot
    std::string modelfile = ros::package::getPath("msl_hand_models") + "/JAXON_RED_WITH_MSLHAND/JAXON_REDmain.wrl";
    cnoid::BodyPtr robot = bodyLoader.load(modelfile);
    if(!robot) std::cerr << "!robot" << std::endl;

    // load desk
    std::string deskModelfile = ros::package::getPath("global_inverse_kinematics_solver_sample") + "/models/desk.wrl";
    cnoid::BodyPtr desk = bodyLoader.load(deskModelfile);

    // setup viewer
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    viewer->objects(std::vector<cnoid::BodyPtr>{robot, desk});


    // setup constraints
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0; // 自己干渉のvclip計算などに時間がかかるので、使いまわしたほうがいい

    // task: joint limit
    for(int i=0;i<robot->numJoints();i++){
      std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
      constraint->joint() = robot->joint(i);
      constraints0.push_back(constraint);
    }

    {
      // task: self collision
      std::vector<std::vector<std::string> > pairs {
        std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT2"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","LLEG_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT6"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT6"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT6"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT3"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT4"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT5"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT6"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT2","WAIST"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT3","WAIST"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT4","WAIST"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT5","WAIST"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT6","LARM_JOINT6"}, std::vector<std::string>{"RARM_JOINT6","WAIST"}, std::vector<std::string>{"LARM_JOINT2","WAIST"}, std::vector<std::string>{"LARM_JOINT3","WAIST"}, std::vector<std::string>{"LARM_JOINT4","WAIST"}, std::vector<std::string>{"LARM_JOINT5","WAIST"}, std::vector<std::string>{"LARM_JOINT6","WAIST"}, std::vector<std::string>{"RLEG_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT1","LARM_JOINT7"}, std::vector<std::string>{"HEAD_JOINT1","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT0","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT3","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT4","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT5","LARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT7"}, std::vector<std::string>{"LARM_JOINT7","WAIST"}, std::vector<std::string>{"RLEG_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT3","RARM_JOINT7"}, std::vector<std::string>{"RLEG_JOINT5","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT3","RARM_JOINT7"}, std::vector<std::string>{"LLEG_JOINT5","RARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT1","RARM_JOINT7"}, std::vector<std::string>{"HEAD_JOINT1","RARM_JOINT7"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT0"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT2"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT3"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT4"}, std::vector<std::string>{"RARM_JOINT7","LARM_JOINT5"}, std::vector<std::string>{"RARM_JOINT7","WAIST"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT3"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT4"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT5"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT6"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT7"}, std::vector<std::string>{"CHEST_JOINT2","LARM_JOINT2"}, std::vector<std::string>{"CHEST_JOINT2","RARM_JOINT2"}, std::vector<std::string>{"RLEG_JOINT2","HANDBASE_R"}, std::vector<std::string>{"RLEG_JOINT3","HANDBASE_R"}, std::vector<std::string>{"RLEG_JOINT5","HANDBASE_R"}, std::vector<std::string>{"LLEG_JOINT2","HANDBASE_R"}, std::vector<std::string>{"LLEG_JOINT3","HANDBASE_R"}, std::vector<std::string>{"LLEG_JOINT5","HANDBASE_R"}, std::vector<std::string>{"WAIST","HANDBASE_R"}, std::vector<std::string>{"CHEST_JOINT1","HANDBASE_R"}, std::vector<std::string>{"CHEST_JOINT2","HANDBASE_R"}, std::vector<std::string>{"HEAD_JOINT1","HANDBASE_R"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT0"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT2"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT3"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT4"}, std::vector<std::string>{"HANDBASE_R","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","R_THUMB_JOINT1"}, std::vector<std::string>{"RLEG_JOINT3","R_THUMB_JOINT1"}, std::vector<std::string>{"RLEG_JOINT5","R_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT2","R_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT3","R_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT5","R_THUMB_JOINT1"}, std::vector<std::string>{"WAIST","R_THUMB_JOINT1"}, std::vector<std::string>{"CHEST_JOINT1","R_THUMB_JOINT1"}, std::vector<std::string>{"CHEST_JOINT2","R_THUMB_JOINT1"}, std::vector<std::string>{"HEAD_JOINT1","R_THUMB_JOINT1"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT0"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT2"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"R_THUMB_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","HANDBASE_L"}, std::vector<std::string>{"RLEG_JOINT3","HANDBASE_L"}, std::vector<std::string>{"RLEG_JOINT5","HANDBASE_L"}, std::vector<std::string>{"LLEG_JOINT2","HANDBASE_L"}, std::vector<std::string>{"LLEG_JOINT3","HANDBASE_L"}, std::vector<std::string>{"LLEG_JOINT5","HANDBASE_L"}, std::vector<std::string>{"WAIST","HANDBASE_L"}, std::vector<std::string>{"CHEST_JOINT1","HANDBASE_L"}, std::vector<std::string>{"CHEST_JOINT2","HANDBASE_L"}, std::vector<std::string>{"HEAD_JOINT1","HANDBASE_L"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT0"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT2"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT3"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT4"}, std::vector<std::string>{"HANDBASE_L","LARM_JOINT5"}, std::vector<std::string>{"RLEG_JOINT2","L_THUMB_JOINT1"}, std::vector<std::string>{"RLEG_JOINT3","L_THUMB_JOINT1"}, std::vector<std::string>{"RLEG_JOINT5","L_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT2","L_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT3","L_THUMB_JOINT1"}, std::vector<std::string>{"LLEG_JOINT5","L_THUMB_JOINT1"}, std::vector<std::string>{"WAIST","L_THUMB_JOINT1"}, std::vector<std::string>{"CHEST_JOINT1","L_THUMB_JOINT1"}, std::vector<std::string>{"CHEST_JOINT2","L_THUMB_JOINT1"}, std::vector<std::string>{"HEAD_JOINT1","L_THUMB_JOINT1"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT0"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT2"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT3"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT4"}, std::vector<std::string>{"L_THUMB_JOINT1","LARM_JOINT5"}, std::vector<std::string>{"HANDBASE_R","HANDBASE_L"}, std::vector<std::string>{"R_THUMB_JOINT1","L_THUMB_JOINT1"}
      };

      for(int i=0;i<pairs.size();i++){
        std::shared_ptr<ik_constraint2_vclip::VclipCollisionConstraint> constraint = std::make_shared<ik_constraint2_vclip::VclipCollisionConstraint>();
        constraint->A_link() = robot->link(pairs[i][0]);
        constraint->B_link() = robot->link(pairs[i][1]);
        constraint->tolerance() = 0.01;
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


    while(true){

      // reset manip pose
      robot->rootLink()->p() = cnoid::Vector3(0,0,1.0);
      robot->rootLink()->v().setZero();
      robot->rootLink()->R() = cnoid::Matrix3::Identity();
      robot->rootLink()->w().setZero();
      std::vector<double> reset_manip_pose{
        0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// rleg
          0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// lleg
          0.0, 0.0, 0.0, // torso
          0.0, 0.0, // head
          0.0, 0.959931, -0.349066, -0.261799, -1.74533, -0.436332, 0.0, -0.785398,// rarm
          0.0, 0.959931, 0.349066, 0.261799, -1.74533, 0.436332, 0.0, -0.785398,// larm
          0.0, 1.5708, 0.0, 0.0, 0.0, 0.0, // right_hand
          0.0, 1.5708, 0.0, 0.0, 0.0, 0.0 // left_hand
          };

      for(int j=0; j < robot->numJoints(); ++j){
        robot->joint(j)->q() = reset_manip_pose[j];
      }
      robot->calcForwardKinematics();
      robot->calcCenterOfMass();

      desk->rootLink()->p() = cnoid::Vector3(0.9,0.0,0.7); // これ以上Zが高いとreset-manip-poseの手と干渉する
      desk->calcForwardKinematics();
      desk->calcCenterOfMass();

      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1;
      {
        // task: rleg to current position
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        constraint->A_link() = robot->link("RLEG_JOINT5");
        constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.1);
        constraint->B_link() = nullptr;
        constraint->B_localpos() = constraint->A_link()->T() * constraint->A_localpos();
        constraints1.push_back(constraint);
      }
      {
        // task: lleg to current position
        std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
        constraint->A_link() = robot->link("LLEG_JOINT5");
        constraint->A_localpos().translation() = cnoid::Vector3(0.0,0.0,-0.1);
        constraint->B_link() = nullptr;
        constraint->B_localpos() = constraint->A_link()->T() * constraint->A_localpos();
        constraints1.push_back(constraint);
      }
      {
        // task: COM to current position
        std::shared_ptr<ik_constraint2::COMConstraint> constraint = std::make_shared<ik_constraint2::COMConstraint>();
        constraint->A_robot() = robot;
        constraint->B_localp() = robot->centerOfMass();
        constraint->weight() << 1.0, 1.0, 0.0; // Z free
        constraints1.push_back(constraint);
      }

      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0,constraints1};

      // setup goals
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > goal0;
      std::shared_ptr<ik_constraint2::PositionConstraint> goalRaw;
      {
        // task: rarm to target.
        std::shared_ptr<ik_constraint2::PositionConstraint> goal = std::make_shared<ik_constraint2::PositionConstraint>();
        goal->A_link() = robot->link("RARM_JOINT7");
        goal->A_localpos().translation() = cnoid::Vector3(0.0,0.055,-0.217);
        goal->A_localpos().linear() = cnoid::Matrix3(cnoid::AngleAxis(1.5708, cnoid::Vector3(0,1,0)));
        goal->B_link() = nullptr;
        goal->B_localpos().translation() = cnoid::Vector3(0.55,-0.25,0.45); // below desk
        goal0.push_back(goal);

        goalRaw = goal;
      }

      std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > goals{goal0};

      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > nominals;
      {
        // task: joint angle to target
        for(int i=0;i<robot->numJoints()-12;i++){ // handの12関節は除く
          std::shared_ptr<ik_constraint2::JointAngleConstraint> constraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
          constraint->joint() = robot->joint(i);
          constraint->targetq() = reset_manip_pose[i];
          constraint->precision() = 1e10; // always satisfied
          nominals.push_back(constraint);
        }
      }

      std::vector<cnoid::LinkPtr> variables;
      variables.push_back(robot->rootLink());
      for(size_t i=0;i<robot->numJoints()-12;i++){ // handの12関節は除く
        variables.push_back(robot->joint(i));
      }

      // for(size_t i=0;i<constraints.size();i++){
      //   for(size_t j=0;j<constraints[i].size();j++){
      //     constraints[i][j]->debugLevel() = 1;//debug
      //   }
      // }
      global_inverse_kinematics_solver::GIKParam param;
      param.debugLevel=0;
      param.range = 0.3; // 0.2よりも0.3の方が速い
      param.delta = 0.2; // 大きければ大きいほど速はずだが、干渉計算や補間の正確さが犠牲になる.
      param.goalBias = 0.2; // 0.05よりも0.2や0.3の方が速い. goalSampingはIKの変位が大きいので、この値が大きいとsample1回あたりの時間が長くなるデメリットもある.
      param.timeout = 30.0;
      param.projectLink.push_back(goalRaw->A_link());
      param.projectLocalPose = goalRaw->A_localpos();
      param.projectCellSize = 0.2; // 0.05よりも0.1の方が速い. 0.3よりも0.2の方が速い
      param.viewer = viewer;
      param.drawLoop = 1;
      param.threads = 10;
      param.pikParam.we = 3e2;
      std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();
      bool solved = global_inverse_kinematics_solver::solveGIK(variables,
                                                               constraints,
                                                               goals,
                                                               nominals,
                                                               param,
                                                               path);


      std::copy(goals[0].begin(), goals[0].end(), std::back_inserter(constraints[1]));
      goals[0].clear();
      {
        // task: rarm to target.
        std::shared_ptr<ik_constraint2::PositionConstraint> goal = std::make_shared<ik_constraint2::PositionConstraint>();
        goal->A_link() = robot->link("LARM_JOINT7");
        goal->A_localpos().translation() = cnoid::Vector3(0.0,-0.055,-0.217);
        goal->A_localpos().linear() = cnoid::Matrix3(cnoid::AngleAxis(1.5708, cnoid::Vector3(0,1,0)));
        goal->B_localpos().translation() = cnoid::Vector3(0.55,0.25,0.45); // below desk
        goals[0].push_back(goal);

        goalRaw = goal;
      }
      param.projectLink[0] = goalRaw->A_link();
      param.projectLocalPose = goalRaw->A_localpos();
      std::shared_ptr<std::vector<std::vector<double> > > path2 = std::make_shared<std::vector<std::vector<double> > >();
      bool solved2 = global_inverse_kinematics_solver::solveGIK(variables,
                                                                constraints,
                                                                goals,
                                                                nominals,
                                                                param,
                                                                path2);
      std::copy(path2->begin(), path2->end(), std::back_inserter(*path));

      std::cerr << "solved: " << solved2 << std::endl;

      // for(size_t i=0;i<constraints.size();i++){
      //   for(size_t j=0;j<constraints[i].size();j++){
      //     constraints[i][j]->debugLevel() = 0;//not debug
      //     constraints[i][j]->updateBounds();
      //     if(constraints[i][j]->isSatisfied()) std::cerr << "constraint " << i << " " << j << ": satisfied"<< std::endl;
      //     else std::cerr << "constraint " << i << " " << j << ": NOT satisfied"<< std::endl;
      //   }
      // }
      // for(size_t i=0;i<goals.size();i++){
      //   for(size_t j=0;j<goals[i].size();j++){
      //     goals[i][j]->debugLevel() = 0;//not debug
      //     goals[i][j]->updateBounds();
      //     if(goals[i][j]->isSatisfied()) std::cerr << "goal " << i << " " << j << ": satisfied"<< std::endl;
      //     else std::cerr << "goal " << i << " " << j << ": NOT satisfied"<< std::endl;
      //   }
      // }

      // main loop
      for(int i=0;i<path->size();i++){
        global_inverse_kinematics_solver::frame2Link(path->at(i),variables);
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
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }

}
