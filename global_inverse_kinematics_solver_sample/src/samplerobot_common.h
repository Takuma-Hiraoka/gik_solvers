#ifndef SAMPLEROBOT_COMMON_H
#define SAMPLEROBOT_COMMON_H

#include <cnoid/Body>

namespace global_inverse_kinematics_solver_sample{

  void generateSampleRobot(cnoid::BodyPtr& robot, cnoid::BodyPtr& abstractRobot, cnoid::BodyPtr& horizontalRobot);

};

#endif
