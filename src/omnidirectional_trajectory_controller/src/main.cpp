#include <ros/ros.h>

#include "omnidirectional_trajectory_controller/OmniTrajectoryCtrlROS.h"

int main(int argc, char *argv[]) {
  ros::init(argc,argv,"omnidirectional_trajectory_controller");

  omnidirectional_trajectory_controller::OmniTrajectoryCtrlROS omni_ctrl;

  omni_ctrl.Execute();

  return 0;
}