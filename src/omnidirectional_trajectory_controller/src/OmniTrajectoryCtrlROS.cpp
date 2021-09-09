#include "omnidirectional_trajectory_controller/OmniTrajectoryCtrlROS.h"

namespace omnidirectional_trajectory_controller {

OmniTrajectoryCtrlROS::OmniTrajectoryCtrlROS(ros::NodeHandle &nh)
    : nh_(nh) , lr_(kFreqPosCtrl) {
  // Read parameters
  if (!ReadParameters()) {
    ROS_ERROR("[OMNI_CTRL] Could not read parameters.");
    ros::requestShutdown();
  }

  // Publishers

  // Subscriptions

  // Services

  // Debug
  ROS_INFO("[OMNI_CTRL] Node launched successfully launched.");
}

void OmniTrajectoryCtrlROS::Execute() {
  while (ros::ok()) {
    // Update robot info saved in the controller state
    //omni_ctrl_->UpdateRobotInfo();

    // Execute 1 cycle of positioning control
    //omni_ctrl_->OmniRobotCtrl();

    // Spin/sleep to process callbacks and subscription of messages
    ros::spinOnce();
    lr_.sleep();
  }
}

OmniTrajectoryCtrlROS::~OmniTrajectoryCtrlROS() {
  delete omni_ctrl_;
}

bool OmniTrajectoryCtrlROS::ReadParameters() {
  // Read parameters
  // ...

  // Update OmniTrajectoryCtrl object
  omni_ctrl_ = new OmniTrajectoryCtrl(
      kNFuture, kGoalTolXY, kGoalTolTh, xXvel,
      kPDCtrlKcV, kPDCtrlKcVn, kPDCtrlKcW, kPDCtrlTdV, kPDCtrlTdVn, kPDCtrlTdW,
      kRobModelKpV , kRobModelKpVn , kRobModelKpW,
      kRobModelTauV, kRobModelTauVn, kRobModelTauW);

  // Test load trajectory
  std::string filename =
      "/home/sousarbarb/catkin_ws/src/omnidirectional_trajectory_controller"
      "/input/square_vn-1.0.txt";
  omni_ctrl_->LoadTrajectoryFile(filename);

  return true;
}

}  // omnidirectional_trajectory_controller
