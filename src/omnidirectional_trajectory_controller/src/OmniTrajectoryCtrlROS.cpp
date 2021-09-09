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
    //


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
  omni_ctrl_ = new OmniTrajectoryCtrl(kNFuture);
  omni_ctrl_->LoadTrajectoryFile(
      "/home/sousarbarb/catkin_ws/src/omnidirectional_trajectory_controller/"
      "input/square_vn-1.0.txt");

  return true;
}

}  // omnidirectional_trajectory_controller
