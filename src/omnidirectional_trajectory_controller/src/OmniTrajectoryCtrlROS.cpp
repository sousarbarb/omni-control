#include "omnidirectional_trajectory_controller/OmniTrajectoryCtrlROS.h"

namespace omnidirectional_trajectory_controller {

// Public functions

OmniTrajectoryCtrlROS::OmniTrajectoryCtrlROS()
    : lr_(kFreqPosCtrl) {
  // Initialization
  nh_ = ros::NodeHandle("~");

  // Read parameters
  if (!ReadParameters()) {
    ROS_ERROR("[OMNI_CTRL] Could not read parameters.");
    ros::requestShutdown();
  }

  // Publishers
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // Subscriptions
  sub_odom_ = nh_.subscribe("odom", 100,
                            &OmniTrajectoryCtrlROS::SubOdom, this);

  // Services
  srv_load_trajectory = nh_.advertiseService(
      "load_trajectory",
      &OmniTrajectoryCtrlROS::SrvLoadTrajectory, this);
  srv_set_future_size = nh_.advertiseService(
      "set_future_size",
      &OmniTrajectoryCtrlROS::SrvSetFutureSize, this);
  srv_set_xvel = nh_.advertiseService(
      "set_xvel",
      &OmniTrajectoryCtrlROS::SrvSetXvel, this);

#ifdef ANALYZE_PROCESSING_TIME
  srv_analyze_time = nh_.advertiseService(
      "set_analyze_time",
      &OmniTrajectoryCtrlROS::SrvAnalyzeTime, this);
#endif

  // Debug
  ROS_INFO("[OMNI_CTRL] Node launched successfully launched.");
}

void OmniTrajectoryCtrlROS::Execute() {
  // Initialization
  double v, vn, w;

  while (ros::ok()) {
    // Execute 1 cycle of positioning control
    omni_ctrl_->OmniRobotCtrl(v, vn, w);

    // Publish the computed velocity of the robot (based on the desired pose)
    PubCmdVel(v, vn, w);

    // Spin/sleep to process callbacks and subscription of messages
    ros::spinOnce();
    lr_.sleep();
  }
}

OmniTrajectoryCtrlROS::~OmniTrajectoryCtrlROS() {
  delete omni_ctrl_;
  pub_cmd_vel_.shutdown();
  sub_odom_.shutdown();
}



// Private functions

bool OmniTrajectoryCtrlROS::ReadParameters() {
  // Initialization
  ros::NodeHandle nh("~");

  // Read parameters
  nh.param(std::string("odom_frame_id"), odom_frame_id_, std::string("odom"));

  // Update OmniTrajectoryCtrl object
  omni_ctrl_ = new OmniTrajectoryCtrl(
      kNFuture, kGoalTolXY, RADIANS(kGoalTolTh), xXvel,
      kPDCtrlKcV, kPDCtrlKcVn, kPDCtrlKcW, kPDCtrlTdV, kPDCtrlTdVn, kPDCtrlTdW,
      kRobModelKpV , kRobModelKpVn , kRobModelKpW,
      kRobModelTauV, kRobModelTauVn, kRobModelTauW);
  omni_ctrl_->Reset();

  /*/ Test load trajectory
  std::string filename =
      "/home/sousarbarb/catkin_ws/src/omnidirectional_trajectory_controller"
      "/input/square_vn-1.0.txt";
  omni_ctrl_->LoadTrajectoryFile(filename);*/

  return true;
}

void OmniTrajectoryCtrlROS::PubCmdVel(double &v_r, double &vn_r, double &w_r) {
  // Initialization
  geometry_msgs::Twist cmd_vel;

  // Linear velocity
  cmd_vel.linear.x = v_r;
  cmd_vel.linear.y = vn_r;
  cmd_vel.linear.z = 0;

  // Angular velocity
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = w_r;

  // Publish data
  pub_cmd_vel_.publish(cmd_vel);

  // Debug
  ROS_DEBUG("[OMNI_CTRL] V_r: %lf \t %lf \t %lf", v_r, vn_r, w_r);
}

void OmniTrajectoryCtrlROS::SubOdom(const nav_msgs::Odometry::ConstPtr &msg) {
  // Process message
  omni_ctrl_->UpdateRobot(
      msg->pose.pose.position.x, msg->pose.pose.position.y,
      tf2::getYaw(msg->pose.pose.orientation),
      msg->twist.twist.linear.x, msg->twist.twist.linear.y,
      msg->twist.twist.angular.z);

  // Debug
  ROS_DEBUG("[OMNI_CTRL] P  : %lf \t %lf \t %lf",
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            tf2::getYaw(msg->pose.pose.orientation) * M_PI / 180);
  ROS_DEBUG("[OMNI_CTRL] V  : %lf \t %lf \t %lf",
            msg->twist.twist.linear.x, msg->twist.twist.linear.y,
            msg->twist.twist.angular.z);
}

bool OmniTrajectoryCtrlROS::SrvLoadTrajectory(
    omnidirectional_trajectory_controller::LoadTrajectory::Request &request,
    omnidirectional_trajectory_controller::LoadTrajectory::Response &response) {
  return omni_ctrl_->LoadTrajectoryFile(request.filename);
}

bool OmniTrajectoryCtrlROS::SrvSetFutureSize(
    omnidirectional_trajectory_controller::SetFutureSize::Request &request,
    omnidirectional_trajectory_controller::SetFutureSize::Response &response) {
  return omni_ctrl_->SetFutureSize(static_cast<uint32_t>(request.size));
}

bool OmniTrajectoryCtrlROS::SrvSetXvel(
    omnidirectional_trajectory_controller::SetXvel::Request &request,
    omnidirectional_trajectory_controller::SetXvel::Response &response) {
  return omni_ctrl_->SetXvel(static_cast<double>(request.xvel));
}

#ifdef ANALYZE_PROCESSING_TIME

bool OmniTrajectoryCtrlROS::SrvAnalyzeTime(
    std_srvs::Trigger::Request &request,
    std_srvs::Trigger::Response &response) {
  return omni_ctrl_->SetAnalyzeTime();
}

#endif

}  // omnidirectional_trajectory_controller
