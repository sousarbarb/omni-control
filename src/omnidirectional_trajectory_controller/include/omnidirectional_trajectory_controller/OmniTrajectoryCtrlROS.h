#ifndef SRC_OMNITRAJECTORYCTRLROS_H
#define SRC_OMNITRAJECTORYCTRLROS_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf2/utils.h>

#include "omnidirectional_trajectory_controller/LoadTrajectory.h"
#include "omnidirectional_trajectory_controller/OmniTrajectoryCtrl.h"

namespace omnidirectional_trajectory_controller {

class OmniTrajectoryCtrlROS {
 private:
  ros::NodeHandle nh_;
  ros::Rate lr_;

  std::string odom_frame_id_;

  ros::Publisher pub_cmd_vel_;
  ros::Subscriber sub_odom_;
  ros::ServiceServer srv_load_trajectory;

  OmniTrajectoryCtrl *omni_ctrl_ = nullptr;

 public:
  explicit OmniTrajectoryCtrlROS();
  virtual ~OmniTrajectoryCtrlROS();
  void Execute();

 private:
  bool ReadParameters();
  void PubCmdVel(double &v_r, double &vn_r, double &w_r);
  void SubOdom(const nav_msgs::Odometry::ConstPtr &msg);
  bool SrvLoadTrajectory(
      omnidirectional_trajectory_controller::LoadTrajectory::Request &request,
      omnidirectional_trajectory_controller::LoadTrajectory::Response
          &response);
};

}

#endif //SRC_OMNITRAJECTORYCTRLROS_H
