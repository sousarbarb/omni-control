#ifndef SRC_OMNITRAJECTORYCTRLROS_H
#define SRC_OMNITRAJECTORYCTRLROS_H

#include <ros/ros.h>

#include "omnidirectional_trajectory_controller/OmniTrajectoryCtrl.h"

namespace omnidirectional_trajectory_controller {

class OmniTrajectoryCtrlROS {
 private:
  ros::NodeHandle &nh_;
  ros::Rate lr_;

  OmniTrajectoryCtrl *omni_ctrl_;

 public:
  explicit OmniTrajectoryCtrlROS(ros::NodeHandle &nh);
  virtual ~OmniTrajectoryCtrlROS();
  void Execute();
  bool ReadParameters();
};

}

#endif //SRC_OMNITRAJECTORYCTRLROS_H
