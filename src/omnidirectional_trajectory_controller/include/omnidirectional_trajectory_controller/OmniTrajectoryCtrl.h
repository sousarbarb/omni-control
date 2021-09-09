#ifndef SRC_OMNITRAJECTORYCTRL_H
#define SRC_OMNITRAJECTORYCTRL_H

#include <ros/ros.h>
#include <Eigen/Eigen>

#define DEBUG_OMNITRAJECTORYCTRL

namespace omnidirectional_trajectory_controller {

const double kFreqPosCtrl = 25;
// Robot (parameters, controllers, ...)
const double kRobotWhD = 0.049283 * 2;  // diameter of the wheels (m)
const double kRobotL   = 0.191367;      // distance between center - wheels (m)
const double kMotEncRes = 12 * 1024;    // resolution of the encoders + gear
const double kPDCtrlKpV  = 1;           // model V/Vn/W = f (v1, v2, v3)
const double kPDCtrlKpVn = 1;
const double kPDCtrlKpW  = 1;
const double kPDCtrlTauV  = 0.129073;
const double kPDCtrlTauVn = 0.128070;
const double kPDCtrlTauW  = 0.099488;
const double kPDCtrlTsett = 0.8;            // settling time desired for the PD
const double kPDCtrlKcV  = 4.41720255759;   // PD ctrl gains
const double kPDCtrlKcVn = 4.38287737598;
const double kPDCtrlKcW  = 3.40472947905;
const double kPDCtrlTdV  = 0.0696893946987;
const double kPDCtrlTdVn = 0.0679164050154;
const double kPDCtrlTdW  = 0.00236792968417;
// Trajectory
const uint32_t kNFuture = 10;     // size of the buffer to compute the ff
// derivatives
const double kGoalTolXY = 0.05;   // goal tolerances
const double kGoalTolTh = 2.5;

struct OmniTrajectoryCtrl {
 public:
  // Robot state
  // - "local" coordinate frame
  Eigen::Vector3d rob_p_loc, rob_p_loc_r, rob_p_loc_e, rob_p_loc_eprev,
                  rob_p_loc_ederiv, rob_p_loc_r_1d, rob_p_loc_r_2d;
  Eigen::Vector3d rob_p, rob_p_r, rob_p_r_1d, rob_p_r_2d;
  Eigen::Vector3d rob_v, rob_v_r, rob_v_r_pd, rob_v_r_ff;

  // Trajectory
  bool trajectory_on;
  uint32_t i_global;
  double t_global, u_ref;
  Eigen::Matrix<double,Eigen::Dynamic,3> trajectory;
  Eigen::Matrix<double,Eigen::Dynamic,3> future_buffer;
  Eigen::Matrix<double,1,3> future_approx_coeff_1d, future_approx_coeff_2d;
  Eigen::Matrix<double,2,Eigen::Dynamic> future_approx_l2_vel;
  Eigen::Matrix<double,1,Eigen::Dynamic> future_approx_l2_acc;

  // Parameters
  double goal_tol_xy, goal_tol_th;

 public:
  explicit OmniTrajectoryCtrl(
      uint32_t future_buffer_size, double tol_goal_xy, double tol_goal_th);
  bool LoadTrajectoryFile(std::string &filename);
  void OmniRobotCtrl(double &v_r, double &vn_r, double &w_r);
  void UpdateRobot(double rob_p_x, double rob_p_y , double rob_p_th,
                   double rob_v_v, double rob_v_vn, double rob_v_w);
  void UpdateRobotPosition(double rob_p_x, double rob_p_y , double rob_p_th);
  void UpdateRobotVelocity(double rob_v_v, double rob_v_vn, double rob_v_w);

 private:
  void InitializeBuffers(uint32_t &index_0);
  void InitializeL2Matrices(uint32_t future_buffer_size);
  bool IsGoalReached();
  static void RotGlobal2Local(Eigen::Vector3d &x_global,
                              Eigen::Vector3d &x_local);
};

}

#endif //SRC_OMNITRAJECTORYCTRL_H
