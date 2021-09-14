#ifndef SRC_OMNITRAJECTORYCTRL_H
#define SRC_OMNITRAJECTORYCTRL_H

#include <ros/ros.h>
#include <Eigen/Eigen>

//#define DEBUG_OMNITRAJECTORYCTRL
#define SAVE_DATA_CSV \
    "/home/sousarbarb/catkin_ws/log/5dpo_article/omnictrl_log_"

namespace omnidirectional_trajectory_controller {

const double kFreqPosCtrl = 25;
// Robot (parameters, controllers, ...)
const double kRobotWhD = 0.049283 * 2;  // diameter of the wheels (m)
const double kRobotL   = 0.191367;      // distance between center - wheels (m)
const double kMotEncRes = 12 * 1024;    // resolution of the encoders + gear
const double kRobModelKpV  = 1.019649723; // model V/Vn/W = f (v1, v2, v3)
const double kRobModelKpVn = 1.031417953;
const double kRobModelKpW  = 0.978734897;
const double kRobModelTauV  = 0.121202798;
const double kRobModelTauVn = 0.105652003;
const double kRobModelTauW  = 0.090826489;
const double kPDCtrlTsett = 0.7;            // settling time desired for the PD
const double kPDCtrlKcV  = 5.31321566905;   // PD ctrl gains
const double kPDCtrlKcVn = 4.57866482846;
const double kPDCtrlKcW  = 4.14804307364;
const double kPDCtrlTdV  = 0.0744844569897;
const double kPDCtrlTdVn = 0.0473159100918;
const double kPDCtrlTdW  = 0.0127519342197;
// Trajectory
const uint32_t kNFuture = 10;     // size of the buffer to compute the ff
const double xXvel = 0.5;         // multiplying factor for the trajectory's vn
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
  std::string trajectory_filename;
  Eigen::MatrixX3d trajectory;
  Eigen::MatrixX3d future_buffer;
  Eigen::RowVector3d future_approx_coeff_1d, future_approx_coeff_2d;
  Eigen::Matrix2Xd future_approx_l2_vel;
  Eigen::RowVectorXd future_approx_l2_acc;

  // Parameters
  double goal_tol_xy, goal_tol_th;
  double x_vel;
  Eigen::DiagonalMatrix<double, 3> pd_kc, pd_td;
  Eigen::DiagonalMatrix<double, 3> model_1_kp, model_tau_kp;

#ifdef SAVE_DATA_CSV
  // History
  int index_data = 0;
  std::vector<double> t_global_vec;
  std::vector<bool> trajectory_on_vec;
  std::vector<Eigen::Vector3d> rob_v_vec, rob_v_r_vec,
                               rob_v_r_pd_vec, rob_v_r_ff_vec;
  std::vector<Eigen::Vector3d> rob_p_loc_vec, rob_p_loc_r_vec,
                               rob_p_loc_e_vec, rob_p_loc_ederiv_vec,
                               rob_p_loc_r_1d_vec, rob_p_loc_r_2d_vec;
  std::vector<Eigen::Vector3d> rob_p_vec, rob_p_r_vec,
                               rob_p_r_1d_vec, rob_p_r_2d_vec;
  std::vector<double> x_vel_vec;
  std::vector<uint32_t> i_global_vec;
  std::vector<double> u_ref_vec;
#endif

public:
  explicit OmniTrajectoryCtrl(
      uint32_t future_buffer_size, double tol_goal_xy, double tol_goal_th,
      double xvel,
      double pd_kc_v, double pd_kc_vn, double pd_kc_w,
      double pd_td_v, double pd_td_vn, double pd_td_w,
      double model_kp_v , double model_kp_vn , double model_kp_w,
      double model_tau_v, double model_tau_vn, double model_tau_w);
  bool LoadTrajectoryFile(std::string &filename);
  void OmniRobotCtrl(double &v_r, double &vn_r, double &w_r);
  void Reset();
  void UpdateRobot(double rob_p_x, double rob_p_y , double rob_p_th,
                   double rob_v_v, double rob_v_vn, double rob_v_w);
  void UpdateRobotPosition(double rob_p_x, double rob_p_y , double rob_p_th);
  void UpdateRobotVelocity(double rob_v_v, double rob_v_vn, double rob_v_w);
  bool SetFutureSize(uint32_t future_buffer_size);
  bool SetXvel(double xvel);

 private:
  void InitializeBuffers(uint32_t &index_0);
  void InitializeL2Matrices(uint32_t future_buffer_size);
  bool IsGoalReached();
  void RobotPosControllers();
  void RobotPosFFControllers();
  void RobotPosPDControllers();
  void RotGlobal2Local(Eigen::Vector3d &x_global, Eigen::Vector3d &x_local);
  void SetPositionControllerFFReferences();

#ifdef SAVE_DATA_CSV
    void ClearHistory();
    void SaveHistory();
    void UpdateHistory();
#endif
};

}

#endif //SRC_OMNITRAJECTORYCTRL_H
