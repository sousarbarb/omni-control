#include "omnidirectional_trajectory_controller/OmniTrajectoryCtrl.h"

#include <iostream>
#include <fstream>
#include <vector>

namespace omnidirectional_trajectory_controller {

// Local private functions

double Dist(const double x, const double y) {
  return std::sqrt( std::pow(x, 2) + std::pow(y, 2) );
}

double Dist(double &x1, double &y1, double &x2, double &y2) {
  return Dist(x1-x2, y1-y2);
}

double NormAngle(const double angle) {
  double norm_angle = angle;

  while (norm_angle >= M_PI)
    norm_angle -= 2*M_PI;
  while (norm_angle < -M_PI)
    norm_angle += 2*M_PI;

  return norm_angle;
}



// Public methods

OmniTrajectoryCtrl::OmniTrajectoryCtrl(
    uint32_t future_buffer_size, double tol_goal_xy, double tol_goal_th,
    double xvel,
    double pd_kc_v, double pd_kc_vn, double pd_kc_w,
    double pd_td_v, double pd_td_vn, double pd_td_w,
    double model_kp_v , double model_kp_vn , double model_kp_w,
    double model_tau_v, double model_tau_vn, double model_tau_w)
    : trajectory_on(false) , i_global(0) , t_global(0), u_ref(0) ,
      goal_tol_xy(tol_goal_xy) , goal_tol_th(tol_goal_th) , x_vel(xvel) {
  // Initialize future and least-squares matrices
  InitializeL2Matrices(future_buffer_size);

  // Initialize the gains for the pd controllers
  pd_kc.diagonal() << pd_kc_v , pd_kc_vn , pd_kc_w;
  pd_td.diagonal() << pd_td_v , pd_td_vn , pd_td_w;

  // Initialize the model of the robot's velocity
  model_1_kp.diagonal() << 1 / model_kp_v , 1 / model_kp_vn , 1 / model_kp_w;
  model_tau_kp.diagonal() << model_tau_v  / model_kp_v ,
                             model_tau_vn / model_kp_vn ,
                             model_tau_w  / model_kp_w;

  // Debug
#ifdef DEBUG_OMNITRAJECTORYCTRL
  std::cout << std::endl << "[OmniTrajectoryCtrl]" << std::endl;
  std::cout << "PD Ctrl Kc Diagonal Matrix:" << std::endl
            << pd_kc.diagonal() << std::endl;
  std::cout << "PD Ctrl Td Diagonal Matrix:" << std::endl
            << pd_td.diagonal() << std::endl;
  std::cout << "Model Diagonal 1 / Kp Matrix:" << std::endl
            << model_1_kp.diagonal() << std::endl;
  std::cout << "Model Diagonal Tau / Kp Matrix:" << std::endl
            << model_tau_kp.diagonal() << std::endl;
#endif
}

bool OmniTrajectoryCtrl::LoadTrajectoryFile(std::string &filename) {
  // Initialization
  std::ifstream file;

  // Open file
  file.open(filename, std::ios::in);

  // Process file
  // - file exists
  if (file.good()) {
    std::string line;
    uint32_t line_index = 0;
    uint32_t num_lines = 0;
    uint32_t trajectory_index = 0;

    // Count number of lines
    while (getline(file, line))
      num_lines++;

    // Point again to the beginning of the file
    file.clear();
    file.seekg(0, std::ios::beg);

    // Resize trajectory
    trajectory.resize(num_lines, Eigen::NoChange_t::NoChange);

    // Debug
#ifdef DEBUG_OMNITRAJECTORYCTRL
    std::cout << std::endl << "[LoadTrajectoryFile]" << std::endl;
#endif

    // Process line
    while (getline(file, line)) {
      // - remove \n character
      if ((line[line.length()-1] == '\n') || (line[line.length()-1] == '\r'))
        line.erase(line.length()-1, std::string::npos);
      else if (line[line.length()-2] == '\r')
        line.erase(line.length()-2, std::string::npos);

      // - check commas in the string
      size_t line_len = line.length();
      std::vector<size_t> comma_indexes;
      for (uint32_t i=0; i < line_len; i++)
        if (line[i] == ',')
          comma_indexes.push_back(i);

      // - convert to float
      try {
        if (comma_indexes.size() < 2)
          throw line;

        // - convert string to double
        trajectory(trajectory_index,0) =
            std::stod(line.substr(0, comma_indexes[0]) );
        trajectory(trajectory_index,1) =
            std::stod(line.substr(comma_indexes[0] + 1,
                                  comma_indexes[1] - comma_indexes[0]) );
        trajectory(trajectory_index,2) =
            std::stod(line.substr(
                comma_indexes[1] + 1,
                comma_indexes.size() == 2?
                    std::string::npos : comma_indexes[2] - comma_indexes[1]));
        // - debug
#ifdef DEBUG_OMNITRAJECTORYCTRL
        std::cout << line << "   [ "
                  << trajectory(trajectory_index,0) << " , "
                  << trajectory(trajectory_index,1) << " , "
                  << trajectory(trajectory_index,2) << " ]" << std::endl;
#endif

        trajectory_index++;
      } catch (std::string &e) {
        ROS_WARN("Line [%u] %s did not contain enough data to define a "
                 "trajectory point", line_index, line.c_str());
      } catch (...) {
        ROS_WARN("Error when processing line [%u] %s",
                 line_index, line.c_str());
      }

      // - update line index
      line_index++;
    }

    // Set trajectory boolean true (new trajectory was loaded)
    trajectory_on = true;
    i_global = 0;
    t_global = 0;
    u_ref = 0;
    InitializeBuffers(i_global);

    // Update filename
    trajectory_filename = filename;
    ROS_INFO("Trajectory loaded (filename: %s)", trajectory_filename.c_str());

#ifdef SAVE_DATA_CSV
    // Clear history
    ClearHistory();
#endif

  // - file does not exists
  } else {
    file.close();
    return false;
  }
}

void OmniTrajectoryCtrl::OmniRobotCtrl(double &v_r, double &vn_r,
                                       double &w_r) {
  // Check goal
  if (trajectory_on) {
    if (IsGoalReached() && (i_global >= trajectory.rows())) {
      ROS_INFO("Goal (x, y, th = %f, %f, %f) reached!",
               trajectory.row(trajectory.rows()-1)(0),
               trajectory.row(trajectory.rows()-1)(1),
               DEGREES(trajectory.row(trajectory.rows()-1)(2)));

      trajectory_on = false;

#ifdef SAVE_DATA_CSV
      SaveHistory();
#endif
    }
  }

#ifdef  ANALYZE_PROCESSING_TIME
  // Start high resolution clock
  auto t_start = std::chrono::high_resolution_clock::now();
#endif

  // Set position references
  if (trajectory_on)
    SetPositionControllerFFReferences();

  // Position control
  RobotPosControllers();

  // Set reference velocity of the robot
  v_r  = rob_v_r(0);
  vn_r = rob_v_r(1);
  w_r  = rob_v_r(2);

#ifdef ANALYZE_PROCESSING_TIME
  // Stop high resolution clock
  auto t_stop = std::chrono::high_resolution_clock::now();

  // Compute processing time in milliseconds
  std::chrono::duration<double, std::milli> t_processing_time =
      t_stop - t_start;

  // Save processing time
  if (time_analyze) {
    time_trajectory_on_vec.push_back(trajectory_on);
    time_processing_vec.push_back(t_processing_time.count());
  }
#endif

#ifdef SAVE_DATA_CSV
  // Update history (only when following a trajectory)
  if (trajectory_on)
    UpdateHistory();
#endif
}

void OmniTrajectoryCtrl::Reset() {
  for (uint32_t i=0; i<3; i++) {
    rob_p_loc(i) = 0;
    rob_p_loc_r(i) = 0;
    rob_p_loc_e(i) = 0;
    rob_p_loc_eprev(i) = 0;
    rob_p_loc_ederiv(i) = 0;
    rob_p_loc_r_1d(i) = 0;
    rob_p_loc_r_2d(i) = 0;
    rob_p(i) = 0;
    rob_p_r(i) = 0;
    rob_p_r_1d(i) = 0;
    rob_p_r_2d(i) = 0;
    rob_v(i) = 0;
    rob_v_r(i) = 0;
    rob_v_r_pd(i) = 0;
    rob_v_r_ff(i) = 0;
  }
  trajectory_on = false;
  i_global = 0;
  t_global = 0;
  u_ref = 0;
}

void OmniTrajectoryCtrl::UpdateRobot(double rob_p_x, double rob_p_y,
                                     double rob_p_th, double rob_v_v,
                                     double rob_v_vn, double rob_v_w) {
  UpdateRobotPosition(rob_p_x, rob_p_y , rob_p_th);
  UpdateRobotVelocity(rob_v_v, rob_v_vn, rob_v_w);
}

void OmniTrajectoryCtrl::UpdateRobotPosition(double rob_p_x, double rob_p_y,
                                             double rob_p_th) {
  // Update robot position (global coordinate frame)
  rob_p(0) = rob_p_x;
  rob_p(1) = rob_p_y;
  rob_p(2) = rob_p_th;

  // Compute position in the robot's local coordinate frame
  RotGlobal2Local(rob_p, rob_p_loc);
}

void OmniTrajectoryCtrl::UpdateRobotVelocity(double rob_v_v, double rob_v_vn,
                                             double rob_v_w) {
  // Update robot velocity
  rob_v(0) = rob_v_v;
  rob_v(1) = rob_v_vn;
  rob_v(2) = rob_v_w;
}

bool OmniTrajectoryCtrl::SetFutureSize(uint32_t future_buffer_size) {
  // Successful update
  if (!trajectory_on) {
    InitializeL2Matrices(future_buffer_size);
    ROS_INFO("Future size updated (size: %u)", future_buffer_size);

    return true;

  // Unsuccessful update of the future buffer's size
  } else {
    ROS_ERROR("Future size NOT UPDATED (size: %u)", future_buffer_size);

    return false;
  }
}

bool OmniTrajectoryCtrl::SetXvel(double xvel) {
  // Successful update
  if (!trajectory_on) {
    x_vel = xvel;
    ROS_INFO("Xvel updated (value: %f)", x_vel);

    return true;

  // Unsuccessful update of xvel
  } else {
    ROS_ERROR("Xvel NOT UPDATED (value: %f)", x_vel);

    return false;
  }
}

#ifdef ANALYZE_PROCESSING_TIME

bool OmniTrajectoryCtrl::SetAnalyzeTime() {
  if (!time_analyze) {
    ROS_INFO("Started analysis of processing time");

    time_analyze = true;
  } else {
    ROS_INFO("Finished analysis of processing time");

    SaveTimeProcessingAnalysis();

    time_analyze = false;
  }

  return  true;
}

#endif



// Private methods

void OmniTrajectoryCtrl::InitializeBuffers(uint32_t &index_0) {
  // Initialization
  uint32_t index_last = 0, future_buffer_size = future_buffer.rows(),
           trajectory_size = trajectory.rows();
  Eigen::Vector3d p_0, v_0;
  Eigen::Matrix<double,Eigen::Dynamic,3> future_buffer_tmp;

  // Initialize initial position and velocity
  // - initial position (1st restriction):
  if (index_0 < trajectory_size)
    p_0 = trajectory.row(index_0).transpose();
  else
    p_0 = trajectory.row(trajectory_size-1).transpose();
  // - initial velocity (2nd restriction):
  if (index_0 + 1 < trajectory_size) {
    v_0 = trajectory.row(index_0 + 1).transpose() -
          trajectory.row(index_0).transpose();
    v_0(2) = NormAngle(v_0(2));
  } else {
    for (uint32_t i=0; i<3; i++)
      v_0(i) = 0;
  }

  // Initialize the future/horizon buffers (the original and the temporary ones)
  future_buffer_tmp.resize(future_buffer_size, Eigen::NoChange_t::NoChange);
  for (uint32_t i=0; i < future_buffer_size; i++, index_last++) {
    // - index outside the trajectory
    if (i + index_0 >= trajectory_size)
      break;

    // - buffer
    future_buffer.row(i) = trajectory.row(i + index_0);
    // - auxiliar buffer
    future_buffer_tmp.row(i) =
        trajectory.row(i + index_0) - p_0.transpose() - i * v_0.transpose();
  }
  // - retain final pose of the trajectory
  for (uint32_t i=index_last; i < future_buffer_size; i++) {
    // - buffer
    future_buffer.row(i) = trajectory.row(trajectory_size-1);
    // - auxiliar buffer
    future_buffer_tmp.row(i) =
        trajectory.row(trajectory_size-1) - p_0.transpose() - i*v_0.transpose();
  }
  // - unwrap the orientation
  for (uint32_t i=1; i < future_buffer_size; i++) {
    future_buffer(i,2) =
        future_buffer(i-1,2) +
        NormAngle( future_buffer(i,2) - future_buffer(i-1,2) );
    future_buffer_tmp(i,2) =
        future_buffer_tmp(i-1,2) +
        NormAngle( future_buffer_tmp(i,2) - future_buffer_tmp(i-1,2) );
  }

  // Estimate approximations of velocity and acceleration
  // - velocity coefficients (1st-order curve / line)
  Eigen::Matrix<double,2,3> future_approx_coeff_vel;
  future_approx_coeff_vel = future_approx_l2_vel * future_buffer;
  future_approx_coeff_1d = future_approx_coeff_vel.row(1);
  // - acceleration coefficients (2nd-order curve restricted in p_0 and v_0)
  future_approx_coeff_2d = future_approx_l2_acc * future_buffer_tmp;

  // Debug
#ifdef DEBUG_OMNITRAJECTORYCTRL
  std::cout << std::endl << "[InitializeBuffers]" << std::endl;
  std::cout << "Future buffer:" << std::endl
            << future_buffer << std::endl;
  std::cout << "Future buffer (auxiliar variable):" << std::endl
            << future_buffer_tmp << std::endl;
  std::cout << "Approximation coefficients - velocity:" << std::endl
            << future_approx_coeff_1d << std::endl;
  std::cout << "Approximation coefficients - acceleration:" << std::endl
            << future_approx_coeff_2d << std::endl;
#endif
}

void OmniTrajectoryCtrl::InitializeL2Matrices(
    uint32_t future_buffer_size) {
  // Initialization
  future_buffer.resize(future_buffer_size, Eigen::NoChange_t::NoChange);
  future_approx_l2_vel.resize(Eigen::NoChange_t::NoChange, future_buffer_size);
  future_approx_l2_acc.resize(Eigen::NoChange_t::NoChange, future_buffer_size);

  // Least-squares with restrictions: acceleration (in terms of p0 and v0)
  // - initialize matrix that relates the curve coefficients to its points
  Eigen::Matrix<double,Eigen::Dynamic,1> future_approx_mat_acc;
  future_approx_mat_acc.resize(future_buffer_size, Eigen::NoChange_t::NoChange);
  for (uint32_t i=0; i < future_buffer_size; i++)
    future_approx_mat_acc(i,0) = std::pow(i, 2) / 2;
  // - Monroe-Penrose inverse
  future_approx_l2_acc =
      ( future_approx_mat_acc.transpose() * future_approx_mat_acc).inverse() *
      future_approx_mat_acc.transpose();

  // Least-squares without restrictions: velocity
  // - initialize matrix that relates the curve coefficients to its points
  Eigen::Matrix<double,Eigen::Dynamic,2> future_approx_mat_vel;
  future_approx_mat_vel.resize(future_buffer_size, Eigen::NoChange_t::NoChange);
  for (uint32_t j=0; j < 2; j++)
    for (uint32_t i=0; i < future_buffer_size; i++)
      future_approx_mat_vel(i, j) = std::pow(i,j);
  // - Monroe-Penrose inverse
  future_approx_l2_vel =
      ( future_approx_mat_vel.transpose() * future_approx_mat_vel).inverse() *
      future_approx_mat_vel.transpose();

  // Debug
#ifdef DEBUG_OMNITRAJECTORYCTRL
  std::cout << std::endl << "[InitializeL2Matrices]" << std::endl;
  std::cout << "MatVel:" << std::endl
            << future_approx_mat_vel << std::endl;
  std::cout << "MatVelL2:" << std::endl
            << future_approx_l2_vel  << std::endl;
  std::cout << "MatAcc:" << std::endl
            << future_approx_mat_acc << std::endl;
  std::cout << "MatAccL2:" << std::endl
            << future_approx_l2_acc  << std::endl << std::endl;
#endif
}

bool OmniTrajectoryCtrl::IsGoalReached() {
  if (trajectory_on && (trajectory.rows() > 0)) {
    Eigen::Vector3d goal = trajectory.row(trajectory.rows()-1).transpose();

    if ((Dist(goal(0) - rob_p(0), goal(1) - rob_p(1)) <= goal_tol_xy) &&
        (std::fabs(NormAngle(goal(2) - rob_p(2))) <= goal_tol_th))
      return true;
    else
      return false;
  } else {
    return true;
  }
}

void OmniTrajectoryCtrl::RobotPosControllers() {
  // Global > Local coordinate frame
  RotGlobal2Local(rob_p_r   , rob_p_loc_r   );
  RotGlobal2Local(rob_p_r_1d, rob_p_loc_r_1d);
  RotGlobal2Local(rob_p_r_2d, rob_p_loc_r_2d);

  // PD controllers
  RobotPosPDControllers();

  // FF controllers
  RobotPosFFControllers();

  // Velocity of the robot
  rob_v_r = rob_v_r_pd + rob_v_r_ff;
}

void OmniTrajectoryCtrl::RobotPosFFControllers() {
  // Robot velocity
  rob_v_r_ff = model_tau_kp * rob_p_loc_r_2d + model_1_kp * rob_p_loc_r_1d;
}

void OmniTrajectoryCtrl::RobotPosPDControllers() {
  // Errors (proportional and derivative)
  rob_p_loc_eprev  = rob_p_loc_e;
  rob_p_loc_e      = rob_p_loc_r - rob_p_loc;
  rob_p_loc_ederiv = rob_p_loc_e - rob_p_loc_eprev;
  // - special case of orientation
  rob_p_loc_e(2)      = NormAngle(rob_p_loc_e(2));
  rob_p_loc_ederiv(2) = NormAngle(rob_p_loc_ederiv(2));
  // - frequency multiplication
  rob_p_loc_ederiv = rob_p_loc_ederiv * kFreqPosCtrl;

  // Robot velocity
  rob_v_r_pd = pd_kc * ( rob_p_loc_e + pd_td * rob_p_loc_ederiv );
}

void OmniTrajectoryCtrl::RotGlobal2Local(Eigen::Vector3d &x_global,
                                         Eigen::Vector3d &x_local) {
  // Initialize rotation matrix
  Eigen::AngleAxis<double> rot_g2l =
      Eigen::AngleAxis<double>(-rob_p(2), Eigen::Vector3d::UnitZ());

  // Compute the vector in the global coordinate frame
  x_local = rot_g2l * x_global;
}

void OmniTrajectoryCtrl::SetPositionControllerFFReferences() {
  // Initialization
  double u_0, u_f;

  // Global time
  t_global = t_global + (1.0 / kFreqPosCtrl);
  // Reference indexes
  u_ref = t_global * x_vel * kFreqPosCtrl - static_cast<double>(i_global);
  while (u_ref >= 1) {
    i_global++;
    u_ref = t_global * x_vel * kFreqPosCtrl - static_cast<double>(i_global);
  }
  InitializeBuffers(i_global);

  // Interpolation
  u_0 = std::floor(u_ref) >= 0? std::floor(u_ref) : 0;
  u_f = u_0 + 1;

  // Set reference
  // - position (interpolation):
  rob_p_r =
      future_buffer.row(u_0).transpose() + (
      future_buffer.row(u_f).transpose() - future_buffer.row(u_0).transpose()) *
          (u_ref - u_0);
  // - first derivative:
  rob_p_r_1d = future_approx_coeff_1d.transpose();
  rob_p_r_1d = rob_p_r_1d * x_vel * kFreqPosCtrl;
  // - second derivative:
  rob_p_r_2d = future_approx_coeff_2d.transpose();
  rob_p_r_2d = rob_p_r_2d * x_vel * kFreqPosCtrl;
}

#ifdef SAVE_DATA_CSV

void OmniTrajectoryCtrl::ClearHistory() {
  t_global_vec.clear();
  trajectory_on_vec.clear();
  rob_v_vec.clear();
  rob_v_r_vec.clear();
  rob_v_r_pd_vec.clear();
  rob_v_r_ff_vec.clear();
  rob_p_loc_vec.clear();
  rob_p_loc_r_vec.clear();
  rob_p_loc_e_vec.clear();
  rob_p_loc_ederiv_vec.clear();
  rob_p_loc_r_1d_vec.clear();
  rob_p_loc_r_2d_vec.clear();
  rob_p_vec.clear();
  rob_p_r_vec.clear();
  rob_p_r_1d_vec.clear();
  rob_p_r_2d_vec.clear();
  x_vel_vec.clear();
  i_global_vec.clear();
  u_ref_vec.clear();
}

void OmniTrajectoryCtrl::SaveHistory() {
  std::ostringstream filename;
  filename << SAVE_DATA_CSV << index_data << ".csv";

  std::ofstream file;
  file.open(filename.str(), std::ios::out | std::ios::trunc);

  // Metadata
  file << "Tctrl:," << kFreqPosCtrl << "," << std::endl;
  file << std::endl;
  file << "ROBOT:,(v),(vn),(w)," << std::endl;
  file << "kp:," << kRobModelKpV << "," << kRobModelKpVn << ","
                 << kRobModelKpW << "," << std::endl;
  file << "tau:," << kRobModelTauV << "," << kRobModelTauVn << ","
                  << kRobModelTauW << "," << std::endl;
  file << "kc:," << pd_kc.diagonal()(0) << "," << pd_kc.diagonal()(1) << ","
                 << pd_kc.diagonal()(2) << "," << std::endl;
  file << "td:," << pd_td.diagonal()(0) << "," << pd_td.diagonal()(1) << ","
                 << pd_td.diagonal()(2) << "," << std::endl;
  file << std::endl;
  file << "Filename:,Nfuture:," << std::endl;
  file << trajectory_filename << "," << future_buffer.rows() << ","
       << std::endl << std::endl;

  // Header
  file << "time (s),traj (bool),"
       << "v_rob (m/s),vn_rob (m/s),w_rob (rad/s),"
       << "v_r_rob (m/s),vn_r_rob (m/s),w_r_rob (rad/s),"
       << "v_pd_r_rob (m/s),vn_pd_r_rob (m/s),w_pd_r_rob (rad/s),"
       << "v_ff_r_rob (m/s),vn_ff_r_rob (m/s),w_ff_r_rob (rad/s),"
       << "px_loc (m),py_loc (m),pth_loc (rad),"
       << "px_r_loc (m),py_r_loc (m),pth_r_loc (rad),"
       << "px_e_loc (m),py_e_loc (m),pth_e_loc (rad),"
       << "px_e_1deriv_loc,py_e_1deriv_loc,pth_e_1deriv_loc,"
       << "px_r_1deriv_loc,py_r_1deriv_loc,pth_r_1deriv_loc,"
       << "px_r_2deriv_loc,py_r_2deriv_loc,pth_r_2deriv_loc,"
       << "px (m),py (m),pth (rad),px_r (m),py_r (m),pth_r (rad),"
       << "px_r_1deriv,py_r_1deriv,pth_r_1deriv,"
       << "px_r_2deriv,py_r_2deriv,pth_r_2deriv,"
       << "xvel,i_global,u_ref,";

  // Data
  for (size_t i = 0; i < t_global_vec.size(); i++) {
    file << t_global_vec[i] << ","
         << trajectory_on_vec[i] << ","
         << rob_v_vec[i](0) << "," << rob_v_vec[i](1) << ","
         << rob_v_vec[i](2) << ","
         << rob_v_r_vec[i](0) << "," << rob_v_r_vec[i](1) << ","
         << rob_v_r_vec[i](2) << ","
         << rob_v_r_pd_vec[i](0) << "," << rob_v_r_pd_vec[i](1) << ","
         << rob_v_r_pd_vec[i](2) << ","
         << rob_v_r_ff_vec[i](0) << "," << rob_v_r_ff_vec[i](1) << ","
         << rob_v_r_ff_vec[i](2) << ","
         << rob_p_loc_vec[i](0) << "," << rob_p_loc_vec[i](1) << ","
         << rob_p_loc_vec[i](2) << ","
         << rob_p_loc_r_vec[i](0) << "," << rob_p_loc_r_vec[i](1) << ","
         << rob_p_loc_r_vec[i](2) << ","
         << rob_p_loc_e_vec[i](0) << "," << rob_p_loc_e_vec[i](1) << ","
         << rob_p_loc_e_vec[i](2) << ","
         << rob_p_loc_ederiv_vec[i](0) <<","<< rob_p_loc_ederiv_vec[i](1) << ","
         << rob_p_loc_ederiv_vec[i](2) <<","
         << rob_p_loc_r_1d_vec[i](0) << "," << rob_p_loc_r_1d_vec[i](1) << ","
         << rob_p_loc_r_1d_vec[i](2) << ","
         << rob_p_loc_r_2d_vec[i](0) << "," << rob_p_loc_r_2d_vec[i](1) << ","
         << rob_p_loc_r_2d_vec[i](2) << ","
         << rob_p_vec[i](0) << "," << rob_p_vec[i](1) << ","
         << rob_p_vec[i](2) << ","
         << rob_p_r_vec[i](0) << "," << rob_p_r_vec[i](1) << ","
         << rob_p_r_vec[i](2) << ","
         << rob_p_r_1d_vec[i](0) << "," << rob_p_r_1d_vec[i](1) << ","
         << rob_p_r_1d_vec[i](2) << ","
         << rob_p_r_2d_vec[i](0) << "," << rob_p_r_2d_vec[i](1) << ","
         << rob_p_r_2d_vec[i](2) << ","
         << x_vel_vec[i] << ","
         << i_global_vec[i] << ","
         << u_ref_vec[i] << ","
         << std::endl;
  }

  // Save file
  file.close();

  // Update index of data (to not overwrite files on top of each other)
  index_data++;
}

void OmniTrajectoryCtrl::UpdateHistory() {
  t_global_vec.push_back(t_global);
  trajectory_on_vec.push_back(trajectory_on);
  rob_v_vec.push_back(rob_v);
  rob_v_r_vec.push_back(rob_v_r);
  rob_v_r_pd_vec.push_back(rob_v_r_pd);
  rob_v_r_ff_vec.push_back(rob_v_r_ff);
  rob_p_loc_vec.push_back(rob_p_loc);
  rob_p_loc_r_vec.push_back(rob_p_loc_r);
  rob_p_loc_e_vec.push_back(rob_p_loc_e);
  rob_p_loc_ederiv_vec.push_back(rob_p_loc_ederiv);
  rob_p_loc_r_1d_vec.push_back(rob_p_loc_r_1d);
  rob_p_loc_r_2d_vec.push_back(rob_p_loc_r_2d);
  rob_p_vec.push_back(rob_p);
  rob_p_r_vec.push_back(rob_p_r);
  rob_p_r_1d_vec.push_back(rob_p_r_1d);
  rob_p_r_2d_vec.push_back(rob_p_r_2d);
  x_vel_vec.push_back(x_vel);
  i_global_vec.push_back(i_global);
  u_ref_vec.push_back(u_ref);
}

#endif

#ifdef ANALYZE_PROCESSING_TIME

void OmniTrajectoryCtrl::SaveTimeProcessingAnalysis() {
  std::ostringstream filename;
  filename << ANALYZE_PROCESSING_TIME << future_buffer.rows() << ".csv";

  std::ofstream file;
  file.open(filename.str(), std::ios::out | std::ios::trunc);

  // Metadata
  file << "Nfuture:," << std::endl;
  file << future_buffer.rows() << "," << std::endl << std::endl;

  // Header
  file << "traj (bool),delta_time (ms)," << std::endl;

  // Data
  for (size_t i = 0; i < time_trajectory_on_vec.size(); i++) {
    file << time_trajectory_on_vec[i] << ","
         << time_processing_vec[i] << ","
         << std::endl;
  }

  // Save file
  file.close();

  // Clear vectors
  time_trajectory_on_vec.clear();
  time_processing_vec.clear();
}

#endif

}  // omnidirectional_trajectory_controller
