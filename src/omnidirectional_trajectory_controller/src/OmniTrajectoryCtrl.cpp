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
    uint32_t future_buffer_size, double tol_goal_xy, double tol_goal_th)
    : trajectory_on(false) , i_global(0) , t_global(0), u_ref(0) ,
      goal_tol_xy(tol_goal_xy) , goal_tol_th(tol_goal_th) {
  // Initialize future and least-squares matrices
  InitializeL2Matrices(future_buffer_size);
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

  // - file does not exists
  } else {
    file.close();
    return false;
  }
}

void OmniTrajectoryCtrl::OmniRobotCtrl(double &v_r, double &vn_r,
                                       double &w_r) {
  // Check goal
  if (trajectory_on)
    if (IsGoalReached() && (i_global >= trajectory.rows()))
      trajectory_on = false;

  // Set position references
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
  RotGlobal2Local(rob_p_loc, rob_p);
}

void OmniTrajectoryCtrl::UpdateRobotVelocity(double rob_v_v, double rob_v_vn,
                                             double rob_v_w) {
  // Update robot velocity
  rob_v(0) = rob_v_v;
  rob_v(1) = rob_v_vn;
  rob_v(2) = rob_v_w;
}



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

void OmniTrajectoryCtrl::RotGlobal2Local(Eigen::Vector3d &x_global,
                                         Eigen::Vector3d &x_local) {
  // Initialize rotation matrix
  Eigen::AngleAxis<double> rot_g2l =
      Eigen::AngleAxis<double>(-x_global(2), Eigen::Vector3d::UnitZ());

  // Compute the vector in the global coordinate frame
  x_local = rot_g2l * x_global;
}

}  // omnidirectional_trajectory_controller
