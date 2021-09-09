#include "omnidirectional_trajectory_controller/OmniTrajectoryCtrl.h"

#include <iostream>
#include <fstream>
#include <vector>

namespace omnidirectional_trajectory_controller {

// Public methods

OmniTrajectoryCtrl::OmniTrajectoryCtrl(uint32_t future_buffer_size)
    : trajectory_on(false) , trajectory_numpoints(0) , i_global(0) ,
      t_global(0), u_ref(0) {
  // Initialize future and least-squares matrices
  InitializeL2Matrices(future_buffer_size);
}

bool OmniTrajectoryCtrl::LoadTrajectoryFile(const std::string filename) {
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
    std::cout << "[LoadTrajectoryFile]" << std::endl;
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
      for (int i=0; i < line_len; i++)
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

  // - file does not exists
  } else {
    file.close();
    return false;
  }
}

void OmniTrajectoryCtrl::OmniRobotCtrl(double &v_r, double &vn_r,
                                       double &w_r) {

}

void OmniTrajectoryCtrl::UpdateRobotInfo(double rob_p_x, double rob_p_y,
                                         double rob_p_th, double rob_v_v,
                                         double rob_v_vn, double rob_v_w) {
  // Update robot info
  // - position (global coordinate frame):
  rob_p(0) = rob_p_x;
  rob_p(1) = rob_p_y;
  rob_p(2) = rob_p_th;
  // - velocity:
  rob_v(0) = rob_v_v;
  rob_v(1) = rob_v_vn;
  rob_v(2) = rob_v_w;

  // Compute position in the robot's local coordinate frame
  RotGlobal2Local(rob_p_loc, rob_p);
}

// Private methods

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
  for (int i=0; i < future_buffer_size; i++)
    future_approx_mat_acc(i,0) = std::pow(i, 2) / 2;
  // - Monroe-Penrose inverse
  future_approx_l2_acc =
      ( future_approx_mat_acc.transpose() * future_approx_mat_acc).inverse() *
      future_approx_mat_acc.transpose();

  // Least-squares without restrictions: velocity
  // - initialize matrix that relates the curve coefficients to its points
  Eigen::Matrix<double,Eigen::Dynamic,2> future_approx_mat_vel;
  future_approx_mat_vel.resize(future_buffer_size, Eigen::NoChange_t::NoChange);
  for (int j=0; j < 2; j++)
    for (int i=0; i < future_buffer_size; i++)
      future_approx_mat_vel(i, j) = std::pow(i,j);
  // - Monroe-Penrose inverse
  future_approx_l2_vel =
      ( future_approx_mat_vel.transpose() * future_approx_mat_vel).inverse() *
      future_approx_mat_vel.transpose();

  // Debug
#ifdef DEBUG_OMNITRAJECTORYCTRL
  std::cout << "[InitializeL2Matrices]" << std::endl;
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

void OmniTrajectoryCtrl::RotGlobal2Local(Eigen::Vector3d &x_global,
                                         Eigen::Vector3d &x_local) {
  // Initialize rotation matrix
  Eigen::AngleAxis<double> rot_g2l =
      Eigen::AngleAxis<double>(-x_global(2), Eigen::Vector3d::UnitZ());

  // Compute the vector in the global coordinate frame
  x_local = rot_g2l * x_global;
}

}  // omnidirectional_trajectory_controller
