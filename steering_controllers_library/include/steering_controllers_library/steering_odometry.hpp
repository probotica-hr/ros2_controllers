// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: dr. sc. Tomislav Petkovic, Dr. Ing. Denis Štogl
//

#ifndef STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_
#define STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_

#include "steering_controllers_library/steering_odometry_base.hpp"

#include <cmath>
#include <tuple>
#include <vector>

#include <rclcpp/time.hpp>

// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCPPUTILS_VERSION_MAJOR >= 2 && RCPPUTILS_VERSION_MINOR >= 6
#include "rcpputils/rolling_mean_accumulator.hpp"
#else
#include "rcppmath/rolling_mean_accumulator.hpp"
#endif

namespace steering_odometry
{
const unsigned int BICYCLE_CONFIG = 0;
const unsigned int TRICYCLE_CONFIG = 1;
const unsigned int ACKERMANN_CONFIG = 2;

/**
 * \brief The Odometry class handles odometry readings
 * (2D pose and velocity with related timestamp)
 */
class SteeringOdometry : public SteeringOdometryBase
{
public:
  /**
   * \brief Constructor
   * Timestamp will get the current time value
   * Value will be set to zero
   * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
   *
   */
  SteeringOdometry(size_t velocity_rolling_window_size = 10);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param traction_wheel_pos  traction wheel position [rad]
   * \param steer_pos Steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_position(
    const double traction_wheel_pos, const double steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param right_traction_wheel_pos  Right traction wheel velocity [rad]
   * \param left_traction_wheel_pos  Left traction wheel velocity [rad]
   * \param steer_pos Steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_position(
    const double right_traction_wheel_pos, const double left_traction_wheel_pos,
    const double steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param right_traction_wheel_pos  Right traction wheel position [rad]
   * \param left_traction_wheel_pos  Left traction wheel position [rad]
   * \param right_steer_pos Right steer wheel position [rad]
   * \param left_steer_pos Left steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_position(
    const double right_traction_wheel_pos, const double left_traction_wheel_pos,
    const double right_steer_pos, const double left_steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param traction_wheel_vel  Traction wheel velocity [rad/s]
   * \param steer_pos Steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_velocity(
    const double traction_wheel_vel, const double steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param right_traction_wheel_vel  Right traction wheel velocity [rad/s]
   * \param left_traction_wheel_vel  Left traction wheel velocity [rad/s]
   * \param steer_pos Steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_velocity(
    const double right_traction_wheel_vel, const double left_traction_wheel_vel,
    const double steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param right_traction_wheel_vel  Right traction wheel velocity [rad/s]
   * \param left_traction_wheel_vel  Left traction wheel velocity [rad/s]
   * \param right_steer_pos Right steer wheel position [rad]
   * \param left_steer_pos Left steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_velocity(
    const double right_traction_wheel_vel, const double left_traction_wheel_vel,
    const double right_steer_pos, const double left_steer_pos, const double dt);

  /**
   * \brief Set odometry type
   * \param type odometry type
   */
  void set_odometry_type(const unsigned int type);

  /**
   * \brief Calculates inverse kinematics for the desired linear and angular velocities
   * \param v_bx     Desired linear velocity of the robot in x_b-axis direction
   * \param omega_bz Desired angular velocity of the robot around x_z-axis
   * \param open_loop If false, the IK will be calculated using measured steering angle
   * \param reduce_wheel_speed_until_steering_reached Reduce wheel speed until the steering angle
   * has been reached
   * \return Tuple of velocity commands and steering commands
   */
  std::tuple<std::vector<double>, std::vector<double>> get_commands(
    const double v_bx, const double omega_bz, const bool open_loop = true,
    const bool reduce_wheel_speed_until_steering_reached = false);

private:
  /**
   * \brief Calculates linear velocity of a robot with double traction axle
   * \param right_traction_wheel_vel  Right traction wheel velocity [rad/s]
   * \param left_traction_wheel_vel  Left traction wheel velocity [rad/s]
   * \param steer_pos Steer wheel position [rad]
   */
  double get_linear_velocity_double_traction_axle(
    const double right_traction_wheel_vel, const double left_traction_wheel_vel,
    const double steer_pos);

  /// Configuration type used for the forward kinematics
  int config_type_ = -1;

  /// Previous wheel position/state [rad]:
  double traction_wheel_old_pos_;
  double traction_right_wheel_old_pos_;
  double traction_left_wheel_old_pos_;
};
}  // namespace steering_odometry

#endif  // STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_
