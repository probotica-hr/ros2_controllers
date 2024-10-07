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

#ifndef STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_BASE_HPP_
#define STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_BASE_HPP_

#include <cmath>
#include <tuple>
#include <vector>

#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "rcppmath/rolling_mean_accumulator.hpp"

namespace steering_odometry
{
const unsigned int BICYCLE_CONFIG = 0;
const unsigned int TRICYCLE_CONFIG = 1;
const unsigned int ACKERMANN_CONFIG = 2;

inline bool is_close_to_zero(double val) { return std::fabs(val) < 1e-6; }

/**
 * \brief The Odometry class handles odometry readings
 * (2D pose and velocity with related timestamp)
 */
class SteeringOdometryBase
{
public:
  /**
   * \brief Constructor
   * Timestamp will get the current time value
   * Value will be set to zero
   * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
   *
   */
  explicit SteeringOdometryBase(size_t velocity_rolling_window_size = 10);

  /**
   * \brief Initialize the odometry
   * \param time Current time
   */
  void init(const rclcpp::Time & time);

  /**
   * \brief Updates the odometry class with latest velocity command
   * \param v_bx  Linear velocity   [m/s]
   * \param omega_bz Angular velocity [rad/s]
   * \param dt      time difference to last call
   */
  void update_open_loop(const double v_bx, const double omega_bz, const double dt);

  /**
   * \brief heading getter
   * \return heading [rad]
   */
  double get_heading() const { return heading_; }

  /**
   * \brief x position getter
   * \return x position [m]
   */
  double get_x() const { return x_; }

  /**
   * \brief y position getter
   * \return y position [m]
   */
  double get_y() const { return y_; }

  /**
   * \brief linear velocity getter
   * \return linear velocity [m/s]
   */
  double get_linear() const { return linear_; }

  /**
   * \brief angular velocity getter
   * \return angular velocity [rad/s]
   */
  double get_angular() const { return angular_; }

  /**
   * \brief Sets the wheel parameters: radius, separation and wheelbase
   */
  void set_wheel_params(
    const double wheel_radius, const double wheelbase = 0.0, const double wheel_track = 0.0);

  /**
   * \brief Velocity rolling window size setter
   * \param velocity_rolling_window_size Velocity rolling window size
   */
  void set_velocity_rolling_window_size(const size_t velocity_rolling_window_size);

  /**
   *  \brief Reset poses, heading, and accumulators
   */
  void reset_odometry();

private:
  /**
   * \brief Uses precomputed linear and angular velocities to compute odometry
   * \param v_bx  Linear  velocity   [m/s]
   * \param omega_bz Angular velocity [rad/s]
   * \param dt      time difference to last call
   */
  bool update_odometry(const double v_bx, const double omega_bz, const double dt);

  /**
   * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
   * \param v_bx Linear velocity [m/s]
   * \param omega_bz Angular velocity [rad/s]
   * \param dt time difference to last call
   */
  void integrate_runge_kutta_2(const double v_bx, const double omega_bz, const double dt);

  /**
   * \brief Integrates the velocities (linear and angular)
   * \param v_bx Linear velocity [m/s]
   * \param omega_bz Angular velocity [rad/s]
   * \param dt time difference to last call
   */
  void integrate_fk(const double v_bx, const double omega_bz, const double dt);

  /**
   * \brief Calculates steering angle from the desired twist
   * \param v_bx     Linear velocity of the robot in x_b-axis direction
   * \param omega_bz Angular velocity of the robot around x_z-axis
   */
  double convert_twist_to_steering_angle(const double v_bx, const double omega_bz);

  /**
   *  \brief Reset linear and angular accumulators
   */
  void reset_accumulators();

  /// Current timestamp:
  rclcpp::Time timestamp_;

  /// Current pose:
  double x_;          //   [m]
  double y_;          //   [m]
  double steer_pos_;  // [rad]
  double heading_;    // [rad]

  /// Current velocity:
  double linear_;   //   [m/s]
  double angular_;  // [rad/s]

  /// Kinematic parameters
  double wheel_track_;   // [m]
  double wheelbase_;     // [m]
  double wheel_radius_;  // [m]

  /// Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  rcppmath::RollingMeanAccumulator<double> linear_acc_;
  rcppmath::RollingMeanAccumulator<double> angular_acc_;
};
}  // namespace steering_odometry

#endif  // STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_
