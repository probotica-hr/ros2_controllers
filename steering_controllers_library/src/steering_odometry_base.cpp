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

/*
 * Author: dr. sc. Tomislav Petkovic
 * Author: Dr. Ing. Denis Stogl
 */

#include "steering_controllers_library/steering_odometry_base.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace steering_odometry
{
SteeringOdometryBase::SteeringOdometryBase(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_(0.0),
  angular_(0.0),
  wheel_track_(0.0),
  wheelbase_(0.0),
  wheel_radius_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_acc_(velocity_rolling_window_size),
  angular_acc_(velocity_rolling_window_size)
{
}

void SteeringOdometryBase::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  reset_accumulators();
  timestamp_ = time;
}

bool SteeringOdometryBase::update_odometry(
  const double linear_velocity, const double angular_velocity, const double dt)
{
  /// Integrate odometry:
  integrate_fk(linear_velocity, angular_velocity, dt);

  /// We cannot estimate the speed with very small time intervals:
  if (dt < 0.0001)
  {
    return false;  // Interval too small to integrate with
  }

  /// Estimate speeds using a rolling mean to filter them out:
  linear_acc_.accumulate(linear_velocity);
  angular_acc_.accumulate(angular_velocity);

  linear_ = linear_acc_.getRollingMean();
  angular_ = angular_acc_.getRollingMean();

  return true;
}

void SteeringOdometryBase::update_open_loop(const double v_bx, const double omega_bz, const double dt)
{
  /// Save last linear and angular velocity:
  linear_ = v_bx;
  angular_ = omega_bz;

  /// Integrate odometry:
  integrate_fk(v_bx, omega_bz, dt);
}

void SteeringOdometryBase::set_wheel_params(double wheel_radius, double wheelbase, double wheel_track)
{
  wheel_radius_ = wheel_radius;
  wheelbase_ = wheelbase;
  wheel_track_ = wheel_track;
}

void SteeringOdometryBase::set_velocity_rolling_window_size(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;

  reset_accumulators();
}

double SteeringOdometryBase::convert_twist_to_steering_angle(double v_bx, double omega_bz)
{
  // phi can be nan if both v_bx and omega_bz are zero
  const auto phi = std::atan(omega_bz * wheelbase_ / v_bx);
  return std::isfinite(phi) ? phi : 0.0;
}

void SteeringOdometryBase::reset_odometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  reset_accumulators();
}

void SteeringOdometryBase::integrate_runge_kutta_2(
  const double v_bx, const double omega_bz, const double dt)
{
  // Compute intermediate value of the heading
  const double theta_mid = heading_ + omega_bz * 0.5 * dt;

  // Use the intermediate values to update the state
  x_ += v_bx * std::cos(theta_mid) * dt;
  y_ += v_bx * std::sin(theta_mid) * dt;
  heading_ += omega_bz * dt;
}

void SteeringOdometryBase::integrate_fk(const double v_bx, const double omega_bz, const double dt)
{
  const double delta_x_b = v_bx * dt;
  const double delta_theta = omega_bz * dt;

  if (is_close_to_zero(delta_theta))
  {
    /// Runge-Kutta 2nd Order (should solve problems when omega_bz is zero):
    integrate_runge_kutta_2(v_bx, omega_bz, dt);
  }
  else
  {
    /// Exact integration
    const double heading_old = heading_;
    const double R = delta_x_b / delta_theta;
    heading_ += delta_theta;
    x_ += R * (sin(heading_) - std::sin(heading_old));
    y_ += -R * (cos(heading_) - std::cos(heading_old));
  }
}

void SteeringOdometryBase::reset_accumulators()
{
  linear_acc_ = rcppmath::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
  angular_acc_ = rcppmath::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
}

}  // namespace steering_odometry
