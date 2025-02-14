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

#ifndef STEERING_CONTROLLERS_LIBRARY__STEERING_CONTROLLERS_LIBRARY_HPP_
#define STEERING_CONTROLLERS_LIBRARY__STEERING_CONTROLLERS_LIBRARY_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <tuple>

#include "controller_interface/chainable_controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

// TODO(anyone): Replace with controller specific messages
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "control_msgs/msg/steering_controller_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "steering_controllers_library/steering_controller_base.hpp"
#include "steering_controllers_library/steering_controllers_library_parameters.hpp"
#include "steering_controllers_library/steering_odometry.hpp"

namespace steering_controllers_library
{
class SteeringControllersLibrary : public SteeringControllerBase
{
public:
  SteeringControllersLibrary();

  std::tuple<std::vector<double>, std::vector<double>>
  get_commands(const double v_bx, const double omega_bz, const bool open_loop = true) override;

  using ControllerAckermannReferenceMsg = ackermann_msgs::msg::AckermannDriveStamped;
  using ControllerTwistReferenceMsg = geometry_msgs::msg::TwistStamped;
  using ControllerStateMsgOdom = nav_msgs::msg::Odometry;
  using ControllerStateMsgTf = tf2_msgs::msg::TFMessage;
  using AckermannControllerState = control_msgs::msg::SteeringControllerStatus;

protected:
  /// Odometry:
  steering_odometry::SteeringOdometry odometry_;
};

}  // namespace steering_controllers_library

#endif  // STEERING_CONTROLLERS_LIBRARY__STEERING_CONTROLLERS_LIBRARY_HPP_
