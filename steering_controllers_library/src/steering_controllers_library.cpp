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

#include "steering_controllers_library/steering_controllers_library.hpp"

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{  // utility

using ControllerTwistReferenceMsg =
  steering_controllers_library::SteeringControllersLibrary::ControllerTwistReferenceMsg;

}  // namespace

namespace steering_controllers_library
{
SteeringControllersLibrary::SteeringControllersLibrary()
: SteeringControllerBase(odometry_)
{
}

std::tuple<std::vector<double>, std::vector<double>>
SteeringControllersLibrary::get_commands(
    const double v_bx, const double omega_bz, const bool open_loop)
{
  return odometry_.get_commands(v_bx, omega_bz, open_loop);
}

}  // namespace steering_controllers_library
