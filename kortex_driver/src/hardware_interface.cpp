// Copyright 2021, PickNik Inc.
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

//----------------------------------------------------------------------
/*!\file
 *
 * KIMA hardware interface backed by Kinova's Robot Control Library (RCL).
 * All Kinova RCL access goes through kortex_driver::RclRobotDriver; this file
 * only deals with ros2_control and unit conversion (RCL is in degrees,
 * ros2_control in radians).
 *
 */
//----------------------------------------------------------------------

#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "kortex_driver/hardware_interface.hpp"
#include "kortex_driver/kortex_math_util.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("KortexMultiInterfaceHardware");
rclcpp::Clock g_clock{RCL_STEADY_TIME};

// Fetch a hardware parameter with a fallback default.
std::string GetParam(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  const std::string & fallback)
{
  const auto it = params.find(key);
  if (it == params.end() || it->second.empty())
  {
    return fallback;
  }
  return it->second;
}
}  // namespace

namespace kortex_driver
{
KortexMultiInterfaceHardware::KortexMultiInterfaceHardware() = default;

CallbackReturn KortexMultiInterfaceHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(LOGGER, "Initializing KIMA RCL hardware interface");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  const auto & p = info_.hardware_parameters;

  // --- EtherCAT / RCL configuration (all defaulted so the interface runs even
  //     before the launch/xacro are updated) ---
  ethercat_lib_path_ = GetParam(p, "ethercat_lib_path", "/usr/local/lib/libethercat.so");
  topology_file_path_ = GetParam(
    p, "network_topology_file",
    "/home/aalmrad/Documents/KIMA/RCL/"
    "robot_control_library_linux_x86_gcc13_release_0.1.0-dev.126/src/examples/"
    "network_topology/etherlab/network_topology_1_arm.yaml");

  expected_arm_.part_number = GetParam(p, "arm_part_number", "ARM-L3M000-001");
  expected_arm_.part_number_revision = GetParam(p, "arm_part_number_revision", "A");
  expected_arm_.serial_number = GetParam(p, "arm_serial_number", "0001");

  try
  {
    expected_arm_.nb_actuators =
      static_cast<std::uint32_t>(std::stoul(GetParam(p, "arm_nb_actuators", "7")));
    expected_arm_.slave_pos_min =
      static_cast<std::uint16_t>(std::stoul(GetParam(p, "slave_pos_range_min", "0")));
    expected_arm_.slave_pos_max =
      static_cast<std::uint16_t>(std::stoul(GetParam(p, "slave_pos_range_max", "13")));

    network_thread_cpu_core_ = std::stoi(GetParam(p, "network_thread_cpu_core", "-1"));
    control_thread_cpu_core_ = std::stoi(GetParam(p, "control_thread_cpu_core", "-1"));

    gravity_x_ = std::stod(GetParam(p, "gravity_x", "0.0"));
    gravity_y_ = std::stod(GetParam(p, "gravity_y", "0.0"));
    gravity_z_ = std::stod(GetParam(p, "gravity_z", "-9.81"));
  }
  catch (const std::exception & ex)
  {
    RCLCPP_ERROR(LOGGER, "Failed to parse a numeric hardware parameter: %s", ex.what());
    return CallbackReturn::ERROR;
  }

  actuator_count_ = expected_arm_.nb_actuators;
  if (actuator_count_ == 0 || actuator_count_ > RclRobotDriver::kMaxJoints)
  {
    RCLCPP_ERROR(
      LOGGER, "arm_nb_actuators (%zu) must be in [1, %zu]", actuator_count_,
      RclRobotDriver::kMaxJoints);
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "EtherCAT master library: %s", ethercat_lib_path_.c_str());
  RCLCPP_INFO(LOGGER, "Network topology file:  %s", topology_file_path_.c_str());
  RCLCPP_INFO(
    LOGGER, "Expected arm: %s rev %s serial %s, %u actuators, slaves [%u, %u]",
    expected_arm_.part_number.c_str(), expected_arm_.part_number_revision.c_str(),
    expected_arm_.serial_number.c_str(), expected_arm_.nb_actuators, expected_arm_.slave_pos_min,
    expected_arm_.slave_pos_max);

  // --- Validate the ros2_control joint interfaces (position command;
  //     position/velocity/effort state; no gripper) ---
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' must have exactly one '%s' command interface.", joint.name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    for (const auto & si : joint.state_interfaces)
    {
      if (si.name != hardware_interface::HW_IF_POSITION &&
          si.name != hardware_interface::HW_IF_VELOCITY &&
          si.name != hardware_interface::HW_IF_EFFORT)
      {
        RCLCPP_FATAL(
          LOGGER, "Joint '%s' has unsupported state interface '%s'. Expected %s, %s, or %s.",
          joint.name.c_str(), si.name.c_str(), hardware_interface::HW_IF_POSITION,
          hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
        return CallbackReturn::ERROR;
      }
    }
  }

  if (info_.joints.size() != actuator_count_)
  {
    RCLCPP_WARN(
      LOGGER, "URDF exposes %zu joints but %zu actuators are expected on the arm.",
      info_.joints.size(), actuator_count_);
  }

  // --- Allocate state/command buffers ---
  const double nan = std::numeric_limits<double>::quiet_NaN();
  arm_positions_.assign(actuator_count_, nan);
  arm_velocities_.assign(actuator_count_, nan);
  arm_efforts_.assign(actuator_count_, nan);
  arm_commands_positions_.assign(actuator_count_, nan);
  fb_position_deg_.assign(actuator_count_, 0.0);
  fb_velocity_deg_.assign(actuator_count_, 0.0);
  fb_torque_nm_.assign(actuator_count_, 0.0);
  cmd_position_deg_.assign(actuator_count_, nan);

  RCLCPP_INFO(LOGGER, "KIMA RCL hardware interface initialized");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KortexMultiInterfaceHardware::on_configure(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Configuring KIMA RCL hardware interface");

  driver_ = std::make_unique<RclRobotDriver>();

  std::string err = driver_->init(
    ethercat_lib_path_, network_thread_cpu_core_, control_thread_cpu_core_);
  if (!err.empty())
  {
    RCLCPP_ERROR(LOGGER, "%s", err.c_str());
    driver_.reset();
    return CallbackReturn::ERROR;
  }

  // Read the network topology file content (ScanNetwork takes content, not a path).
  std::string topology;
  {
    std::ifstream topology_stream{topology_file_path_};
    if (!topology_stream)
    {
      RCLCPP_ERROR(LOGGER, "Cannot open network topology file '%s'", topology_file_path_.c_str());
      driver_.reset();
      return CallbackReturn::ERROR;
    }
    std::stringstream buffer;
    buffer << topology_stream.rdbuf();
    topology = buffer.str();
  }

  err = driver_->scan(topology, expected_arm_);
  if (!err.empty())
  {
    RCLCPP_ERROR(LOGGER, "%s", err.c_str());
    driver_.reset();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(LOGGER, "ScanNetwork succeeded; system is in Standby");

  // Standby-only dynamic-model configuration (best-effort).
  err = driver_->setGravity(gravity_x_, gravity_y_, gravity_z_);
  if (!err.empty())
  {
    RCLCPP_WARN(LOGGER, "%s", err.c_str());
  }
  err = driver_->setNoTool();
  if (!err.empty())
  {
    RCLCPP_WARN(LOGGER, "%s", err.c_str());
  }

  RCLCPP_INFO(LOGGER, "KIMA RCL hardware interface configured");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KortexMultiInterfaceHardware::on_cleanup(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Cleaning up KIMA RCL hardware interface");
  // Destroying the driver stops all RCL threads and unloads the EtherCAT master.
  driver_.reset();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
KortexMultiInterfaceHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size() && i < actuator_count_; i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arm_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &arm_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arm_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KortexMultiInterfaceHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size() && i < actuator_count_; i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arm_commands_positions_[i]));
  }
  return command_interfaces;
}

CallbackReturn KortexMultiInterfaceHardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Activating KIMA RCL hardware interface");

  if (!driver_)
  {
    RCLCPP_ERROR(LOGGER, "RCL not configured; cannot activate.");
    return CallbackReturn::ERROR;
  }

  // Start cyclic exchange (Standby -> Operational).
  std::string err = driver_->startCyclic();
  if (!err.empty())
  {
    RCLCPP_ERROR(LOGGER, "%s", err.c_str());
    return CallbackReturn::ERROR;
  }

  // Seed states and the command buffer from the arm's current position so there
  // is no discontinuity when RealTimeJointPosition engages.
  err = driver_->getFeedback(
    fb_position_deg_.data(), fb_velocity_deg_.data(), fb_torque_nm_.data(), actuator_count_);
  if (!err.empty())
  {
    RCLCPP_ERROR(LOGGER, "Initial feedback read failed: %s", err.c_str());
    return CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    arm_positions_[i] = KortexMathUtil::toRad(fb_position_deg_[i]);
    arm_velocities_[i] = 0.0;
    arm_efforts_[i] = fb_torque_nm_[i];
    arm_commands_positions_[i] = arm_positions_[i];
  }
  driver_->seedCommand(fb_position_deg_.data(), actuator_count_);

  err = driver_->setRealtimeJointPositionMode();
  if (!err.empty())
  {
    RCLCPP_ERROR(LOGGER, "%s", err.c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "KIMA RCL hardware interface activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KortexMultiInterfaceHardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Deactivating KIMA RCL hardware interface");

  if (driver_)
  {
    std::string err = driver_->setNoMode();
    if (!err.empty())
    {
      RCLCPP_WARN(LOGGER, "%s", err.c_str());
    }
    err = driver_->stopCyclic();
    if (!err.empty())
    {
      RCLCPP_WARN(LOGGER, "%s", err.c_str());
    }
  }

  RCLCPP_INFO(LOGGER, "KIMA RCL hardware interface deactivated");
  return CallbackReturn::SUCCESS;
}

return_type KortexMultiInterfaceHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!driver_)
  {
    return return_type::ERROR;
  }

  if (driver_->isSystemFaulted())
  {
    RCLCPP_ERROR_THROTTLE(LOGGER, g_clock, 1000, "System is in fault.");
    return return_type::ERROR;
  }

  const std::string err = driver_->getFeedback(
    fb_position_deg_.data(), fb_velocity_deg_.data(), fb_torque_nm_.data(), actuator_count_);
  if (!err.empty())
  {
    RCLCPP_ERROR_THROTTLE(LOGGER, g_clock, 1000, "%s", err.c_str());
    return return_type::ERROR;
  }

  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    // NOTE: continuous joints (1,3,5,7) may need turn-count unwrapping; v1 uses a
    // plain deg->rad conversion. Revisit with KortexMathUtil::wrapRadiansFromMinusPiToPi
    // if joint_trajectory_controller reports position wrap jumps.
    arm_positions_[i] = KortexMathUtil::toRad(fb_position_deg_[i]);
    arm_velocities_[i] = KortexMathUtil::toRad(fb_velocity_deg_[i]);
    arm_efforts_[i] = fb_torque_nm_[i];
  }

  return return_type::OK;
}

return_type KortexMultiInterfaceHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!driver_)
  {
    return return_type::ERROR;
  }

  // Publish the latest position command (rad -> deg) into the driver's lock-free
  // buffer that the real-time control callback consumes. No bus I/O happens here.
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    cmd_position_deg_[i] = std::isnan(arm_commands_positions_[i])
                             ? std::numeric_limits<double>::quiet_NaN()
                             : KortexMathUtil::toDeg(arm_commands_positions_[i]);
  }
  driver_->setCommand(cmd_position_deg_.data(), actuator_count_);

  return return_type::OK;
}

}  // namespace kortex_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kortex_driver::KortexMultiInterfaceHardware, hardware_interface::SystemInterface)
