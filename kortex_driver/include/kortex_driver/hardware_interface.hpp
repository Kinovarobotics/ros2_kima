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
 *
 * The arm is driven directly over EtherCAT (EtherLab master) instead of the
 * Kortex TCP/UDP API. RCL owns its own 1 kHz real-time thread: real-time joint
 * commands can only be delivered from inside RCL's real-time control callback.
 * This interface therefore acts as a bridge between ros2_control's read()/write()
 * and RCL's real-time thread. All Kinova RCL access is confined to
 * kortex_driver::RclRobotDriver (see rcl_robot_driver.hpp) so that Kinova's
 * <rcl/rcl.h> never clashes with ROS 2's identically-named rcl headers.
 *
 * v1 scope: joint position command only (RCL exposes only
 * ArmMode::RealTimeJointPosition); position/velocity/effort exported as state.
 * No gripper (KIMA is arm-only on the EtherCAT bus).
 *
 */
//----------------------------------------------------------------------
#ifndef KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_
#define KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_

#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "kortex_driver/rcl_robot_driver.hpp"
#include "kortex_driver/visibility_control.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kortex_driver
{
class KortexMultiInterfaceHardware : public hardware_interface::SystemInterface
{
public:
  KortexMultiInterfaceHardware();

  RCLCPP_SHARED_PTR_DEFINITIONS(KortexMultiInterfaceHardware);

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) final;

  KORTEX_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  KORTEX_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) final;

  KORTEX_DRIVER_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) final;

  KORTEX_DRIVER_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) final;

private:
  // The ROS-free RCL wrapper (owns the EtherCAT master + real-time bridge).
  std::unique_ptr<RclRobotDriver> driver_;

  // --- Configuration parsed in on_init() ---
  std::string ethercat_lib_path_;
  std::string topology_file_path_;
  RclRobotDriver::ArmIdentity expected_arm_;
  int network_thread_cpu_core_{-1};
  int control_thread_cpu_core_{-1};
  double gravity_x_{0.0};
  double gravity_y_{0.0};
  double gravity_z_{-9.81};

  std::size_t actuator_count_{RclRobotDriver::kMaxJoints};

  // --- State interfaces (radians / rad-s / Nm) ---
  std::vector<double> arm_positions_;
  std::vector<double> arm_velocities_;
  std::vector<double> arm_efforts_;

  // --- Command interface storage bound in export_command_interfaces (radians) ---
  std::vector<double> arm_commands_positions_;

  // --- Scratch buffers (degrees) for the read()/write() bridge ---
  std::vector<double> fb_position_deg_;
  std::vector<double> fb_velocity_deg_;
  std::vector<double> fb_torque_nm_;
  std::vector<double> cmd_position_deg_;

  // --- reset_fault interfaces, driven by picknik_reset_fault_controller ---
  // The controller writes ISSUE_CMD to reset_fault/command and polls
  // reset_fault/async_success; we mirror the latched arm fault on
  // reset_fault/internal_fault. Faults are never cleared automatically.
  double reset_fault_cmd_{std::numeric_limits<double>::quiet_NaN()};
  double reset_fault_async_success_{std::numeric_limits<double>::quiet_NaN()};
  double in_fault_{0.0};

  // Command-guard rejections already reported by read().
  std::uint64_t rejected_logged_{0};

  // Number of consecutive ScanNetwork attempts before giving up in on_configure.
  static constexpr int kMaxScanAttempts = 3;

  /**
   * @brief Seed states/commands from current feedback and enter RT joint-position mode.
   *
   * Used at activation only; it blocks for the whole enable sequence. After a
   * fault is cleared the driver's worker thread re-enters the mode instead, so
   * the update loop never blocks. Returns false on failure.
   */
  bool enterRealtimeMode();
};

}  // namespace kortex_driver

#endif  // KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_
