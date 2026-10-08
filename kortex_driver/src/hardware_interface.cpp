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

#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "kortex_driver/hardware_interface.hpp"
#include "kortex_driver/kortex_math_util.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
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

// Default EtherCAT network topology descriptor. Resolved through the ament index
// from kinova_rcl_vendor, which installs the descriptors shipped with the RCL
// SDK, so the default follows the vendored SDK version instead of a path baked
// in at authoring time. Returns an empty string if the package cannot be found;
// on_init reports that when it fails to open the file.
std::string DefaultTopologyFile()
{
  try
  {
    return ament_index_cpp::get_package_share_directory("kinova_rcl_vendor") +
           "/network_topology/etherlab/network_topology_1_arm.yaml";
  }
  catch (const std::exception & ex)
  {
    RCLCPP_WARN(
      LOGGER,
      "Could not locate the kinova_rcl_vendor share directory: %s. Set the "
      "'network_topology_file' hardware parameter explicitly.",
      ex.what());
    return {};
  }
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
  topology_file_path_ = GetParam(p, "network_topology_file", DefaultTopologyFile());

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
    if (
      joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' must have exactly one '%s' command interface.", joint.name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    for (const auto & si : joint.state_interfaces)
    {
      if (
        si.name != hardware_interface::HW_IF_POSITION &&
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

  std::string err =
    driver_->init(ethercat_lib_path_, network_thread_cpu_core_, control_thread_cpu_core_);
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

  // ScanNetwork can report success while the bus-state monitor latches a system
  // fault microseconds later, on the transition into Standby (a transient
  // PreOp->None blip is enough). Verify the state we actually landed in, and
  // reset + rescan if it is Fault: ResetNetwork() returns the system to
  // Initialization, which is the only way back from a system fault.
  bool scanned = false;
  for (int attempt = 1; attempt <= kMaxScanAttempts; ++attempt)
  {
    err = driver_->scan(topology, expected_arm_);

    if (err.empty() && driver_->getSystemState() == RclRobotDriver::SystemState::Standby)
    {
      RCLCPP_INFO(LOGGER, "ScanNetwork succeeded; system is in Standby");
      scanned = true;
      break;
    }

    if (err.empty())
    {
      const std::string fault = driver_->getSystemFaultDescription();
      RCLCPP_WARN(
        LOGGER,
        "ScanNetwork returned success but the system is not in Standby (attempt %d/%d).%s%s",
        attempt, kMaxScanAttempts, fault.empty() ? "" : " Latched system fault:\n", fault.c_str());
    }
    else
    {
      RCLCPP_WARN(LOGGER, "%s (attempt %d/%d)", err.c_str(), attempt, kMaxScanAttempts);
    }

    if (attempt == kMaxScanAttempts)
    {
      break;
    }

    const std::string reset_err = driver_->resetNetwork();
    if (!reset_err.empty())
    {
      RCLCPP_ERROR(LOGGER, "Cannot recover: %s", reset_err.c_str());
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{500});
  }

  if (!scanned)
  {
    RCLCPP_ERROR(
      LOGGER, "Failed to bring the network to Standby after %d attempts.", kMaxScanAttempts);
    driver_.reset();
    return CallbackReturn::ERROR;
  }

  // Standby-only dynamic-model configuration. These are not optional: without
  // them the dynamics model is unconfigured, so a failure here is an error.
  err = driver_->setGravity(gravity_x_, gravity_y_, gravity_z_);
  if (!err.empty())
  {
    RCLCPP_ERROR(LOGGER, "%s", err.c_str());
    driver_.reset();
    return CallbackReturn::ERROR;
  }
  err = driver_->setNoTool();
  if (!err.empty())
  {
    RCLCPP_ERROR(LOGGER, "%s", err.c_str());
    driver_.reset();
    return CallbackReturn::ERROR;
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
  state_interfaces.emplace_back(
    hardware_interface::StateInterface("reset_fault", "internal_fault", &in_fault_));
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
  command_interfaces.emplace_back(
    hardware_interface::CommandInterface("reset_fault", "command", &reset_fault_cmd_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "reset_fault", "async_success", &reset_fault_async_success_));
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

  // startCyclic() returns before the first frame is processed; feedback read in
  // that window is uninitialized. Seeding the command buffer from it sent
  // setpoints like 1e281 deg and faulted actuators 2/4/6, and it also hid
  // already-latched faults from isArmFaulted() below.
  err = driver_->waitForFeedback(std::chrono::milliseconds{500});
  if (!err.empty())
  {
    RCLCPP_ERROR(LOGGER, "%s", err.c_str());
    return CallbackReturn::ERROR;
  }

  // The first cyclic exchange is where a pre-existing actuator fault surfaces:
  // the drives latch it themselves, so it is already set before we command
  // anything. Activation still succeeds when faulted, matching upstream
  // ros2_kortex: the fault controller can only claim reset_fault/* while the
  // hardware is active, so failing here would make ~/reset_fault - the only way
  // to clear the fault - unreachable exactly when it is needed. write() gates
  // all command output on in_fault_, so a faulted arm cannot move.
  if (driver_->isArmFaulted())
  {
    in_fault_ = 1.0;
    const std::string causes = driver_->getArmFaultDescription();
    RCLCPP_ERROR(
      LOGGER, "Arm is latched in Fault; no commands will be sent.%s%s",
      causes.empty() ? "" : " Causes:\n", causes.c_str());
    // RCL reports only that an actuator faulted, never why. These banks are the
    // only record of the cause, so log them while the fault is still latched.
    RCLCPP_ERROR(LOGGER, "Actuator fault banks:\n%s", driver_->getFaultBanks().c_str());
    RCLCPP_ERROR(
      LOGGER,
      "Inspect the arm, then clear with: ros2 service call "
      "/fault_controller/reset_fault example_interfaces/srv/Trigger");

    RCLCPP_INFO(LOGGER, "KIMA RCL hardware interface activated (faulted, commands inhibited)");
    return CallbackReturn::SUCCESS;
  }
  in_fault_ = 0.0;

  if (!enterRealtimeMode())
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "KIMA RCL hardware interface activated");
  return CallbackReturn::SUCCESS;
}

bool KortexMultiInterfaceHardware::enterRealtimeMode()
{
  // Seed states and the command buffer from the arm's current position so there
  // is no discontinuity when RealTimeJointPosition engages.
  std::string err = driver_->getFeedback(
    fb_position_deg_.data(), fb_velocity_deg_.data(), fb_torque_nm_.data(), actuator_count_);
  if (!err.empty())
  {
    RCLCPP_ERROR(LOGGER, "Initial feedback read failed: %s", err.c_str());
    return false;
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
    return false;
  }
  return true;
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

  // The real-time guard replaces implausible commands; report each new batch.
  std::array<double, RclRobotDriver::kMaxJoints> rejected{};
  const std::uint64_t n_rejected = driver_->rejectedCommandCount(rejected.data(), rejected.size());
  if (n_rejected != rejected_logged_)
  {
    RCLCPP_ERROR_THROTTLE(
      LOGGER, g_clock, 1000,
      "Command guard rejected %llu implausible position command(s) so far; last rejected "
      "[deg]: %g %g %g %g %g %g %g",
      static_cast<unsigned long long>(n_rejected), rejected[0], rejected[1], rejected[2],
      rejected[3], rejected[4], rejected[5], rejected[6]);
    rejected_logged_ = n_rejected;
  }

  // Published on reset_fault/internal_fault; refreshed from the control callback,
  // so this costs an atomic load rather than a bus query.
  const bool faulted = driver_->armFaultLatched();
  if (faulted && in_fault_ == 0.0)
  {
    // RCL's own log only says "self-test triggered"; the causes and fault banks
    // are the only record of why, so capture them on the edge while latched.
    // This queries the bus and allocates, which is acceptable once per fault.
    const std::string causes = driver_->getArmFaultDescription();
    RCLCPP_ERROR(
      LOGGER, "Arm fault latched.%s%s\nActuator fault banks:\n%s",
      causes.empty() ? "" : " Causes:\n", causes.c_str(), driver_->getFaultBanks().c_str());
  }
  in_fault_ = faulted ? 1.0 : 0.0;

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

  // Service a pending reset_fault request from the fault controller. Clearing
  // and re-enabling both block, so they run on the driver's worker thread and we
  // only poll here; the controller spins on reset_fault/async_success meanwhile.
  bool recovering = false;
  if (!std::isnan(reset_fault_cmd_))
  {
    switch (driver_->clearArmFaultsResult())
    {
      case RclRobotDriver::ClearFaultsResult::Idle:
        RCLCPP_INFO(LOGGER, "reset_fault requested; clearing arm faults.");
        driver_->requestClearArmFaults();
        recovering = true;
        break;

      case RclRobotDriver::ClearFaultsResult::Pending:
        recovering = true;  // still running; keep the controller waiting
        break;

      case RclRobotDriver::ClearFaultsResult::Success:
        // The worker already re-entered real-time mode from the live position
        // and confirmed the arm stayed enabled. Match the command interface to
        // that position so nothing buffered before the fault is replayed.
        RCLCPP_INFO(LOGGER, "Arm faults cleared; real-time mode re-entered.");
        in_fault_ = 0.0;
        for (std::size_t i = 0; i < actuator_count_; i++)
        {
          arm_commands_positions_[i] = arm_positions_[i];
        }
        reset_fault_async_success_ = 1.0;
        reset_fault_cmd_ = std::numeric_limits<double>::quiet_NaN();
        driver_->consumeClearArmFaultsResult();
        break;

      case RclRobotDriver::ClearFaultsResult::NotFaulted:
        // RCL rejects ClearFault unless the arm is actually in Fault. A reset
        // asked of a healthy arm is a no-op, not a failure, so report success:
        // a reset service should be idempotent. Deliberately does NOT call
        // enterRealtimeMode() - there is no fault to recover from, and re-entering
        // the mode would interrupt whatever controller is currently running.
        RCLCPP_INFO(LOGGER, "reset_fault: arm is not in Fault; nothing to clear.");
        in_fault_ = 0.0;
        reset_fault_async_success_ = 1.0;
        reset_fault_cmd_ = std::numeric_limits<double>::quiet_NaN();
        driver_->consumeClearArmFaultsResult();
        break;

      case RclRobotDriver::ClearFaultsResult::Failure:
        RCLCPP_ERROR(
          LOGGER, "Fault recovery failed: %s\nFault banks:\n%s",
          driver_->clearArmFaultsMessage().c_str(), driver_->getFaultBanks().c_str());
        reset_fault_async_success_ = 0.0;
        reset_fault_cmd_ = std::numeric_limits<double>::quiet_NaN();
        driver_->consumeClearArmFaultsResult();
        break;
    }
  }

  // A faulted arm accepts no commands. Gating here (rather than refusing to
  // activate) keeps the reset path above reachable, as upstream ros2_kortex does.
  // Commands also stay gated while a recovery is in flight: the arm drops out of
  // Fault as soon as the clear finishes, before the worker has reseeded and
  // re-enabled, and a controller's pre-fault command must not land in between.
  if (recovering)
  {
    return return_type::OK;
  }
  if (in_fault_ != 0.0)
  {
    RCLCPP_ERROR_THROTTLE(
      LOGGER, g_clock, 5000, "Arm is faulted; commands inhibited. Clear via ~/reset_fault.");
    return return_type::OK;
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
