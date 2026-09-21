// Copyright 2026, Kinova / KIMA
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

#ifndef KORTEX_DRIVER__RCL_ROBOT_DRIVER_HPP_
#define KORTEX_DRIVER__RCL_ROBOT_DRIVER_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace kortex_driver
{
/**
 * @brief ROS-free wrapper around Kinova's Robot Control Library (RCL).
 *
 * Kinova's RCL ships an umbrella header `<rcl/rcl.h>` whose include path
 * collides with ROS 2's own client library `rcl` (which also exposes
 * `<rcl/rcl.h>`, `<rcl/version.h>`, etc., and is pulled in transitively by
 * rclcpp). The two cannot share a translation unit under any include order.
 *
 * This wrapper is the single boundary where Kinova RCL is used. Its header pulls
 * in NO Kinova headers (pimpl), and its implementation TU is compiled in a
 * separate CMake target that never sees ROS's `rcl` include directory. The
 * ros2_control plugin talks only to this class and to rclcpp.
 *
 * The wrapper also owns the real-time command bridge: it registers RCL's
 * real-time control callback and exchanges joint commands with it through a
 * lock-free buffer, so callers never touch the real-time thread directly.
 *
 * All joint quantities are in RCL units: degrees, degrees/second, Nm.
 * Every method returns an empty string on success or a human-readable error.
 */
class RclRobotDriver
{
public:
  /** Maximum number of actuators (matches rcl::api::g_max_number_of_actuators). */
  static constexpr std::size_t kMaxJoints = 7;

  /**
   * @brief Coarse RCL system state, mirrored here so this header stays RCL-free.
   *
   * Values match rcl::api::RclSystemState; Unknown is returned when the state
   * cannot be read (e.g. before init()).
   */
  enum class SystemState : std::uint8_t
  {
    Initialization,
    Standby,
    Maintenance,
    Operational,
    Fault,
    Unknown,
  };

  /** Outcome of an asynchronous clear-faults request. */
  enum class ClearFaultsResult : std::uint8_t
  {
    Idle,     ///< No request has been made since the last one was collected.
    Pending,  ///< A clear sequence is running on the worker thread.
    Success,  ///< Every actuator's fault was cleared.
    NotFaulted,  ///< RCL refused the request: the arm was not in Fault, so there was nothing to clear.
    Failure,  ///< The clear sequence ran and finished with at least one fault remaining.
  };

  /** Expected arm identity, validated by the bus scan. */
  struct ArmIdentity
  {
    std::string part_number;
    std::string part_number_revision;
    std::string serial_number;
    std::uint32_t nb_actuators{7};
    std::uint16_t slave_pos_min{0};
    std::uint16_t slave_pos_max{13};
  };

  RclRobotDriver();
  ~RclRobotDriver();

  RclRobotDriver(const RclRobotDriver &) = delete;
  RclRobotDriver & operator=(const RclRobotDriver &) = delete;
  RclRobotDriver(RclRobotDriver &&) = delete;
  RclRobotDriver & operator=(RclRobotDriver &&) = delete;

  /**
   * @brief Construct the library (loads/dlopen's the EtherCAT master) and
   *        register the real-time control bridge + system-fault hook.
   *
   * @param ethercat_lib_path Path to the EtherCAT master shared library.
   * @param net_cpu  0-based core for the network cyclic thread, or -1 (unpinned).
   * @param ctrl_cpu 0-based core for the arms-control thread, or -1 (unpinned).
   */
  std::string init(const std::string & ethercat_lib_path, int net_cpu, int ctrl_cpu);

  /** Scan the bus (Initialization -> Standby) and validate the expected arm. */
  std::string scan(const std::string & topology_content, const ArmIdentity & arm);

  /** Set the gravity vector used by the dynamics model (Standby only). */
  std::string setGravity(double x, double y, double z);

  /** Configure an identity, mass-less tool (Standby only). */
  std::string setNoTool();

  /** Start cyclic exchange (Standby -> Operational). */
  std::string startCyclic();

  /** Stop cyclic exchange (Operational -> Standby). */
  std::string stopCyclic();

  /** Enter real-time joint-position control mode. */
  std::string setRealtimeJointPositionMode();

  /** Leave real-time control (NoMode). */
  std::string setNoMode();

  /**
   * @brief Fill up to @p n joints of the latest feedback snapshot (degrees).
   *
   * @param position_deg Out: joint positions [deg]  (size >= n).
   * @param velocity_deg Out: joint velocities [deg/s] (size >= n).
   * @param torque_nm    Out: joint torques [Nm]      (size >= n).
   * @param n            Number of joints to fill (<= kMaxJoints).
   */
  std::string getFeedback(
    double * position_deg, double * velocity_deg, double * torque_nm, std::size_t n);

  /**
   * @brief Publish the latest desired joint positions (degrees) for the
   *        real-time control callback to consume. Thread-safe, non-blocking.
   */
  void setCommand(const double * position_deg, std::size_t n);

  /** Seed the command buffer (degrees), e.g. right after activation. */
  void seedCommand(const double * position_deg, std::size_t n);

  /** True once a system fault has been signalled by RCL. */
  bool isSystemFaulted() const;

  /** Current coarse system state, or SystemState::Unknown if it cannot be read. */
  SystemState getSystemState() const;

  /**
   * @brief Reset the network (Fault -> Initialization) so a new scan can be issued.
   *
   * RCL only accepts this from a fault state; calling it otherwise returns an error.
   */
  std::string resetNetwork();

  /** Every latched system-fault cause, one per line ("" when no fault is latched). */
  std::string getSystemFaultDescription() const;

  /**
   * @brief True when the arm itself is latched in Fault (distinct from a system fault).
   *
   * Queries the bus, so it is for one-shot use (e.g. on_activate); the update
   * loop should use armFaultLatched() instead.
   */
  bool isArmFaulted() const;

  /**
   * @brief Arm fault state as of the last control cycle.
   *
   * Refreshed by the real-time callback, so this is a plain atomic load with no
   * bus traffic - safe to call every update.
   */
  bool armFaultLatched() const;

  /** Every latched arm-fault cause, one per line ("" when no fault is latched). */
  std::string getArmFaultDescription() const;

  /**
   * @brief Per-joint fault bitfields, one line per actuator.
   *
   * RCL reports only that an actuator faulted, never why; these four banks are
   * the only place the manufacturer's cause bits are exposed. Logged on every
   * arm fault so the cause is recoverable after the fact.
   */
  std::string getFaultBanks() const;

  /**
   * @brief Request an asynchronous ClearArmFaults on the worker thread.
   *
   * ClearArmFaults blocks until the per-actuator sequence resolves, which must
   * never happen on the controller_manager update loop. Returns immediately;
   * poll clearArmFaultsResult(). A request made while one is Pending is ignored.
   */
  void requestClearArmFaults();

  /** Outcome of the latest requestClearArmFaults(). */
  ClearFaultsResult clearArmFaultsResult() const;

  /** RCL's description of the latest clear-faults outcome (empty on success). */
  std::string clearArmFaultsMessage() const;

  /** Reset the clear-faults result back to Idle once it has been consumed. */
  void consumeClearArmFaultsResult();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace kortex_driver

#endif  // KORTEX_DRIVER__RCL_ROBOT_DRIVER_HPP_
