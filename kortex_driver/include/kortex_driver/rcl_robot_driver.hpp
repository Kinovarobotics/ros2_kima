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

#include <chrono>
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
   * Mirrors rcl::api::SystemState; Unknown is returned before init().
   */
  enum class SystemState : std::uint8_t
  {
    Initialization,
    Configuration,
    Maintenance,
    Operational,
    Fault,
    Unknown,
  };

  /** Outcome of an asynchronous clear-faults request. */
  enum class ClearFaultsResult : std::uint8_t
  {
    Idle,        ///< No request has been made since the last one was collected.
    Pending,     ///< A clear sequence is running on the worker thread.
    Success,     ///< Faults cleared, real-time mode re-entered, and the arm stayed enabled.
    NotFaulted,  ///< RCL refused the request: the arm was not in Fault, so there was nothing to clear.
    Failure,     ///< Faults remained, re-entering real-time mode failed, or the arm faulted again.
  };

  /** Control mode of one joint in real-time joint control. */
  enum class JointMode : std::uint8_t
  {
    Position,
    Velocity,
    Torque,
  };

  /** Outcome of an asynchronous joint-mode change. */
  enum class ModeSwitchResult : std::uint8_t
  {
    Idle,     ///< No request has been made since the last one was collected.
    Pending,  ///< The arm is being re-enabled in the requested modes.
    Success,  ///< The arm is in real-time control in the requested modes and stayed enabled.
    Failure,  ///< The change failed; every joint fell back to position control.
  };

  /** Expected arm identity, validated by the bus scan. */
  struct ArmIdentity
  {
    std::string part_number;
    std::uint32_t nb_actuators{7};
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
   * @param topology_content  Content of the network topology file.
   * @param arm      Arm expected on the bus.
   * @param net_cpu  0-based core for the network cyclic thread, or -1 (unpinned).
   * @param ctrl_cpu 0-based core for the arms-control thread, or -1 (unpinned).
   * @param torque_gravity_compensated When true a torque command is added to the
   *        model's static torque, so 0 Nm makes the joint float. When false it is
   *        sent as the total joint torque.
   */
  std::string init(
    const std::string & ethercat_lib_path, const std::string & topology_content,
    const ArmIdentity & arm, int net_cpu, int ctrl_cpu, bool torque_gravity_compensated);

  /** Scan the bus (Initialization -> Configuration) and read the arm's limits. */
  std::string scan();

  /** Set the gravity vector used by the dynamics model (Configuration only). */
  std::string setGravity(double x, double y, double z);

  /** Configure an identity, mass-less tool (Configuration only). */
  std::string setNoTool();

  /** Start cyclic exchange (Configuration -> Operational). */
  std::string startCyclic();

  /**
   * @brief Block until the cyclic exchange has delivered real feedback.
   *
   * startCyclic() returns before the first frame is processed, and feedback read
   * in that window is uninitialized memory. Call this after startCyclic() and
   * before any getFeedback()/isArmFaulted(). Errors on timeout or if the
   * feedback still is not a plausible set of joint angles.
   */
  std::string waitForFeedback(std::chrono::milliseconds timeout);

  /** Stop cyclic exchange (Operational -> Configuration). */
  std::string stopCyclic();

  /** Enter real-time joint control with every joint in position mode. */
  std::string setRealtimeJointPositionMode();

  /**
   * @brief Bring the arm to rest in Standby ahead of a joint-mode change.
   *
   * A joint's mode can only change by selecting the arm mode again. Blocks until
   * the arm has stopped, so it must not run on the real-time update loop. Follow
   * it with requestModeSwitch().
   */
  std::string holdForModeSwitch();

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
   * @brief Publish the latest joint commands for the real-time control callback
   *        to consume. Thread-safe, non-blocking.
   *
   * Each joint only follows the array matching its current JointMode. A NaN
   * position keeps the previous command. A NaN velocity stops the joint and a NaN
   * torque holds it against gravity.
   *
   * @param position_deg   Joint positions [deg]    (size >= n).
   * @param velocity_deg_s Joint velocities [deg/s] (size >= n).
   * @param torque_nm      Joint torques [Nm]       (size >= n).
   * @param n              Number of joints (<= kMaxJoints).
   */
  void setCommand(
    const double * position_deg, const double * velocity_deg_s, const double * torque_nm,
    std::size_t n);

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
   * @brief Request an asynchronous fault recovery on the worker thread.
   *
   * Runs ClearFaults, then reseeds the command buffer from the live position,
   * re-enters real-time control with every joint in position mode and checks the
   * arm is still unfaulted after a
   * short settle window. Both RCL calls block (the re-enable for ~750 ms), which
   * must never happen on the controller_manager update loop. Returns immediately;
   * poll clearArmFaultsResult(). A request made while one is Pending is ignored.
   */
  void requestClearArmFaults();

  /**
   * @brief Number of position commands the real-time guard has rejected so far.
   *
   * A command that is non-finite or more than 10 deg from the measured position
   * is never sent; the joint keeps its last accepted command instead. Fills
   * @p last_rejected_deg (size >= n) with the last rejected value per joint
   * (NaN if none). Lock-free.
   */
  std::uint64_t rejectedCommandCount(double * last_rejected_deg, std::size_t n) const;

  /** Outcome of the latest requestClearArmFaults(). */
  ClearFaultsResult clearArmFaultsResult() const;

  /** RCL's description of the latest clear-faults outcome (empty on success). */
  std::string clearArmFaultsMessage() const;

  /** Reset the clear-faults result back to Idle once it has been consumed. */
  void consumeClearArmFaultsResult();

  /**
   * @brief Request real-time control in @p modes on the worker thread.
   *
   * Call holdForModeSwitch() first. Returns immediately, so it is safe on the
   * real-time update loop; poll modeSwitchResult().
   *
   * @param modes Requested mode per joint (size >= n).
   * @param n     Number of joints (<= kMaxJoints); any others use position mode.
   */
  void requestModeSwitch(const JointMode * modes, std::size_t n);

  /** Outcome of the latest requestModeSwitch(). */
  ModeSwitchResult modeSwitchResult() const;

  /** Reason for the latest mode-switch failure (empty on success). */
  std::string modeSwitchMessage() const;

  /** Reset the mode-switch result back to Idle once it has been consumed. */
  void consumeModeSwitchResult();

  /** Fill @p modes (size >= n) with the mode each joint is currently in. Lock-free. */
  void getJointModes(JointMode * modes, std::size_t n) const;

  /**
   * @brief Number of velocity or torque commands changed to respect the arm's limits.
   *
   * Counts cycles where a command was clamped to the joint's velocity or torque
   * limit or reduced near a position limit. Lock-free.
   */
  std::uint64_t limitedCommandCount() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace kortex_driver

#endif  // KORTEX_DRIVER__RCL_ROBOT_DRIVER_HPP_
