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

// This is the ONLY translation unit that includes Kinova's RCL. It must never
// be compiled with ROS 2's `rcl` include directory on the command line (see
// rcl_robot_driver.hpp for why). Keep all Kinova RCL usage confined here.

#include "kortex_driver/rcl_robot_driver.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <limits>
#include <memory>
#include <mutex>
#include <span>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rcl/rcl.h>

namespace kortex_driver
{
namespace
{
using namespace rcl::api;

constexpr rcl::api::RobotArm kArm = rcl::api::RobotArm::Arm1;
constexpr double kNoCmd = std::numeric_limits<double>::quiet_NaN();

// How long the arm must stay out of Fault after a post-clear re-enable before
// the recovery counts as successful. Actuator self-test faults on enable have
// been seen latching ~5 ms after OperationEnabled, so this leaves a wide margin.
constexpr std::chrono::milliseconds kReenableSettle{50};

// A position command further than this from the measured position is never a
// real setpoint: the fastest joint covers ~0.36 deg per 1 ms cycle. Such a
// command is replaced by the last accepted one instead of reaching the drives.
constexpr double kMaxCommandOffsetDeg = 10.0;

// ClearArmFaults attempts when RCL reports the clear as incomplete (see workerLoop).
constexpr int kClearAttempts = 3;
constexpr std::chrono::milliseconds kClearRetryDelay{300};

// Turn an RCL Result into "" on success or "<prefix>: <description>".
std::string ToError(const char * prefix, const rcl::Result & res)
{
  if (res)
  {
    return {};
  }
  return std::string(prefix) + ": " + res.description.data();
}
}  // namespace

struct RclRobotDriver::Impl
{
  std::unique_ptr<RobotControlLibrary> rcl;

  // Command bridge: latest commanded joint positions [deg]. Lock-free on
  // x86-64, so safe to load/store from the real-time control thread.
  std::array<std::atomic<double>, kMaxJoints> cmd_deg;

  // True while the arm is expected to be in RealTimeJointPosition mode.
  std::atomic<bool> rt_active{false};

  // Set by the system-fault callback.
  std::atomic<bool> system_faulted{false};

  // Mirrors the arm's fault state, refreshed every control cycle.
  std::atomic<bool> arm_faulted{false};

  // Control cycles run since construction. GetArmFeedback returns an unfilled
  // snapshot until the first cyclic frame has been processed, so callers wait
  // on this before trusting feedback (see waitForFeedback()).
  std::atomic<std::uint64_t> control_cycles{0};

  // Command guard: last command accepted per joint, and how many were rejected.
  std::array<double, kMaxJoints> last_good_deg{};
  std::atomic<std::uint64_t> rejected_cmds{0};
  std::array<std::atomic<double>, kMaxJoints> last_rejected_deg;

  // Clear-faults worker. ClearArmFaults() blocks until the per-actuator
  // sequence resolves, so it runs here instead of on the update loop.
  std::thread clear_worker;
  std::mutex clear_mutex;
  std::condition_variable clear_cv;
  bool clear_requested{false};
  bool clear_stop{false};
  std::atomic<RclRobotDriver::ClearFaultsResult> clear_result{
    RclRobotDriver::ClearFaultsResult::Idle};
  mutable std::mutex clear_msg_mutex;
  std::string clear_msg;

  Impl()
  {
    for (auto & c : cmd_deg)
    {
      c.store(kNoCmd, std::memory_order_relaxed);
    }
    for (auto & c : last_rejected_deg)
    {
      c.store(kNoCmd, std::memory_order_relaxed);
    }
  }

  ~Impl() { stopWorker(); }

  void startWorker()
  {
    clear_worker = std::thread([this] { workerLoop(); });
  }

  void stopWorker()
  {
    if (!clear_worker.joinable())
    {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(clear_mutex);
      clear_stop = true;
    }
    clear_cv.notify_one();
    clear_worker.join();
  }

  void workerLoop()
  {
    for (;;)
    {
      {
        std::unique_lock<std::mutex> lock(clear_mutex);
        clear_cv.wait(lock, [this] { return clear_requested || clear_stop; });
        if (clear_stop)
        {
          return;
        }
        clear_requested = false;
      }

      // A clear can come back ArmFaultClearIncomplete even though every fault
      // bank already reads zero: RCL samples the actuators' fault-status bits
      // before they drop. Asking again finds no actuator in fault and releases
      // the arm, so retry that one case a few times.
      rcl::Result res = rcl->Robot().ClearArmFaults(kArm);
      for (int attempt = 2; !res && res.error_code == rcl::ErrorCode::ArmFaultClearIncomplete &&
                            attempt <= kClearAttempts;
           ++attempt)
      {
        std::this_thread::sleep_for(kClearRetryDelay);
        res = rcl->Robot().ClearArmFaults(kArm);
      }

      // RCL only accepts ClearFault while the arm is in Fault; asking to clear a
      // healthy arm comes back as NotAllowedInCurrentState. That is "nothing to
      // clear", which is materially different from a clear sequence that ran and
      // left faults behind - collapsing both onto Failure is what produced the
      // self-contradicting "faults remaining: (no actuator reports a non-zero
      // fault bank)" log.
      RclRobotDriver::ClearFaultsResult outcome = RclRobotDriver::ClearFaultsResult::Success;
      std::string msg = res.description.data();
      if (!res)
      {
        outcome = (res.error_code == rcl::ErrorCode::NotAllowedInCurrentState)
                    ? RclRobotDriver::ClearFaultsResult::NotFaulted
                    : RclRobotDriver::ClearFaultsResult::Failure;
      }
      else
      {
        // The clear left the arm in Idle. Re-enabling blocks for the whole
        // enable sequence (~750 ms measured), so it happens here rather than
        // on the update loop.
        msg = reenable();
        if (!msg.empty())
        {
          outcome = RclRobotDriver::ClearFaultsResult::Failure;
        }
      }

      {
        std::lock_guard<std::mutex> lock(clear_msg_mutex);
        clear_msg = msg;
      }
      clear_result.store(outcome, std::memory_order_release);
    }
  }

  // Enter RealTimeJointPosition mode. Blocks until the arm is enabled.
  rcl::Result setRealtimeMode()
  {
    rt_active.store(true);
    const rcl::Result res = rcl->Robot().SetArmMode(kArm, ArmMode::RealTimeJointPosition);
    if (!res)
    {
      rt_active.store(false);
    }
    return res;
  }

  // Worker-thread half of fault recovery: reseed the command buffer from the
  // live position, re-enter real-time mode and confirm the arm stays enabled.
  // Returns "" on success or the reason it failed.
  std::string reenable()
  {
    // Drop out of real-time commanding first. The arm may still report
    // RealTimeJointPosition after the fault, in which case the control callback
    // would keep replaying the command buffered before the fault instead of
    // mirroring the live position into it.
    rt_active.store(false);

    ArmFeedback feedback;
    const rcl::Result fb = rcl->Robot().GetArmFeedback(kArm, feedback);
    if (!fb)
    {
      return ToError("Faults cleared, but GetArmFeedback failed", fb);
    }
    for (std::size_t i = 0; i < g_max_number_of_actuators; ++i)
    {
      cmd_deg[i].store(feedback.cyclic_data.joints[i].joint_position, std::memory_order_relaxed);
    }

    const rcl::Result mode = setRealtimeMode();
    if (!mode)
    {
      return ToError("Faults cleared, but SetArmMode failed", mode);
    }

    // SetArmMode returning OK only means the arm reached OperationEnabled; a
    // self-test fault can still latch a few cycles later.
    std::this_thread::sleep_for(kReenableSettle);
    if (arm_faulted.load(std::memory_order_relaxed))
    {
      rt_active.store(false);
      return "Faults cleared, but the arm faulted again within " +
             std::to_string(kReenableSettle.count()) + " ms of re-enabling";
    }
    return {};
  }

  // Real-time control callback: runs on RCL's control thread. No blocking,
  // no allocation, no throwing.
  void onControlCycle(std::span<const ArmIO> arms) noexcept
  {
    if (arms.empty())
    {
      return;
    }
    const ArmIO & arm = arms[0];
    control_cycles.fetch_add(1, std::memory_order_release);

    // Cheapest place to track the arm's fault state: read() needs it every
    // cycle and this snapshot is already in hand.
    arm_faulted.store(arm.feedback->status.state == ArmState::Fault, std::memory_order_relaxed);

    if (
      rt_active.load(std::memory_order_relaxed) &&
      arm.feedback->status.mode == ArmMode::RealTimeJointPosition)
    {
      arm.joint_command->mode = JointModeOfOperation::Position;
      for (std::size_t i = 0; i < g_max_number_of_actuators; ++i)
      {
        const double fb = arm.feedback->cyclic_data.joints[i].joint_position;
        double cmd = cmd_deg[i].load(std::memory_order_relaxed);
        if (std::isnan(cmd))
        {
          cmd = fb;
        }
        else if (!std::isfinite(cmd) || std::fabs(cmd - fb) > kMaxCommandOffsetDeg)
        {
          last_rejected_deg[i].store(cmd, std::memory_order_relaxed);
          rejected_cmds.fetch_add(1, std::memory_order_relaxed);
          cmd = last_good_deg[i];
        }
        last_good_deg[i] = cmd;
        arm.joint_command->positions[i] = cmd;
      }
    }
    else
    {
      // Not in real-time mode yet: mirror the current position so the command
      // buffer never drives a jump when the mode engages.
      for (std::size_t i = 0; i < g_max_number_of_actuators; ++i)
      {
        cmd_deg[i].store(
          arm.feedback->cyclic_data.joints[i].joint_position, std::memory_order_relaxed);
        last_good_deg[i] = arm.feedback->cyclic_data.joints[i].joint_position;
      }
    }
  }
};

RclRobotDriver::RclRobotDriver() : impl_(std::make_unique<Impl>()) {}
RclRobotDriver::~RclRobotDriver() = default;

std::string RclRobotDriver::init(const std::string & ethercat_lib_path, int net_cpu, int ctrl_cpu)
{
  try
  {
    const ThreadAffinityConfig affinity{
      .network_thread_cpu_core = net_cpu,
      .control_thread_cpu_core = ctrl_cpu,
    };
    impl_->rcl = std::make_unique<RobotControlLibrary>(EtherLabConfig{ethercat_lib_path}, affinity);
  }
  catch (const RclException & ex)
  {
    return std::string("RobotControlLibrary construction failed: ") + ex.what();
  }
  catch (const std::exception & ex)
  {
    return std::string("RobotControlLibrary construction failed (std): ") + ex.what();
  }

  impl_->rcl->Logging().SetLoggingConfiguration(
    LoggingConfiguration{.type = LoggingType::Console, .verbosity = LoggingVerbosity::Info});

  // Register the real-time control callback before the system goes Operational.
  Impl * impl = impl_.get();
  const rcl::Result cb = impl_->rcl->Robot().RegisterRealTimeControlCallback(
    [impl](std::span<const ArmIO> arms) { impl->onControlCycle(arms); });
  if (!cb)
  {
    return ToError("RegisterRealTimeControlCallback", cb);
  }

  impl_->system_faulted.store(false);
  impl_->rcl->RegisterSystemFaultCallback([impl]() { impl->system_faulted.store(true); });

  impl_->startWorker();

  return {};
}

std::string RclRobotDriver::scan(const std::string & topology_content, const ArmIdentity & arm)
{
  if (!impl_->rcl)
  {
    return "scan called before init";
  }

  RobotArmInformation info;
  info.m_part_number = arm.part_number;
  info.m_part_number_revision = arm.part_number_revision;
  info.m_serial_number = arm.serial_number;
  info.m_nb_actuators = arm.nb_actuators;
  info.m_slave_pos_range = {arm.slave_pos_min, arm.slave_pos_max};

  const std::vector<RobotArmInformation> expected{info};
  return ToError("ScanNetwork", impl_->rcl->Network().ScanNetwork(expected, topology_content));
}

std::string RclRobotDriver::setGravity(double x, double y, double z)
{
  if (!impl_->rcl)
  {
    return "setGravity called before init";
  }
  const GravityVector gravity{.x = x, .y = y, .z = z};
  return ToError("SetGravityVector", impl_->rcl->Robot().SetGravityVector(kArm, gravity));
}

std::string RclRobotDriver::setNoTool()
{
  if (!impl_->rcl)
  {
    return "setNoTool called before init";
  }
  ToolConfiguration tool{};
  tool.transform[0][0] = 1.0;
  tool.transform[1][1] = 1.0;
  tool.transform[2][2] = 1.0;
  tool.transform[3][3] = 1.0;
  const std::vector<ToolConfiguration> tools{tool};
  return ToError(
    "SetPhysicalToolsConfiguration",
    impl_->rcl->Robot().SetPhysicalToolsConfiguration(kArm, tools));
}

std::string RclRobotDriver::startCyclic()
{
  if (!impl_->rcl)
  {
    return "startCyclic called before init";
  }
  return ToError("StartCyclicCommunication", impl_->rcl->Network().StartCyclicCommunication());
}

std::string RclRobotDriver::waitForFeedback(std::chrono::milliseconds timeout)
{
  if (!impl_->rcl)
  {
    return "waitForFeedback called before init";
  }
  // StartCyclicCommunication returns as soon as the bus reaches Op, before the
  // first frame has been exchanged; until then GetArmFeedback hands back a
  // snapshot that was never written (seen as values like 1.6e280 deg). Two
  // completed control cycles guarantee at least one full feedback update.
  const std::uint64_t start = impl_->control_cycles.load(std::memory_order_acquire);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (impl_->control_cycles.load(std::memory_order_acquire) < start + 2)
  {
    if (std::chrono::steady_clock::now() >= deadline)
    {
      return "No cyclic feedback within " + std::to_string(timeout.count()) +
             " ms of starting the cyclic exchange";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds{1});
  }

  // Belt and braces: refuse a snapshot that still does not look like joint angles.
  ArmFeedback feedback;
  const rcl::Result res = impl_->rcl->Robot().GetArmFeedback(kArm, feedback);
  if (!res)
  {
    return ToError("GetArmFeedback", res);
  }
  for (std::size_t i = 0; i < g_max_number_of_actuators; ++i)
  {
    const double p = feedback.cyclic_data.joints[i].joint_position;
    if (!std::isfinite(p) || std::fabs(p) > 3600.0)
    {
      return "Implausible feedback after start: joint " + std::to_string(i + 1) + " position " +
             std::to_string(p) + " deg";
    }
  }
  return {};
}

std::string RclRobotDriver::stopCyclic()
{
  if (!impl_->rcl)
  {
    return "stopCyclic called before init";
  }
  return ToError("StopCyclicCommunication", impl_->rcl->Network().StopCyclicCommunication());
}

std::string RclRobotDriver::setRealtimeJointPositionMode()
{
  if (!impl_->rcl)
  {
    return "setRealtimeJointPositionMode called before init";
  }
  return ToError("SetArmMode", impl_->setRealtimeMode());
}

std::string RclRobotDriver::setNoMode()
{
  impl_->rt_active.store(false);
  if (!impl_->rcl)
  {
    return {};
  }
  return ToError("SetArmMode(NoMode)", impl_->rcl->Robot().SetArmMode(kArm, ArmMode::NoMode));
}

std::string RclRobotDriver::getFeedback(
  double * position_deg, double * velocity_deg, double * torque_nm, std::size_t n)
{
  if (!impl_->rcl)
  {
    return "getFeedback called before init";
  }
  if (n > g_max_number_of_actuators)
  {
    n = g_max_number_of_actuators;
  }

  ArmFeedback feedback;
  const rcl::Result res = impl_->rcl->Robot().GetArmFeedback(kArm, feedback);
  if (!res)
  {
    return ToError("GetArmFeedback", res);
  }

  for (std::size_t i = 0; i < n; ++i)
  {
    const JointFeedback & j = feedback.cyclic_data.joints[i];
    position_deg[i] = j.joint_position;
    velocity_deg[i] = j.joint_velocity;
    torque_nm[i] = j.joint_torque;
  }
  return {};
}

void RclRobotDriver::setCommand(const double * position_deg, std::size_t n)
{
  if (n > g_max_number_of_actuators)
  {
    n = g_max_number_of_actuators;
  }
  for (std::size_t i = 0; i < n; ++i)
  {
    if (!std::isnan(position_deg[i]))
    {
      impl_->cmd_deg[i].store(position_deg[i], std::memory_order_relaxed);
    }
  }
}

void RclRobotDriver::seedCommand(const double * position_deg, std::size_t n)
{
  if (n > g_max_number_of_actuators)
  {
    n = g_max_number_of_actuators;
  }
  for (std::size_t i = 0; i < n; ++i)
  {
    impl_->cmd_deg[i].store(position_deg[i], std::memory_order_relaxed);
  }
}

bool RclRobotDriver::isSystemFaulted() const { return impl_->system_faulted.load(); }

bool RclRobotDriver::armFaultLatched() const
{
  return impl_->arm_faulted.load(std::memory_order_relaxed);
}

RclRobotDriver::SystemState RclRobotDriver::getSystemState() const
{
  if (!impl_->rcl)
  {
    return SystemState::Unknown;
  }

  SystemInformation info;
  if (!impl_->rcl->GetSystemInformation(info))
  {
    return SystemState::Unknown;
  }

  switch (info.state)
  {
    case RclSystemState::Initialization:
      return SystemState::Initialization;
    case RclSystemState::Standby:
      return SystemState::Standby;
    case RclSystemState::Maintenance:
      return SystemState::Maintenance;
    case RclSystemState::Operational:
      return SystemState::Operational;
    case RclSystemState::Fault:
      return SystemState::Fault;
    default:
      return SystemState::Unknown;
  }
}

std::string RclRobotDriver::resetNetwork()
{
  if (!impl_->rcl)
  {
    return "resetNetwork called before init";
  }
  // A successful reset returns the system to Initialization, so the fault flag
  // raised by the callback no longer applies to the new scan.
  const std::string err = ToError("ResetNetwork", impl_->rcl->Network().ResetNetwork());
  if (err.empty())
  {
    impl_->system_faulted.store(false);
  }
  return err;
}

std::string RclRobotDriver::getSystemFaultDescription() const
{
  if (!impl_->rcl)
  {
    return {};
  }

  Fault fault;
  if (!impl_->rcl->GetSystemFault(fault) || fault.IsEmpty())
  {
    return {};
  }

  std::string out;
  for (const auto & cause : fault.Errors())
  {
    if (!out.empty())
    {
      out += "\n";
    }
    out += "  - ";
    out += cause.description.data();
  }
  return out;
}

bool RclRobotDriver::isArmFaulted() const
{
  if (!impl_->rcl)
  {
    return false;
  }

  ArmFeedback feedback;
  if (!impl_->rcl->Robot().GetArmFeedback(kArm, feedback))
  {
    return false;
  }
  return feedback.status.state == ArmState::Fault;
}

std::string RclRobotDriver::getArmFaultDescription() const
{
  if (!impl_->rcl)
  {
    return {};
  }

  Fault fault;
  if (!impl_->rcl->Robot().GetArmFault(kArm, fault) || fault.IsEmpty())
  {
    return {};
  }

  std::string out;
  for (const auto & cause : fault.Errors())
  {
    if (!out.empty())
    {
      out += "\n";
    }
    out += "  - ";
    out += cause.description.data();
  }
  return out;
}

std::string RclRobotDriver::getFaultBanks() const
{
  if (!impl_->rcl)
  {
    return {};
  }

  ArmFeedback feedback;
  const rcl::Result res = impl_->rcl->Robot().GetArmFeedback(kArm, feedback);
  if (!res)
  {
    return ToError("GetArmFeedback", res);
  }

  std::ostringstream out;
  for (std::size_t i = 0; i < g_max_number_of_actuators; ++i)
  {
    const JointFeedback & j = feedback.cyclic_data.joints[i];
    // Only the actuators that actually latched something are worth printing.
    if ((j.fault_bank_a | j.fault_bank_b | j.fault_bank_c | j.fault_bank_d) == 0U)
    {
      continue;
    }
    out << "  actuator " << (i + 1) << ": a=0x" << std::hex << j.fault_bank_a << " b=0x"
        << j.fault_bank_b << " c=0x" << j.fault_bank_c << " d=0x" << j.fault_bank_d << std::dec
        << " (sto=" << (j.sto_activated ? "1" : "0") << " brakes=" << (j.brakes_engaged ? "1" : "0")
        << " V=" << j.voltage << " I=" << j.current << ")\n";
  }

  const std::string banks = out.str();
  return banks.empty() ? std::string{"  (no actuator reports a non-zero fault bank)"} : banks;
}

void RclRobotDriver::requestClearArmFaults()
{
  if (!impl_->rcl || !impl_->clear_worker.joinable())
  {
    impl_->clear_result.store(ClearFaultsResult::Failure, std::memory_order_release);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(impl_->clear_mutex);
    if (impl_->clear_requested)
    {
      return;  // one already queued
    }
    impl_->clear_requested = true;
  }
  impl_->clear_result.store(ClearFaultsResult::Pending, std::memory_order_release);
  impl_->clear_cv.notify_one();
}

std::uint64_t RclRobotDriver::rejectedCommandCount(double * last_rejected_deg, std::size_t n) const
{
  if (n > kMaxJoints)
  {
    n = kMaxJoints;
  }
  for (std::size_t i = 0; i < n; ++i)
  {
    last_rejected_deg[i] = impl_->last_rejected_deg[i].load(std::memory_order_relaxed);
  }
  return impl_->rejected_cmds.load(std::memory_order_relaxed);
}

RclRobotDriver::ClearFaultsResult RclRobotDriver::clearArmFaultsResult() const
{
  return impl_->clear_result.load(std::memory_order_acquire);
}

std::string RclRobotDriver::clearArmFaultsMessage() const
{
  std::lock_guard<std::mutex> lock(impl_->clear_msg_mutex);
  return impl_->clear_msg;
}

void RclRobotDriver::consumeClearArmFaultsResult()
{
  impl_->clear_result.store(ClearFaultsResult::Idle, std::memory_order_release);
}

}  // namespace kortex_driver
