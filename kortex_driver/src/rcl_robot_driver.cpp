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
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include <rcl/rcl.h>

namespace kortex_driver
{
namespace
{
using namespace rcl::api;

constexpr rcl::api::RobotArm kArm = rcl::api::RobotArm::Arm1;
constexpr double kNoCmd = std::numeric_limits<double>::quiet_NaN();

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

  Impl()
  {
    for (auto & c : cmd_deg)
    {
      c.store(kNoCmd, std::memory_order_relaxed);
    }
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

    if (rt_active.load(std::memory_order_relaxed) &&
        arm.feedback->status.mode == ArmMode::RealTimeJointPosition)
    {
      arm.joint_command->mode = JointModeOfOperation::Position;
      for (std::size_t i = 0; i < g_max_number_of_actuators; ++i)
      {
        const double cmd = cmd_deg[i].load(std::memory_order_relaxed);
        arm.joint_command->positions[i] =
          std::isnan(cmd) ? arm.feedback->cyclic_data.joints[i].joint_position : cmd;
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
      }
    }
  }
};

RclRobotDriver::RclRobotDriver() : impl_(std::make_unique<Impl>()) {}
RclRobotDriver::~RclRobotDriver() = default;

std::string RclRobotDriver::init(
  const std::string & ethercat_lib_path, int net_cpu, int ctrl_cpu)
{
  try
  {
    const ThreadAffinityConfig affinity{
      .network_thread_cpu_core = net_cpu,
      .control_thread_cpu_core = ctrl_cpu,
    };
    impl_->rcl =
      std::make_unique<RobotControlLibrary>(EtherLabConfig{ethercat_lib_path}, affinity);
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
  impl_->rcl->RegisterSystemFaultCallback(
    [impl]() { impl->system_faulted.store(true); });

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
    "SetPhysicalToolsConfiguration", impl_->rcl->Robot().SetPhysicalToolsConfiguration(kArm, tools));
}

std::string RclRobotDriver::startCyclic()
{
  if (!impl_->rcl)
  {
    return "startCyclic called before init";
  }
  return ToError("StartCyclicCommunication", impl_->rcl->Network().StartCyclicCommunication());
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
  impl_->rt_active.store(true);
  const std::string err =
    ToError("SetArmMode", impl_->rcl->Robot().SetArmMode(kArm, ArmMode::RealTimeJointPosition));
  if (!err.empty())
  {
    impl_->rt_active.store(false);
  }
  return err;
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

bool RclRobotDriver::isSystemFaulted() const
{
  return impl_->system_faulted.load();
}

}  // namespace kortex_driver
