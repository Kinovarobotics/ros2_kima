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

// Standalone arm fault clear, straight through Kinova's RCL (no ROS).
//
// Actuators keep a latched fault across restarts, and while one is latched the
// ros2_control hardware cannot enter real-time mode, so the ~/reset_fault
// service may be unreachable. This tool scans the bus, starts the cyclic
// exchange, prints the latched causes and per-actuator fault banks, runs
// ClearArmFaults and prints the result. It never selects an arm mode, so the arm
// stays in Idle with its brakes engaged throughout.
//
// It must not run while ros2_control_node holds the EtherCAT master.
//
// Like rcl_robot_driver.cpp, this translation unit includes Kinova's <rcl/rcl.h>
// and must never be compiled with ROS 2's rcl include directory.

#include <rcl/rcl.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace rcl;
using namespace rcl::api;

namespace
{
constexpr int kClearAttempts = 3;
constexpr std::chrono::milliseconds kClearRetryDelay{300};

const char * StateName(ArmState s)
{
  switch (s)
  {
    case ArmState::Fault: return "Fault";
    case ArmState::EmergencyBrakeRelease: return "EmergencyBrakeRelease";
    case ArmState::OperationEnabled: return "OperationEnabled";
    case ArmState::Idle: return "Idle";
    default: return "other";
  }
}

void Report(RobotControlLibrary & rcl, const char * tag)
{
  ArmFeedback fb;
  Result r = rcl.Robot().GetArmFeedback(RobotArm::Arm1, fb);
  if (!r)
  {
    std::cout << "[" << tag << "] GetArmFeedback failed: " << r.description.data() << "\n";
    return;
  }
  std::cout << "[" << tag << "] arm state: " << StateName(fb.status.state) << "\n";

  Fault fault;
  if (rcl.Robot().GetArmFault(RobotArm::Arm1, fault) && !fault.IsEmpty())
  {
    for (const auto & e : fault.Errors())
    {
      std::cout << "[" << tag << "]   cause: " << e.description.data() << "\n";
    }
  }

  bool any = false;
  for (std::size_t i = 0; i < g_max_number_of_actuators; ++i)
  {
    const JointFeedback & j = fb.cyclic_data.joints[i];
    if ((j.fault_bank_a | j.fault_bank_b | j.fault_bank_c | j.fault_bank_d) == 0U)
    {
      continue;
    }
    any = true;
    std::printf(
      "[%s]   actuator %zu: a=0x%x b=0x%x c=0x%x d=0x%x sto=%d brakes=%d V=%.2f I=%.3f\n", tag,
      i + 1, static_cast<unsigned>(j.fault_bank_a), static_cast<unsigned>(j.fault_bank_b),
      static_cast<unsigned>(j.fault_bank_c), static_cast<unsigned>(j.fault_bank_d),
      j.sto_activated ? 1 : 0, j.brakes_engaged ? 1 : 0, static_cast<double>(j.voltage),
      static_cast<double>(j.current));
  }
  if (!any)
  {
    std::cout << "[" << tag << "]   (no actuator reports a non-zero fault bank)\n";
  }
  std::cout.flush();
}

bool Check(const Result & r, const char * what)
{
  if (!r)
  {
    std::cerr << what << " failed: " << r.description.data() << "\n";
    return false;
  }
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cerr << "usage: " << argv[0] << " <network_topology.yaml>\n";
    return 2;
  }
  std::ifstream in{argv[1]};
  std::stringstream buf;
  buf << in.rdbuf();
  const std::string topology = buf.str();
  if (topology.empty())
  {
    std::cerr << "empty or unreadable topology file: " << argv[1] << "\n";
    return 2;
  }

  LoggingApi{}.SetLoggingConfiguration(
    LoggingConfiguration{.type = LoggingType::Console, .verbosity = LoggingVerbosity::Info});

  try
  {
    RobotControlLibrary rcl{EtherLabConfig{"/usr/local/lib/libethercat.so"}};

    const std::vector<RobotArmInformation> arms{{
      .m_part_number = "ARM-L3M000-001",
      .m_part_number_revision = "A",
      .m_serial_number = "0001",
      .m_nb_actuators = 7,
      .m_slave_pos_range = {0, 13},
    }};
    if (!Check(rcl.Network().ScanNetwork(arms, topology), "ScanNetwork")) return 1;
    if (!Check(rcl.Network().StartCyclicCommunication(), "StartCyclicCommunication")) return 1;

    // Let the first cyclic exchanges land so the latched fault is reported.
    std::this_thread::sleep_for(std::chrono::milliseconds{500});
    Report(rcl, "before");

    // A clear can come back ArmFaultClearIncomplete even though every fault bank
    // already reads zero: RCL samples the actuators' fault-status bits before
    // they drop. Asking again finds no actuator in fault and releases the arm,
    // so retry that one case a few times.
    Result clear = rcl.Robot().ClearArmFaults(RobotArm::Arm1);
    for (int attempt = 2;
         !clear && clear.error_code == ErrorCode::ArmFaultClearIncomplete && attempt <= kClearAttempts;
         ++attempt)
    {
      std::cout << "ClearArmFaults: " << clear.description.data() << " Retrying (" << attempt
                << "/" << kClearAttempts << ")...\n";
      std::this_thread::sleep_for(kClearRetryDelay);
      clear = rcl.Robot().ClearArmFaults(RobotArm::Arm1);
    }
    std::cout << "ClearArmFaults: " << (clear ? "OK" : clear.description.data()) << "\n";

    std::this_thread::sleep_for(std::chrono::milliseconds{500});
    Report(rcl, "after");

    Check(rcl.Network().StopCyclicCommunication(), "StopCyclicCommunication");
    return clear ? 0 : 1;
  }
  catch (const std::exception & ex)
  {
    std::cerr << "Error: " << ex.what() << "\n";
    return 1;
  }
}
