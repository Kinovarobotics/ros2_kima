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

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <span>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rcl/rcl.h>

namespace kortex_driver
{
namespace
{
using namespace rcl::api;

constexpr ArmId kArm = ArmId::Arm1;
constexpr double kNoCmd = std::numeric_limits<double>::quiet_NaN();

// Velocity and torque commands have no position limit of their own. Motion
// toward a joint limit is ramped out over the last degrees before this margin.
constexpr double kPositionLimitMarginDeg = 2.0;
constexpr double kPositionLimitGain = 5.0;  // [deg/s per deg of remaining travel]

// Real-time modes apply each cycle's setpoint as is, so a velocity step demands
// near-infinite acceleration: a 0 -> 80 deg/s step on joints 1, 3, 5 and 7 drew
// ~33 A on joint 1 alone, and at 86 deg/s the bus voltage sagged to 13.5 V and
// the EtherCAT link dropped. Velocity commands are therefore slewed at most at
// these accelerations (Kinova's recommended maxima, joints 1 to 7).
constexpr std::array<double, 7> kMaxAccelerationDegS2{720.0,  720.0,  1080.0, 1080.0,
                                                      1440.0, 1440.0, 1440.0};
constexpr double kCyclePeriodS = 0.001;  // RCL's 1 kHz control cycle

// Torque commands are slewed the same way, so that the full torque range is
// crossed in no less than this. Only the torque on top of the model's static
// torque is ramped; the static torque itself follows the pose every cycle.
constexpr double kTorqueFullScaleRampS = 0.1;

// Safety cap on the torque commanded on top of the static torque, joints 1 to 7
// (about 10% of each joint's limit). Torque mode has no speed limit and only
// cuts the push within kPositionLimitMarginDeg of a limit, so larger torques
// can spin a joint up fast enough to coast through that margin and fault the
// arm at its position limit. Raise these deliberately, one joint at a time.
constexpr std::array<double, 7> kMaxTorquePushNm{10.0, 10.0, 7.0, 7.0, 3.5, 3.5, 3.5};

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
std::string ToError(const char * prefix, const Result & res)
{
  if (res)
  {
    return {};
  }
  return std::string(prefix) + ": " + res.description.data();
}

JointModeOfOperation ToRcl(RclRobotDriver::JointMode mode)
{
  switch (mode)
  {
    case RclRobotDriver::JointMode::Velocity:
      return JointModeOfOperation::Velocity;
    case RclRobotDriver::JointMode::Torque:
      return JointModeOfOperation::Torque;
    case RclRobotDriver::JointMode::Position:
    default:
      return JointModeOfOperation::Position;
  }
}

RclRobotDriver::JointMode FromRcl(JointModeOfOperation mode)
{
  switch (mode)
  {
    case JointModeOfOperation::Velocity:
      return RclRobotDriver::JointMode::Velocity;
    case JointModeOfOperation::Torque:
      return RclRobotDriver::JointMode::Torque;
    case JointModeOfOperation::Position:
    default:
      return RclRobotDriver::JointMode::Position;
  }
}

// Per-cycle CSV log of the arm's feedback and of the command sent to RCL, for
// offline analysis of actuator behaviour. The control callback only copies a
// snapshot into a preallocated single-producer/single-consumer ring; a separate
// thread formats and writes it, so file I/O never runs on the real-time thread.
// Rows are dropped (and counted) if the writer falls behind.
class CyclicLog
{
public:
  struct Joint
  {
    // Feedback reported by the actuator.
    std::uint8_t fb_mode;
    double fb_position, fb_velocity, fb_torque, fb_current, fb_voltage, fb_static_torque,
      fb_following_error;
    std::uint32_t fault_a, fault_b, fault_c, fault_d;
    // Command written to RCL this cycle.
    std::uint8_t cmd_mode;
    double cmd_position, cmd_velocity, cmd_torque;
    // User input from ros2_control, before the driver's limiting.
    double in_position, in_velocity, in_torque;
  };
  struct Row
  {
    std::int64_t unix_ns;
    std::uint64_t cycle;
    std::uint8_t arm_state, arm_mode;
    std::uint8_t joints;
    std::array<Joint, RclRobotDriver::kMaxJoints> j;
  };

  ~CyclicLog() { stop(); }

  // Opens the file and starts the writer. Not real-time safe; call before the
  // cyclic exchange starts. Returns the file actually opened, or "" on failure.
  std::string start(const std::string & path, std::size_t joints, std::string & err)
  {
    namespace fs = std::filesystem;
    std::string file = path;
    std::error_code ec;
    if (fs::is_directory(path, ec))
    {
      char stamp[32];
      const std::time_t now = std::time(nullptr);
      std::tm tm{};
      localtime_r(&now, &tm);
      std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tm);
      file = (fs::path(path) / ("kima_cyclic_" + std::string(stamp) + ".csv")).string();
    }
    out_ = std::fopen(file.c_str(), "w");
    if (!out_)
    {
      err = "cannot open cyclic log file '" + file + "'";
      return {};
    }
    std::setvbuf(out_, nullptr, _IOFBF, 1 << 20);
    const fs::path fp(file);
    const std::string events = (fp.parent_path() / (fp.stem().string() + "_faults.csv")).string();
    events_ = std::fopen(events.c_str(), "w");
    if (events_)
    {
      std::fprintf(events_, "unix_ns,cycle,event,details\n");
      std::fflush(events_);
    }
    joints_ = std::min(joints, RclRobotDriver::kMaxJoints);
    ring_.resize(kRingSize);
    writeHeader();
    running_.store(true);
    writer_ = std::thread([this] { writeLoop(); });
    file_ = file;
    enabled_.store(true, std::memory_order_release);
    return file;
  }

  void stop()
  {
    enabled_.store(false, std::memory_order_release);
    if (writer_.joinable())
    {
      running_.store(false);
      writer_.join();
    }
    if (out_)
    {
      std::fclose(out_);
      out_ = nullptr;
    }
    std::lock_guard<std::mutex> lock(events_mutex_);
    if (events_)
    {
      std::fclose(events_);
      events_ = nullptr;
    }
  }

  // Non-real-time side: one row in <log>_faults.csv, flushed at once so it
  // survives a crash. Line breaks in @p details become " | ".
  void logEvent(std::uint64_t cycle, const std::string & event, const std::string & details)
  {
    std::lock_guard<std::mutex> lock(events_mutex_);
    if (!events_)
    {
      return;
    }
    std::string text;
    for (const char c : details)
    {
      if (c == '\n')
      {
        text += " | ";
      }
      else if (c == '"')
      {
        text += "\"\"";
      }
      else
      {
        text += c;
      }
    }
    const auto unix_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::system_clock::now().time_since_epoch())
                           .count();
    std::fprintf(
      events_, "%lld,%llu,%s,\"%s\"\n", static_cast<long long>(unix_ns),
      static_cast<unsigned long long>(cycle), event.c_str(), text.c_str());
    std::fflush(events_);
  }

  bool enabled() const noexcept { return enabled_.load(std::memory_order_acquire); }
  const std::string & file() const noexcept { return file_; }
  std::uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }

  // Real-time side: returns the slot to fill, or nullptr when the ring is full.
  Row * beginPush() noexcept
  {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    if (head - tail_.load(std::memory_order_acquire) >= kRingSize)
    {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return nullptr;
    }
    return &ring_[head & (kRingSize - 1)];
  }
  void endPush() noexcept
  {
    head_.store(head_.load(std::memory_order_relaxed) + 1, std::memory_order_release);
  }

private:
  static constexpr std::size_t kRingSize = 1 << 14;  // ~16 s at 1 kHz

  void writeHeader()
  {
    std::fprintf(out_, "unix_ns,cycle,arm_state,arm_mode");
    static const char * kFields[] = {
      "fb_mode",
      "fb_position_deg",
      "fb_velocity_deg_s",
      "fb_torque_nm",
      "fb_current_a",
      "fb_voltage_v",
      "fb_static_torque_nm",
      "fb_following_error_deg",
      "fault_a",
      "fault_b",
      "fault_c",
      "fault_d",
      "cmd_mode",
      "cmd_position_deg",
      "cmd_velocity_deg_s",
      "cmd_torque_nm",
      "in_position_deg",
      "in_velocity_deg_s",
      "in_torque_nm"};
    for (std::size_t i = 0; i < joints_; ++i)
    {
      for (const char * f : kFields)
      {
        std::fprintf(out_, ",j%zu_%s", i + 1, f);
      }
    }
    std::fprintf(out_, "\n");
  }

  void writeLoop()
  {
    for (;;)
    {
      const std::size_t head = head_.load(std::memory_order_acquire);
      std::size_t tail = tail_.load(std::memory_order_relaxed);
      if (tail == head)
      {
        if (!running_.load())
        {
          break;
        }
        std::fflush(out_);
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
        continue;
      }
      for (; tail != head; ++tail)
      {
        writeRow(ring_[tail & (kRingSize - 1)]);
        tail_.store(tail + 1, std::memory_order_release);
      }
    }
    std::fflush(out_);
  }

  void writeRow(const Row & r)
  {
    std::fprintf(
      out_, "%lld,%llu,%u,%u", static_cast<long long>(r.unix_ns),
      static_cast<unsigned long long>(r.cycle), r.arm_state, r.arm_mode);
    for (std::size_t i = 0; i < joints_ && i < r.joints; ++i)
    {
      const Joint & j = r.j[i];
      std::fprintf(
        out_,
        ",%u,%.4f,%.4f,%.4f,%.4f,%.3f,%.4f,%.4f,0x%x,0x%x,0x%x,0x%x"
        ",%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
        j.fb_mode, j.fb_position, j.fb_velocity, j.fb_torque, j.fb_current, j.fb_voltage,
        j.fb_static_torque, j.fb_following_error, j.fault_a, j.fault_b, j.fault_c, j.fault_d,
        j.cmd_mode, j.cmd_position, j.cmd_velocity, j.cmd_torque, j.in_position, j.in_velocity,
        j.in_torque);
    }
    std::fprintf(out_, "\n");
  }

  std::FILE * out_{nullptr};
  std::FILE * events_{nullptr};
  std::mutex events_mutex_;
  std::string file_;
  std::size_t joints_{0};
  std::vector<Row> ring_;
  std::atomic<std::size_t> head_{0};
  std::atomic<std::size_t> tail_{0};
  std::atomic<std::uint64_t> dropped_{0};
  std::atomic<bool> enabled_{false};
  std::atomic<bool> running_{false};
  std::thread writer_;
};
}  // namespace

struct RclRobotDriver::Impl
{
  std::unique_ptr<RobotControlLibrary> rcl;

  // Command bridge: latest commanded joint positions [deg]. Lock-free on
  // x86-64 and aarch64, so safe to load/store from the real-time control thread.
  std::array<std::atomic<double>, kMaxJoints> cmd_deg;

  // Latest velocity [deg/s] and torque [Nm] commands; NaN means no command yet.
  std::array<std::atomic<double>, kMaxJoints> cmd_velocity_deg_s;
  std::array<std::atomic<double>, kMaxJoints> cmd_torque_nm;

  // Mode written to each joint's command every cycle. RCL faults the arm if it
  // changes while the arm stays in RealTimeJoint, so it is only stored while the
  // arm is out of that mode (see holdForModeSwitch() and reenable()).
  std::array<std::atomic<JointModeOfOperation>, kMaxJoints> joint_mode;

  // When true a torque command is added to the model's static torque.
  bool torque_gravity_compensated{true};

  // The arm's limits, read once after the scan and before the cyclic exchange.
  std::array<double, kMaxJoints> position_min_deg{};
  std::array<double, kMaxJoints> position_max_deg{};
  std::array<double, kMaxJoints> velocity_limit_deg_s{};
  std::array<double, kMaxJoints> torque_limit_nm{};
  // False for a joint RCL reports as continuous or with an empty position range:
  // velocity and torque commands are then not ramped out near position_*_deg.
  std::array<bool, kMaxJoints> position_limited{};
  std::string limits_summary;
  std::atomic<std::uint64_t> limited_cmds{0};

  // True while the arm is expected to be in RealTimeJoint mode.
  std::atomic<bool> rt_active{false};

  // Set by the system-fault callback.
  std::atomic<bool> system_faulted{false};

  // Mirrors the arm's fault state, refreshed every control cycle.
  std::atomic<bool> arm_faulted{false};

  // Control cycles run since construction. GetArmFeedback returns an unfilled
  // snapshot until the first cyclic frame has been processed, so callers wait
  // on this before trusting feedback (see waitForFeedback()).
  std::atomic<std::uint64_t> control_cycles{0};

  // Optional per-cycle CSV log (see startCyclicLog()).
  CyclicLog cyclic_log;

  // Command guard: last command accepted per joint, and how many were rejected.
  std::array<double, kMaxJoints> last_good_deg{};
  std::atomic<std::uint64_t> rejected_cmds{0};

  // Velocity sent to RCL on the previous cycle, for the acceleration limit.
  // Control thread only. Zero whenever the joint is not velocity-commanded,
  // since a mode switch stops the arm first.
  std::array<double, kMaxJoints> last_velocity_deg_s{};
  // Same for torque: what was sent on top of the static torque last cycle.
  std::array<double, kMaxJoints> last_push_nm{};
  static_assert(kMaxAccelerationDegS2.size() == kMaxJoints);
  static_assert(kMaxTorquePushNm.size() == kMaxJoints);
  std::array<std::atomic<double>, kMaxJoints> last_rejected_deg;

  // Worker for the requests that block in RCL: clearing faults and selecting the
  // arm mode both wait for the actuators, so they run here instead of on the
  // update loop.
  std::thread clear_worker;
  std::mutex clear_mutex;
  std::condition_variable clear_cv;
  bool clear_requested{false};
  bool switch_requested{false};
  std::array<JointMode, kMaxJoints> requested_modes{};
  bool clear_stop{false};
  std::atomic<RclRobotDriver::ClearFaultsResult> clear_result{
    RclRobotDriver::ClearFaultsResult::Idle};
  std::atomic<RclRobotDriver::ModeSwitchResult> switch_result{
    RclRobotDriver::ModeSwitchResult::Idle};
  mutable std::mutex clear_msg_mutex;
  std::string clear_msg;
  std::string switch_msg;

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
    clearVelocityAndTorqueCommands();
    setAllJointsToPosition();
  }

  void clearVelocityAndTorqueCommands()
  {
    for (std::size_t i = 0; i < kMaxJoints; ++i)
    {
      cmd_velocity_deg_s[i].store(kNoCmd, std::memory_order_relaxed);
      cmd_torque_nm[i].store(kNoCmd, std::memory_order_relaxed);
    }
  }

  void setAllJointsToPosition()
  {
    for (auto & m : joint_mode)
    {
      m.store(JointModeOfOperation::Position, std::memory_order_relaxed);
    }
  }

  ~Impl()
  {
    stopWorker();
    // Stop RCL's threads before the members its control callback touches are
    // destroyed; the cyclic log's ring in particular is freed below.
    rcl.reset();
    cyclic_log.stop();
  }

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
      bool do_clear = false;
      bool do_switch = false;
      std::array<JointMode, kMaxJoints> modes{};
      {
        std::unique_lock<std::mutex> lock(clear_mutex);
        clear_cv.wait(lock, [this] { return clear_requested || switch_requested || clear_stop; });
        if (clear_stop)
        {
          return;
        }
        do_clear = std::exchange(clear_requested, false);
        do_switch = std::exchange(switch_requested, false);
        modes = requested_modes;
      }

      if (do_clear)
      {
        runClearFaults();
      }
      if (do_switch)
      {
        runModeSwitch(modes);
      }
    }
  }

  void runClearFaults()
  {
    // A clear can come back ArmFaultClearIncomplete even though every fault
    // bank already reads zero: RCL samples the actuators' fault-status bits
    // before they drop. Asking again finds no actuator in fault and releases
    // the arm, so retry that one case a few times.
    Result res = rcl->Arm(kArm).ClearFaults();
    for (int attempt = 2;
         !res && res.error_code == ErrorCode::ArmFaultClearIncomplete && attempt <= kClearAttempts;
         ++attempt)
    {
      std::this_thread::sleep_for(kClearRetryDelay);
      res = rcl->Arm(kArm).ClearFaults();
    }

    // RCL only accepts ClearFaults while the arm is in Fault; asking to clear a
    // healthy arm comes back as NotAllowedInCurrentState. That is "nothing to
    // clear", which is materially different from a clear sequence that ran and
    // left faults behind - collapsing both onto Failure is what produced the
    // self-contradicting "faults remaining: (no actuator reports a non-zero
    // fault bank)" log. RCL also returns that code when the system is not
    // Operational, which is a real failure, so check the arm really is healthy.
    RclRobotDriver::ClearFaultsResult outcome = RclRobotDriver::ClearFaultsResult::Success;
    std::string msg = res.description.data();
    if (!res)
    {
      const bool nothing_to_clear = res.error_code == ErrorCode::NotAllowedInCurrentState &&
                                    rcl->GetState() == rcl::api::SystemState::Operational &&
                                    !arm_faulted.load(std::memory_order_relaxed);
      outcome = nothing_to_clear ? RclRobotDriver::ClearFaultsResult::NotFaulted
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

  // Worker-thread half of a joint-mode change. holdForModeSwitch() has already
  // taken the arm out of RealTimeJoint, so the modes can be stored safely. RCL
  // reads them on the first cycle after the arm mode is selected.
  void runModeSwitch(const std::array<JointMode, kMaxJoints> & modes)
  {
    clearVelocityAndTorqueCommands();
    for (std::size_t i = 0; i < kMaxJoints; ++i)
    {
      joint_mode[i].store(ToRcl(modes[i]), std::memory_order_relaxed);
    }

    std::string msg = ToError("SetMode(RealTimeJoint) failed", setRealtimeMode());
    if (msg.empty())
    {
      // An actuator whose firmware lacks the mode only faults once it is
      // commanded, so SetMode returning OK does not prove the switch worked.
      std::this_thread::sleep_for(kReenableSettle);
      if (arm_faulted.load(std::memory_order_relaxed))
      {
        msg = "The arm faulted within " + std::to_string(kReenableSettle.count()) +
              " ms of selecting the requested joint modes";
      }
    }

    if (!msg.empty())
    {
      rt_active.store(false);
      setAllJointsToPosition();
      if (!arm_faulted.load(std::memory_order_relaxed))
      {
        setRealtimeMode();
      }
    }

    {
      std::lock_guard<std::mutex> lock(clear_msg_mutex);
      switch_msg = msg;
    }
    switch_result.store(
      msg.empty() ? RclRobotDriver::ModeSwitchResult::Success
                  : RclRobotDriver::ModeSwitchResult::Failure,
      std::memory_order_release);
  }

  // Enter RealTimeJoint mode. Blocks until the arm is enabled.
  Result setRealtimeMode()
  {
    rt_active.store(true);
    const Result res = rcl->Arm(kArm).SetMode(ArmMode::RealTimeJoint);
    if (!res)
    {
      rt_active.store(false);
    }
    return res;
  }

  // Worker-thread half of fault recovery: reseed the command buffer from the
  // live position, re-enter real-time mode and confirm the arm stays enabled.
  // Every joint returns to position control so that a stale velocity or torque
  // command cannot resume motion. Returns "" on success or the reason it failed.
  std::string reenable()
  {
    // Drop out of real-time commanding first. The arm may still report
    // RealTimeJoint after the fault, in which case the control callback would
    // keep replaying the command buffered before the fault instead of mirroring
    // the live position into it.
    rt_active.store(false);

    ArmFeedback feedback;
    const Result fb = rcl->Arm(kArm).GetFeedback(feedback);
    if (!fb)
    {
      return ToError("Faults cleared, but GetFeedback failed", fb);
    }
    for (std::size_t i = 0; i < g_max_number_of_actuators; ++i)
    {
      cmd_deg[i].store(feedback.joints[i].position, std::memory_order_relaxed);
    }
    clearVelocityAndTorqueCommands();
    setAllJointsToPosition();
    // A joint left past its position limit (pushed there by hand in torque mode,
    // or by its own momentum) cannot be enabled in position mode: its setpoint
    // would be out of limits and RCL faults the arm again at once. It comes back
    // in torque mode at the model's static torque instead, so it can be moved
    // back into range by hand, and its position field is clamped into limits.
    for (std::size_t i = 0; i < kMaxJoints; ++i)
    {
      const double pos = feedback.joints[i].position;
      if (position_limited[i] && (pos > position_max_deg[i] || pos < position_min_deg[i]))
      {
        joint_mode[i].store(JointModeOfOperation::Torque, std::memory_order_relaxed);
      }
    }

    const Result mode = setRealtimeMode();
    if (!mode)
    {
      return ToError("Faults cleared, but SetMode failed", mode);
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
  void onControlCycle(std::span<const ArmCyclic> arms) noexcept
  {
    if (arms.empty())
    {
      return;
    }
    const ArmCyclic & arm = arms[0];
    control_cycles.fetch_add(1, std::memory_order_release);

    // Cheapest place to track the arm's fault state: read() needs it every
    // cycle and this snapshot is already in hand. FaultReaction is the servo-stop
    // ramp that precedes Fault, so commands are already pointless during it.
    const ArmState state = arm.feedback->status.state;
    arm_faulted.store(
      state == ArmState::Fault || state == ArmState::FaultReaction, std::memory_order_relaxed);

    const bool commanding = rt_active.load(std::memory_order_relaxed) &&
                            arm.feedback->status.mode == ArmMode::RealTimeJoint;

    // The span is sized to the arm's actuator count, which can be below the maximum.
    const std::span<JointCommand> commands = arm.command.joints;
    for (std::size_t i = 0; i < commands.size() && i < kMaxJoints; ++i)
    {
      const JointFeedback & joint = arm.feedback->joints[i];
      const JointModeOfOperation mode = joint_mode[i].load(std::memory_order_relaxed);

      // A safe hold for this cycle. Every field is written every cycle because
      // RCL checks the feed-forwards against the limits too. Velocity mode also
      // needs the static torque fed forward to carry the arm's weight.
      JointCommand & out = commands[i];
      out.mode = mode;
      // A joint pushed past its position limit by hand (torque mode) or by its
      // own momentum would otherwise make RCL fault the arm again on every
      // re-enable. Outside position mode this field only has to pass that check.
      out.position = (mode == JointModeOfOperation::Position || !position_limited[i])
                       ? joint.position
                       : std::clamp(joint.position, position_min_deg[i], position_max_deg[i]);
      out.velocity = 0.0;
      out.torque = (mode == JointModeOfOperation::Position) ? 0.0 : limitTorque(i, joint.static_torque);

      if (!commanding || mode != JointModeOfOperation::Position)
      {
        // Mirror the current position so the command buffer never drives a jump
        // when position control engages.
        cmd_deg[i].store(joint.position, std::memory_order_relaxed);
        last_good_deg[i] = joint.position;
      }
      if (!commanding || mode != JointModeOfOperation::Velocity)
      {
        last_velocity_deg_s[i] = 0.0;
      }
      if (!commanding || mode != JointModeOfOperation::Torque)
      {
        last_push_nm[i] = 0.0;
      }
      if (!commanding)
      {
        continue;
      }

      switch (mode)
      {
        case JointModeOfOperation::Position:
          out.position = guardedPosition(i, joint.position);
          break;
        case JointModeOfOperation::Velocity:
          out.velocity = limitedVelocity(i, joint.position);
          break;
        case JointModeOfOperation::Torque:
          out.torque = limitedTorque(i, joint);
          break;
      }
    }

    if (cyclic_log.enabled())
    {
      logCycle(arm, commands);
    }
  }

  void logCycle(const ArmCyclic & arm, std::span<const JointCommand> commands) noexcept
  {
    CyclicLog::Row * row = cyclic_log.beginPush();
    if (!row)
    {
      return;
    }
    row->unix_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                     std::chrono::system_clock::now().time_since_epoch())
                     .count();
    row->cycle = control_cycles.load(std::memory_order_relaxed);
    row->arm_state = static_cast<std::uint8_t>(arm.feedback->status.state);
    row->arm_mode = static_cast<std::uint8_t>(arm.feedback->status.mode);
    row->joints = static_cast<std::uint8_t>(std::min(commands.size(), kMaxJoints));
    for (std::size_t i = 0; i < row->joints; ++i)
    {
      const JointFeedback & fb = arm.feedback->joints[i];
      const JointCommand & cmd = commands[i];
      CyclicLog::Joint & j = row->j[i];
      j.fb_mode = static_cast<std::uint8_t>(fb.mode_of_operation);
      j.fb_position = fb.position;
      j.fb_velocity = fb.velocity;
      j.fb_torque = fb.torque;
      j.fb_current = fb.current;
      j.fb_voltage = fb.voltage;
      j.fb_static_torque = fb.static_torque;
      j.fb_following_error = fb.following_error;
      j.fault_a = fb.fault_bank_a;
      j.fault_b = fb.fault_bank_b;
      j.fault_c = fb.fault_bank_c;
      j.fault_d = fb.fault_bank_d;
      j.cmd_mode = static_cast<std::uint8_t>(cmd.mode);
      j.cmd_position = cmd.position;
      j.cmd_velocity = cmd.velocity;
      j.cmd_torque = cmd.torque;
      j.in_position = cmd_deg[i].load(std::memory_order_relaxed);
      j.in_velocity = cmd_velocity_deg_s[i].load(std::memory_order_relaxed);
      j.in_torque = cmd_torque_nm[i].load(std::memory_order_relaxed);
    }
    cyclic_log.endPush();
  }

  double guardedPosition(std::size_t i, double measured) noexcept
  {
    double cmd = cmd_deg[i].load(std::memory_order_relaxed);
    if (std::isnan(cmd))
    {
      cmd = measured;
    }
    else if (!std::isfinite(cmd) || std::fabs(cmd - measured) > kMaxCommandOffsetDeg)
    {
      last_rejected_deg[i].store(cmd, std::memory_order_relaxed);
      rejected_cmds.fetch_add(1, std::memory_order_relaxed);
      cmd = last_good_deg[i];
    }
    last_good_deg[i] = cmd;
    return cmd;
  }

  double limitTorque(std::size_t i, double torque) const noexcept
  {
    return std::clamp(torque, -torque_limit_nm[i], torque_limit_nm[i]);
  }

  double limitedVelocity(std::size_t i, double measured) noexcept
  {
    const double cmd = cmd_velocity_deg_s[i].load(std::memory_order_relaxed);
    if (!std::isfinite(cmd))
    {
      return 0.0;
    }

    const double limit = velocity_limit_deg_s[i];
    const double accel = kMaxAccelerationDegS2[i];
    // Motion toward a position limit is allowed less speed the closer the joint
    // gets: no more than it can shed at the acceleration limit before the
    // margin, and the linear ramp over the last degrees. Motion away from it is
    // never restricted.
    const auto approach_speed = [&](double remaining_deg) {
      if (remaining_deg <= 0.0)
      {
        return 0.0;
      }
      return std::min(
        kPositionLimitGain * remaining_deg, std::sqrt(2.0 * accel * remaining_deg));
    };
    const double toward_max =
      position_limited[i]
        ? approach_speed(position_max_deg[i] - kPositionLimitMarginDeg - measured)
        : limit;
    const double toward_min =
      position_limited[i]
        ? -approach_speed(measured - (position_min_deg[i] + kPositionLimitMarginDeg))
        : -limit;
    const double target = std::clamp(std::clamp(cmd, toward_min, toward_max), -limit, limit);
    if (target != cmd)
    {
      limited_cmds.fetch_add(1, std::memory_order_relaxed);
    }

    // Slew toward the target at the acceleration limit. This is the normal
    // response to any command change, so it is not counted as limiting.
    const double step = accel * kCyclePeriodS;
    const double prev = last_velocity_deg_s[i];
    const double out = std::clamp(target, prev - step, prev + step);
    last_velocity_deg_s[i] = out;
    return out;
  }

  double limitedTorque(std::size_t i, const JointFeedback & joint) noexcept
  {
    const double cmd = cmd_torque_nm[i].load(std::memory_order_relaxed);
    if (!std::isfinite(cmd))
    {
      last_push_nm[i] = 0.0;
      return limitTorque(i, joint.static_torque);
    }

    const double requested = torque_gravity_compensated ? joint.static_torque + cmd : cmd;
    const double limit = torque_limit_nm[i];
    // Torque beyond what holds the arm's weight, within the joint's limit and
    // the safety cap.
    const double cap = kMaxTorquePushNm[i];
    const double uncapped_push = std::clamp(requested, -limit, limit) - joint.static_torque;
    const double target_push = std::clamp(uncapped_push, -cap, cap);

    // Slew the push toward its target. A fresh torque-mode joint starts from
    // the static torque, so with or without gravity compensation it eases in.
    const double step = limit / kTorqueFullScaleRampS * kCyclePeriodS;
    const double prev = last_push_nm[i];
    double push = std::clamp(target_push, prev - step, prev + step);

    // Near a position limit only hold the arm's weight or push away from the
    // limit. This cut is immediate, not ramped: stopping the push toward the
    // limit takes priority over a smooth current.
    const bool at_max =
      position_limited[i] && joint.position >= position_max_deg[i] - kPositionLimitMarginDeg;
    const bool at_min =
      position_limited[i] && joint.position <= position_min_deg[i] + kPositionLimitMarginDeg;
    const bool blocked = (at_max && target_push > 0.0) || (at_min && target_push < 0.0);
    if (blocked)
    {
      push = 0.0;
    }
    last_push_nm[i] = push;

    // Only clamping, the safety cap and the limit cut count as limiting, not the ramp.
    if (blocked || std::fabs(requested) > limit || target_push != uncapped_push)
    {
      limited_cmds.fetch_add(1, std::memory_order_relaxed);
    }
    return limitTorque(i, joint.static_torque + push);
  }
};

RclRobotDriver::RclRobotDriver() : impl_(std::make_unique<Impl>()) {}
RclRobotDriver::~RclRobotDriver() = default;

std::string RclRobotDriver::init(
  const std::string & ethercat_lib_path, const std::string & topology_content,
  const ArmIdentity & arm, int net_cpu, int ctrl_cpu, bool torque_gravity_compensated)
{
  impl_->torque_gravity_compensated = torque_gravity_compensated;

  try
  {
    ArmInformation info;
    info.part_number = arm.part_number;
    info.actuator_count = arm.nb_actuators;

    Config config;
    config.ecat_config = EtherLabConfig{ethercat_lib_path};
    config.arms = {info};
    config.network_topology = topology_content;
    config.logging.type = LoggingType::Console;
    config.logging.verbosity = LoggingVerbosity::Info;
    config.thread_affinity.network_thread_cpu_core = net_cpu;
    config.thread_affinity.control_thread_cpu_core = ctrl_cpu;
    impl_->rcl = std::make_unique<RobotControlLibrary>(config);
  }
  catch (const Exception & ex)
  {
    return std::string("RobotControlLibrary construction failed: ") + ex.what();
  }
  catch (const std::exception & ex)
  {
    return std::string("RobotControlLibrary construction failed (std): ") + ex.what();
  }

  // Register the real-time control callback before the system goes Operational.
  Impl * impl = impl_.get();
  const Result cb = impl_->rcl->RegisterRealTimeControlCallback(
    [impl](std::span<const ArmCyclic> arms) { impl->onControlCycle(arms); });
  if (!cb)
  {
    return ToError("RegisterRealTimeControlCallback", cb);
  }

  impl_->system_faulted.store(false);
  const Result fault_cb =
    impl_->rcl->RegisterFaultCallback([impl]() { impl->system_faulted.store(true); });
  if (!fault_cb)
  {
    return ToError("RegisterFaultCallback", fault_cb);
  }

  impl_->startWorker();

  return {};
}

std::string RclRobotDriver::scan()
{
  if (!impl_->rcl)
  {
    return "scan called before init";
  }

  const std::string err = ToError("Network Scan", impl_->rcl->Network().Scan());
  if (!err.empty())
  {
    return err;
  }

  // Velocity and torque commands are limited against these in the control
  // callback, because RCL latches an arm fault on any command beyond them.
  ArmLimits limits;
  const Result res = impl_->rcl->Arm(kArm).Config().GetLimits(limits);
  if (!res)
  {
    return ToError("GetLimits", res);
  }
  // The joint types say which joints have position limits at all. The geometry
  // lists every joint of the chain, fixed ones included, so only the moving
  // joints are matched to the actuators, in chain order.
  ArmGeometry geometry;
  const Result geo_res = impl_->rcl->Arm(kArm).Config().GetGeometry(geometry);
  if (!geo_res)
  {
    return ToError("GetGeometry", geo_res);
  }
  std::vector<JointType> types;
  for (const JointGeometry & joint : geometry.joints)
  {
    if (joint.type != JointType::Fixed)
    {
      types.push_back(joint.type);
    }
  }

  std::ostringstream summary;
  for (std::size_t i = 0; i < kMaxJoints; ++i)
  {
    const JointType type = i < types.size() ? types[i] : JointType::Revolute;
    impl_->position_min_deg[i] = limits.joint_positions[i].minimum;
    impl_->position_max_deg[i] = limits.joint_positions[i].maximum;
    impl_->velocity_limit_deg_s[i] = limits.joint_velocities[i];
    impl_->torque_limit_nm[i] = limits.joint_torques[i];
    impl_->position_limited[i] =
      type != JointType::Continuous && impl_->position_min_deg[i] < impl_->position_max_deg[i];
    summary << "\n  joint_" << i + 1 << ": "
            << (type == JointType::Continuous ? "continuous" : "revolute") << ", position ["
            << impl_->position_min_deg[i] << ", " << impl_->position_max_deg[i] << "] deg"
            << (impl_->position_limited[i] ? "" : " (not limited)") << ", velocity "
            << impl_->velocity_limit_deg_s[i] << " deg/s, torque " << impl_->torque_limit_nm[i]
            << " Nm";
  }
  impl_->limits_summary = summary.str();
  return {};
}

std::string RclRobotDriver::setGravity(double x, double y, double z)
{
  if (!impl_->rcl)
  {
    return "setGravity called before init";
  }
  const GravityVector gravity{.x = x, .y = y, .z = z};
  return ToError("SetGravityVector", impl_->rcl->Arm(kArm).Config().SetGravityVector(gravity));
}

std::string RclRobotDriver::setNoTool()
{
  if (!impl_->rcl)
  {
    return "setNoTool called before init";
  }
  ToolConfig tool{};
  tool.transform[0][0] = 1.0;
  tool.transform[1][1] = 1.0;
  tool.transform[2][2] = 1.0;
  tool.transform[3][3] = 1.0;
  const std::vector<ToolConfig> tools{tool};
  return ToError("SetPhysicalTools", impl_->rcl->Arm(kArm).Config().SetPhysicalTools(tools));
}

std::string RclRobotDriver::startCyclic()
{
  if (!impl_->rcl)
  {
    return "startCyclic called before init";
  }
  return ToError("StartCyclic", impl_->rcl->Network().StartCyclic());
}

std::string RclRobotDriver::waitForFeedback(std::chrono::milliseconds timeout)
{
  if (!impl_->rcl)
  {
    return "waitForFeedback called before init";
  }
  // StartCyclic returns as soon as the bus reaches Op, before the first frame
  // has been exchanged; until then GetFeedback hands back a snapshot that was
  // never written (seen as values like 1.6e280 deg). Two completed control
  // cycles guarantee at least one full feedback update.
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
  const Result res = impl_->rcl->Arm(kArm).GetFeedback(feedback);
  if (!res)
  {
    return ToError("GetFeedback", res);
  }
  for (std::size_t i = 0; i < g_max_number_of_actuators; ++i)
  {
    const double p = feedback.joints[i].position;
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
  return ToError("StopCyclic", impl_->rcl->Network().StopCyclic());
}

std::string RclRobotDriver::setRealtimeJointPositionMode()
{
  if (!impl_->rcl)
  {
    return "setRealtimeJointPositionMode called before init";
  }
  impl_->clearVelocityAndTorqueCommands();
  impl_->setAllJointsToPosition();
  return ToError("SetMode(RealTimeJoint)", impl_->setRealtimeMode());
}

std::string RclRobotDriver::holdForModeSwitch()
{
  if (!impl_->rcl)
  {
    return "holdForModeSwitch called before init";
  }
  // The callback now writes a zero velocity or the static torque, so the arm
  // decelerates under RCL's servo stop instead of following the old controller.
  impl_->rt_active.store(false);
  const Result res = impl_->rcl->Arm(kArm).SetMode(ArmMode::Standby);
  if (!res)
  {
    impl_->rt_active.store(true);
  }
  return ToError("SetMode(Standby)", res);
}

std::string RclRobotDriver::setNoMode()
{
  impl_->rt_active.store(false);
  if (!impl_->rcl)
  {
    return {};
  }
  return ToError("SetMode(NoMode)", impl_->rcl->Arm(kArm).SetMode(ArmMode::NoMode));
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
  const Result res = impl_->rcl->Arm(kArm).GetFeedback(feedback);
  if (!res)
  {
    return ToError("GetFeedback", res);
  }

  for (std::size_t i = 0; i < n; ++i)
  {
    const JointFeedback & j = feedback.joints[i];
    position_deg[i] = j.position;
    velocity_deg[i] = j.velocity;
    torque_nm[i] = j.torque;
  }
  return {};
}

void RclRobotDriver::setCommand(
  const double * position_deg, const double * velocity_deg_s, const double * torque_nm,
  std::size_t n)
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
    impl_->cmd_velocity_deg_s[i].store(velocity_deg_s[i], std::memory_order_relaxed);
    impl_->cmd_torque_nm[i].store(torque_nm[i], std::memory_order_relaxed);
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

  switch (impl_->rcl->GetState())
  {
    case rcl::api::SystemState::Initialization:
      return SystemState::Initialization;
    case rcl::api::SystemState::Configuration:
      return SystemState::Configuration;
    case rcl::api::SystemState::Maintenance:
      return SystemState::Maintenance;
    case rcl::api::SystemState::Operational:
      return SystemState::Operational;
    case rcl::api::SystemState::Fault:
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
  const std::string err = ToError("Network Reset", impl_->rcl->Network().Reset());
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
  if (!impl_->rcl->GetFault(fault) || fault.IsEmpty())
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
  if (!impl_->rcl->Arm(kArm).GetFeedback(feedback))
  {
    return false;
  }
  return feedback.status.state == ArmState::Fault ||
         feedback.status.state == ArmState::FaultReaction;
}

std::string RclRobotDriver::getArmFaultDescription() const
{
  if (!impl_->rcl)
  {
    return {};
  }

  Fault fault;
  if (!impl_->rcl->Arm(kArm).GetFault(fault) || fault.IsEmpty())
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
  const Result res = impl_->rcl->Arm(kArm).GetFeedback(feedback);
  if (!res)
  {
    return ToError("GetFeedback", res);
  }

  std::ostringstream out;
  for (std::size_t i = 0; i < g_max_number_of_actuators; ++i)
  {
    const JointFeedback & j = feedback.joints[i];
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

void RclRobotDriver::requestModeSwitch(const JointMode * modes, std::size_t n)
{
  if (!impl_->rcl || !impl_->clear_worker.joinable())
  {
    impl_->switch_result.store(ModeSwitchResult::Failure, std::memory_order_release);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(impl_->clear_mutex);
    impl_->requested_modes.fill(JointMode::Position);
    std::copy_n(modes, std::min(n, kMaxJoints), impl_->requested_modes.begin());
    impl_->switch_requested = true;
  }
  impl_->switch_result.store(ModeSwitchResult::Pending, std::memory_order_release);
  impl_->clear_cv.notify_one();
}

RclRobotDriver::ModeSwitchResult RclRobotDriver::modeSwitchResult() const
{
  return impl_->switch_result.load(std::memory_order_acquire);
}

std::string RclRobotDriver::modeSwitchMessage() const
{
  std::lock_guard<std::mutex> lock(impl_->clear_msg_mutex);
  return impl_->switch_msg;
}

void RclRobotDriver::consumeModeSwitchResult()
{
  impl_->switch_result.store(ModeSwitchResult::Idle, std::memory_order_release);
}

void RclRobotDriver::getJointModes(JointMode * modes, std::size_t n) const
{
  for (std::size_t i = 0; i < std::min(n, kMaxJoints); ++i)
  {
    modes[i] = FromRcl(impl_->joint_mode[i].load(std::memory_order_relaxed));
  }
}

std::string RclRobotDriver::startCyclicLog(const std::string & path, std::size_t joints)
{
  std::string err;
  const std::string file = impl_->cyclic_log.start(path, joints, err);
  return file.empty() ? err : std::string{};
}

std::string RclRobotDriver::cyclicLogFile() const { return impl_->cyclic_log.file(); }

std::uint64_t RclRobotDriver::cyclicLogDropped() const { return impl_->cyclic_log.dropped(); }

void RclRobotDriver::recordFaultEvent(const std::string & event)
{
  if (!impl_->cyclic_log.enabled())
  {
    return;
  }
  std::string details;
  const std::string system = getSystemFaultDescription();
  if (!system.empty())
  {
    details += "system:\n" + system + "\n";
  }
  const std::string arm = getArmFaultDescription();
  if (!arm.empty())
  {
    details += "arm:\n" + arm + "\n";
  }
  details += "banks:\n" + getFaultBanks();
  impl_->cyclic_log.logEvent(impl_->control_cycles.load(std::memory_order_relaxed), event, details);
}

std::string RclRobotDriver::limitsSummary() const
{
  return impl_->limits_summary;
}

std::uint64_t RclRobotDriver::limitedCommandCount() const
{
  return impl_->limited_cmds.load(std::memory_order_relaxed);
}

}  // namespace kortex_driver
