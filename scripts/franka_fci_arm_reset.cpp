// Read the measured pose, or return slowly to an explicitly supplied joint pose.
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; }
using Joints = std::array<double, 7>;
// FR3 limits, matching franka_vr/config/joint_limits.yaml.
constexpr Joints lower{-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159};
constexpr Joints upper{2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159};

void printArray(const Joints& values) {
  std::cout << '[';
  for (std::size_t i = 0; i < values.size(); ++i) std::cout << (i ? "," : "") << values[i];
  std::cout << ']';
}

void printState(const franka::RobotState& state, const char* event) {
  std::cout << "{\"event\":\"" << event << "\",\"wall_s\":"
            << std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()
            << ",\"q\":";
  printArray(state.q);
  std::cout << ",\"q_d\":"; printArray(state.q_d);
  std::cout << ",\"dq\":"; printArray(state.dq);
  std::cout << ",\"robot_mode\":" << static_cast<int>(state.robot_mode) << "}" << std::endl;
}

double durationFor(const Joints& start, const Joints& goal) {
  double delta = 0;
  for (std::size_t i = 0; i < 7; ++i) delta = std::max(delta, std::abs(goal[i] - start[i]));
  // Quintic profile: speed <= .1 rad/s, acceleration <= .2 rad/s^2,
  // jerk <= .5 rad/s^3, with zero speed/acceleration at both endpoints.
  return std::max({4.0, 1.875 * delta / .1, std::sqrt(5.774 * delta / .2),
                   std::cbrt(60.0 * delta / .5)});
}
}  // namespace

int main(int argc, char** argv) {
  const bool read_only = argc == 3 && std::string(argv[2]) == "--read";
  const bool reset = argc == 5 && std::string(argv[2]) == "--reset"
                     && std::string(argv[4]) == "RETURN_TO_SAVED_POSE";
  if (!read_only && !reset) {
    std::cerr << "Usage: arm_reset ROBOT_IP --read | ROBOT_IP --reset JOINTS_FILE RETURN_TO_SAVED_POSE\n";
    return 2;
  }
  std::signal(SIGINT, onSignal); std::signal(SIGTERM, onSignal);
  std::cout << std::setprecision(17);
  try {
    Joints goal{};
    if (reset) {
      std::ifstream input(argv[3]);
      for (std::size_t i = 0; i < 7; ++i) {
        if (!(input >> goal[i]) || !std::isfinite(goal[i])
            || goal[i] < lower[i] + .02 || goal[i] > upper[i] - .02) {
          throw std::runtime_error("saved pose must contain seven finite FR3 joint angles within limits");
        }
      }
      std::string extra;
      if (input >> extra) throw std::runtime_error("saved pose has extra values");
    }
    franka::Robot robot(argv[1], read_only ? franka::RealtimeConfig::kIgnore
                                         : franka::RealtimeConfig::kEnforce);
    auto state = robot.readOnce();
    printState(state, "arm_initial_state");
    if (read_only) return 0;
    if (state.robot_mode == franka::RobotMode::kReflex) {
      std::cout << "{\"event\":\"automatic_error_recovery\"}" << std::endl;
      robot.automaticErrorRecovery();
      state = robot.readOnce();
    }
    if (state.robot_mode != franka::RobotMode::kIdle) {
      throw std::runtime_error("arm must be idle before reset; stop its controller and release user stop in Desk");
    }
    double error = 0;
    for (std::size_t i = 0; i < 7; ++i) {
      if (!std::isfinite(state.q[i]) || !std::isfinite(state.dq[i]) || std::abs(state.dq[i]) > .02) {
        throw std::runtime_error("arm must be stationary with finite feedback before reset");
      }
      error = std::max(error, std::abs(state.q[i] - goal[i]));
    }
    if (error > .005) {
      Joints start{};
      double elapsed = 0, duration = 0;
      bool first = true;
      std::cout << "{\"event\":\"arm_reset_requested\",\"target_q\":";
      printArray(goal);
      std::cout << ",\"max_speed_rad_s\":0.1}" << std::endl;
      robot.control([&](const franka::RobotState& current, franka::Duration period) -> franka::JointPositions {
        if (interrupted) throw std::runtime_error("arm reset interrupted");
        if (first) {
          start = current.q_d;
          duration = durationFor(start, goal);
          first = false;
        } else {
          elapsed += period.toSec();
        }
        const double u = std::min(elapsed / duration, 1.0);
        const double s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
        Joints q{};
        for (std::size_t i = 0; i < 7; ++i) q[i] = start[i] + (goal[i] - start[i]) * s;
        franka::JointPositions command(q);
        if (u >= 1) return franka::MotionFinished(command);
        return command;
      }, franka::ControllerMode::kJointImpedance, true);
    } else {
      std::cout << "{\"event\":\"already_at_saved_pose\",\"motion_sent\":false}" << std::endl;
    }
    state = robot.readOnce();
    printState(state, "arm_final_state");
    for (std::size_t i = 0; i < 7; ++i) {
      if (std::abs(state.q[i] - goal[i]) > .02) throw std::runtime_error("arm did not reach saved pose within .02 rad");
    }
    std::cout << "{\"event\":\"arm_reset_completed\",\"success\":true}" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "FCI arm reset failed: " << e.what() << std::endl;
    return interrupted ? 130 : 1;
  }
}
