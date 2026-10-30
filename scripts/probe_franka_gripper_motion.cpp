// Gripper-only diagnostic. No arm connection; homing is explicitly opt-in.
// Requires the explicit GRIPPER_MOTION token. Keep the finger travel clear.
#include <franka/gripper.h>
#include <franka/gripper_state.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; }
using Clock = std::chrono::steady_clock;

double environmentDouble(const char* name, double fallback) {
  const char* text = std::getenv(name);
  if (!text || !*text) return fallback;
  std::size_t parsed = 0;
  const double value = std::stod(text, &parsed);
  if (parsed != std::string(text).size() || !std::isfinite(value)) {
    throw std::runtime_error(std::string("invalid ") + name);
  }
  return value;
}

void sample(franka::Gripper& gripper, const std::string& phase) {
  const auto state = gripper.readOnce();
  std::cout << "{\"event\":\"state\",\"phase\":" << std::quoted(phase)
            << ",\"wall_s\":" << std::chrono::duration<double>(
                   std::chrono::system_clock::now().time_since_epoch()).count()
            << ",\"width_m\":" << state.width << ",\"max_width_m\":" << state.max_width
            << ",\"hardware_time_ms\":" << state.time.toMSec()
            << ",\"temperature_c\":" << state.temperature
            << ",\"is_grasped\":" << (state.is_grasped ? "true" : "false") << "}" << std::endl;
}

void record(franka::Gripper& gripper, const std::string& phase, double seconds) {
  const auto deadline = Clock::now() + std::chrono::duration<double>(seconds);
  while (!interrupted && Clock::now() < deadline) {
    sample(gripper, phase);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
}  // namespace

int main(int argc, char** argv) {
  const bool read_only = argc == 3 && std::string(argv[2]) == "READ_ONLY";
  if (!read_only && ((argc != 3 && argc != 4) || std::string(argv[2]) != "GRIPPER_MOTION")) {
    std::cerr << "Usage: probe_franka_gripper_motion ROBOT_IP READ_ONLY\n"
              << "   or: probe_franka_gripper_motion ROBOT_IP GRIPPER_MOTION [single_width_m|HOMING]\n";
    return 2;
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  std::cout << std::setprecision(17);
  try {
    franka::Gripper gripper(argv[1]);
    const bool do_homing = argc == 4 && std::string(argv[3]) == "HOMING";
    const double width_offset = environmentDouble("FCI_GRIPPER_WIDTH_OFFSET", 0.07);
    const double effective_open_width = environmentDouble("FCI_GRIPPER_OPEN_WIDTH", 0.07);
    if (width_offset < 0 || effective_open_width <= 0 || width_offset + effective_open_width > 0.15) {
      throw std::runtime_error("invalid gripper offset/open-width calibration");
    }
    std::cout << "{\"event\":\"connected\",\"robot_ip\":" << std::quoted(argv[1])
              << ",\"protocol_version\":" << gripper.serverVersion()
              << ",\"homing_requested\":" << (do_homing ? "true" : "false")
              << ",\"arm_commands_sent\":false}" << std::endl;
    record(gripper, "BASELINE", 2.0);
    if (read_only) return 0;
    std::vector<std::pair<const char*, double>> steps = {
        {"OPEN_1", 0.07}, {"CLOSE", 0.01}, {"OPEN_2", 0.07}};
    if (do_homing) {
      steps = {{"HOMING", 0.0}};
    } else if (argc == 4) {
      std::size_t parsed = 0;
      const double target = std::stod(argv[3], &parsed);
      const auto state = gripper.readOnce();
      if (parsed != std::string(argv[3]).size() || !std::isfinite(target)
          || target < 0 || target > 0.15 || !std::isfinite(state.max_width)
          || target > state.max_width) {
        std::cerr << "single width must be finite, in [0, 0.15], and within current hardware max_width\n";
        return 2;
      }
      steps = {{"SINGLE_MOVE", target}};
    }
    for (const auto& step : steps) {
      if (interrupted) { gripper.stop(); return 130; }
      if (do_homing) {
        std::cout << "{\"event\":\"homing_requested\",\"phase\":\"HOMING\"}" << std::endl;
      } else {
        std::cout << "{\"event\":\"move_requested\",\"phase\":" << std::quoted(step.first)
                  << ",\"target_width_m\":" << step.second << ",\"speed_m_s\":0.01}" << std::endl;
      }
      const auto started = Clock::now();
      std::pair<bool, std::string> outcome;
      try {
        // libfranka's Gripper connection is not thread-safe. In particular,
        // readOnce() must never run concurrently with homing()/move(): doing so
        // interleaves protocol replies and produced impossible alternating
        // widths/max_width values on this robot. The outer process timeout is
        // the operation watchdog; all calls on this connection stay serial.
        outcome = {do_homing ? gripper.homing() : gripper.move(step.second, 0.01), std::string()};
      } catch (const std::exception& e) {
        outcome = {false, e.what()};
      }
      const double elapsed = std::chrono::duration<double>(Clock::now() - started).count();
      std::cout << "{\"event\":\"move_result\",\"phase\":" << std::quoted(step.first)
                << ",\"success\":" << (outcome.first ? "true" : "false")
                << ",\"elapsed_s\":" << elapsed
                << ",\"timed_out\":false"
                << ",\"error\":" << std::quoted(outcome.second) << "}" << std::endl;
      if (!outcome.first || interrupted) return 1;
      record(gripper, std::string(step.first) + "_SETTLED", 3.0);
      if (do_homing) {
        // This gripper firmware reports max_width in two coordinate systems
        // during the few seconds after homing (about 0.071 m, then the raw
        // 0.142 m range).  Clamping a raw command with that transient effective
        // value closes the fingers instead of opening them.  The calibrated
        // raw target was validated above to be <= 0.15 m; command it directly
        // and require settled feedback validation in fci_init.sh.
        const double open_width = width_offset + effective_open_width;
        std::cout << "{\"event\":\"move_requested\",\"phase\":\"OPEN_AFTER_HOMING\""
                  << ",\"target_width_m\":" << open_width
                  << ",\"width_offset_m\":" << width_offset
                  << ",\"effective_open_width_m\":" << effective_open_width
                  << ",\"speed_m_s\":0.01}" << std::endl;
        const auto open_started = Clock::now();
        bool opened = false;
        std::string open_error;
        try { opened = gripper.move(open_width, 0.01); }
        catch (const std::exception& e) { open_error = e.what(); }
        std::cout << "{\"event\":\"move_result\",\"phase\":\"OPEN_AFTER_HOMING\""
                  << ",\"success\":" << (opened ? "true" : "false")
                  << ",\"elapsed_s\":" << std::chrono::duration<double>(Clock::now() - open_started).count()
                  << ",\"timed_out\":false,\"error\":" << std::quoted(open_error) << "}" << std::endl;
        if (!opened || interrupted) return 1;
        record(gripper, "OPEN_AFTER_HOMING_SETTLED", 3.0);
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "gripper diagnostic failed: " << e.what() << std::endl;
    return 1;
  }
}
