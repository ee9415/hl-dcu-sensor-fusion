
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

extern "C" {
#include "pbv_dcu/can_interface.h"
#include "pbv_dcu/config.h"
#include "pbv_dcu/decision_control.h"
#include "pbv_dcu/fault_management.h"
#include "pbv_dcu/log.h"
#include "pbv_dcu/safety_supervisor.h"
#include "pbv_dcu/socketcan_io.h"
#include "pbv_dcu/state_manager.h"
}

#include "rclcpp/rclcpp.hpp"
#include "pbv_dcu_interfaces/msg/can_tx.hpp"
#include "pbv_dcu_interfaces/msg/control_command.hpp"
#include "pbv_dcu_interfaces/msg/fault_status.hpp"
#include "pbv_dcu_interfaces/msg/perception_snapshot.hpp"
#include "pbv_dcu_interfaces/msg/safety_status.hpp"
#include "pbv_dcu_interfaces/msg/state_status.hpp"

namespace {
builtin_interfaces::msg::Time to_builtin_time(const rclcpp::Time & stamp) {
  builtin_interfaces::msg::Time out;
  const int64_t ns = stamp.nanoseconds();
  out.sec = static_cast<int32_t>(ns / 1000000000LL);
  out.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return out;
}

uint64_t time_msg_to_ms(const builtin_interfaces::msg::Time & stamp) {
  return static_cast<uint64_t>(stamp.sec) * 1000ULL + static_cast<uint64_t>(stamp.nanosec) / 1000000ULL;
}

PbvPerceptionSnapshot to_c_snapshot(const pbv_dcu_interfaces::msg::PerceptionSnapshot & msg, uint64_t fallback_now_ms) {
  PbvPerceptionSnapshot out;
  std::memset(&out, 0, sizeof(out));
  out.stamp_ms = time_msg_to_ms(msg.stamp);
  if (out.stamp_ms == 0ULL) {
    out.stamp_ms = fallback_now_ms;
  }
  out.seq = msg.seq;
  out.lane_offset_m = msg.lane_offset_m;
  out.lane_heading_rad = msg.lane_heading_rad;
  out.lane_confidence = msg.lane_confidence;
  out.target_distance_m = msg.target_distance_m;
  out.stop_line_distance_m = msg.stop_line_distance_m;
  out.traffic_light_confidence = msg.traffic_light_confidence;
  out.frame_fps = msg.frame_fps;
  out.frame_age_ms = msg.frame_age_ms;
  out.camera_ok = msg.camera_ok;
  out.lane_valid = msg.lane_valid;
  out.target_valid = msg.target_valid;
  out.traffic_light_state = static_cast<PbvTrafficLightState>(msg.traffic_light_state);
  out.flags = msg.flags;
  return out;
}

pbv_dcu_interfaces::msg::FaultStatus to_fault_msg(const PbvFaultOutput & fault, const rclcpp::Time & stamp) {
  pbv_dcu_interfaces::msg::FaultStatus msg;
  msg.stamp = to_builtin_time(stamp);
  msg.fault_bits = fault.fault_bits;
  msg.sensor_health = static_cast<uint8_t>(fault.sensor_health);
  msg.requested_gate = static_cast<uint8_t>(fault.requested_gate);
  msg.fail_safe = fault.fail_safe;
  msg.stale_counter = fault.stale_counter;
  msg.lost_counter = fault.lost_counter;
  return msg;
}

pbv_dcu_interfaces::msg::StateStatus to_state_msg(const PbvStateOutput & state, const rclcpp::Time & stamp) {
  pbv_dcu_interfaces::msg::StateStatus msg;
  msg.stamp = to_builtin_time(stamp);
  msg.drive_state = static_cast<uint8_t>(state.drive_state);
  msg.behavior_gate = static_cast<uint8_t>(state.behavior_gate);
  msg.hold_timer_ms = state.hold_timer_ms;
  msg.ready = state.ready;
  msg.stop_requested = state.stop_requested;
  return msg;
}

pbv_dcu_interfaces::msg::SafetyStatus to_safety_msg(const PbvSafetyOutput & safety, const rclcpp::Time & stamp) {
  pbv_dcu_interfaces::msg::SafetyStatus msg;
  msg.stamp = to_builtin_time(stamp);
  msg.enforced_gate = static_cast<uint8_t>(safety.enforced_gate);
  msg.override_stop = safety.override_stop;
  msg.max_speed_mps = safety.max_speed_mps;
  msg.max_steer_norm = safety.max_steer_norm;
  msg.intervention_bits = safety.intervention_bits;
  return msg;
}

pbv_dcu_interfaces::msg::ControlCommand to_cmd_msg(
  const PbvControlCommand & cmd,
  const PbvStateOutput & state,
  const PbvFaultOutput & fault,
  const PbvSafetyOutput & safety,
  const rclcpp::Time & stamp)
{
  pbv_dcu_interfaces::msg::ControlCommand msg;
  msg.stamp = to_builtin_time(stamp);
  msg.target_speed_mps = cmd.target_speed_mps;
  msg.steering_norm = cmd.steering_norm;
  msg.brake_norm = cmd.brake_norm;
  msg.source = static_cast<uint8_t>(cmd.source);
  msg.heartbeat = cmd.heartbeat;
  msg.drive_state = static_cast<uint8_t>(state.drive_state);
  msg.gate = static_cast<uint8_t>(safety.enforced_gate);
  msg.fault_bits = fault.fault_bits;
  msg.intervention_bits = safety.intervention_bits;
  return msg;
}

pbv_dcu_interfaces::msg::CanTx to_can_msg(const PbvCanFrame & frame, const rclcpp::Time & stamp) {
  pbv_dcu_interfaces::msg::CanTx msg;
  msg.stamp = to_builtin_time(stamp);
  msg.can_id = frame.id;
  msg.dlc = frame.dlc;
  std::fill(msg.data.begin(), msg.data.end(), 0U);
  for (std::size_t i = 0; i < frame.dlc && i < msg.data.size(); ++i) {
    msg.data[i] = frame.data[i];
  }
  return msg;
}
}  // namespace

class PbvDcuPipelineNode : public rclcpp::Node {
public:
  PbvDcuPipelineNode()
  : Node("pbv_dcu_pipeline_node"),
    system_clock_(RCL_SYSTEM_TIME),
    have_snapshot_(false),
    new_frame_pending_(false),
    enable_csv_log_(false),
    last_can_tx_ms_(0ULL)
  {
    this->declare_parameter<std::string>("config_path", "");
    this->declare_parameter<bool>("use_socketcan", false);
    this->declare_parameter<std::string>("can_if", "vcan0");
    this->declare_parameter<int>("pipeline_period_ms", 20);
    this->declare_parameter<bool>("enable_csv_log", false);
    this->declare_parameter<std::string>("log_path", "logs/feasibility_dcu_ros2.csv");

    load_config();
    init_core();

    snapshot_sub_ = this->create_subscription<pbv_dcu_interfaces::msg::PerceptionSnapshot>(
      "/pbv/perception/snapshot", 10,
      std::bind(&PbvDcuPipelineNode::on_snapshot, this, std::placeholders::_1));

    fault_pub_ = this->create_publisher<pbv_dcu_interfaces::msg::FaultStatus>("/pbv/fault/status", 10);
    state_pub_ = this->create_publisher<pbv_dcu_interfaces::msg::StateStatus>("/pbv/state/status", 10);
    safety_pub_ = this->create_publisher<pbv_dcu_interfaces::msg::SafetyStatus>("/pbv/safety/status", 10);
    control_pub_ = this->create_publisher<pbv_dcu_interfaces::msg::ControlCommand>("/pbv/control/command", 10);
    can_pub_ = this->create_publisher<pbv_dcu_interfaces::msg::CanTx>("/pbv/can/tx", 20);

    const auto pipeline_period_ms = std::max<int64_t>(5, this->get_parameter("pipeline_period_ms").as_int());
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(pipeline_period_ms),
      std::bind(&PbvDcuPipelineNode::on_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "pbv_dcu_pipeline_node started: use_socketcan=%d can_if=%s period_ms=%ld csv_log=%d",
      cfg_.use_socketcan ? 1 : 0,
      cfg_.can_if,
      pipeline_period_ms,
      enable_csv_log_ ? 1 : 0);
  }

  ~PbvDcuPipelineNode() override {
    pbv_can_close(&can_ctx_);
    if (enable_csv_log_) {
      pbv_log_close(&logger_);
    }
  }

private:
  void load_config() {
    std::memset(&cfg_, 0, sizeof(cfg_));
    pbv_config_set_defaults(&cfg_);

    const auto config_path = this->get_parameter("config_path").as_string();
    if (!config_path.empty()) {
      if (pbv_config_load_file(&cfg_, config_path.c_str()) != 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to load config file: %s. Using defaults.", config_path.c_str());
      }
    }

    cfg_.use_socketcan = this->get_parameter("use_socketcan").as_bool();
    const auto can_if = this->get_parameter("can_if").as_string();
    std::snprintf(cfg_.can_if, sizeof(cfg_.can_if), "%s", can_if.c_str());

    enable_csv_log_ = this->get_parameter("enable_csv_log").as_bool();
    const auto log_path = this->get_parameter("log_path").as_string();
    if (!log_path.empty()) {
      std::snprintf(cfg_.log_path, sizeof(cfg_.log_path), "%s", log_path.c_str());
    }
  }

  void init_core() {
    const auto now_ms = system_now_ms();
    std::memset(&fault_ctx_, 0, sizeof(fault_ctx_));
    std::memset(&state_ctx_, 0, sizeof(state_ctx_));
    std::memset(&safety_ctx_, 0, sizeof(safety_ctx_));
    std::memset(&decision_ctx_, 0, sizeof(decision_ctx_));
    std::memset(&can_ctx_, 0, sizeof(can_ctx_));
    std::memset(&logger_, 0, sizeof(logger_));
    std::memset(&fault_out_, 0, sizeof(fault_out_));
    std::memset(&state_out_, 0, sizeof(state_out_));
    std::memset(&safety_out_, 0, sizeof(safety_out_));
    std::memset(&cmd_out_, 0, sizeof(cmd_out_));
    std::memset(&last_feedback_, 0, sizeof(last_feedback_));
    std::memset(&last_snapshot_, 0, sizeof(last_snapshot_));

    pbv_fault_init(&fault_ctx_);
    pbv_state_init(&state_ctx_, now_ms);
    pbv_safety_init(&safety_ctx_);
    pbv_decision_init(&decision_ctx_);
    pbv_can_init(&can_ctx_, false);

    if (cfg_.use_socketcan && pbv_can_open(&can_ctx_, &cfg_) != 0) {
      RCLCPP_WARN(this->get_logger(), "SocketCAN open failed for %s. Continuing topic-only.", cfg_.can_if);
      cfg_.use_socketcan = false;
      can_ctx_.use_socketcan = false;
      can_ctx_.socket_fd = -1;
    }

    if (enable_csv_log_) {
      if (pbv_log_open(&logger_, cfg_.log_path) != 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to open csv log at %s", cfg_.log_path);
        enable_csv_log_ = false;
      }
    }
  }

  uint64_t system_now_ms() {
    return static_cast<uint64_t>(system_clock_.now().nanoseconds() / 1000000ULL);
  }

  void on_snapshot(const pbv_dcu_interfaces::msg::PerceptionSnapshot::SharedPtr msg) {
    last_snapshot_ = to_c_snapshot(*msg, system_now_ms());
    have_snapshot_ = true;
    new_frame_pending_ = true;
  }

  void on_timer() {
    if (!have_snapshot_) {
      return;
    }

    const auto now = system_clock_.now();
    const auto now_ms = static_cast<uint64_t>(now.nanoseconds() / 1000000ULL);
    if (now_ms > last_snapshot_.stamp_ms) {
      last_snapshot_.frame_age_ms = static_cast<uint32_t>(now_ms - last_snapshot_.stamp_ms);
    }

    const bool new_frame = std::exchange(new_frame_pending_, false);

    fault_out_ = pbv_fault_update(&fault_ctx_, &cfg_, &last_snapshot_, now_ms, new_frame);
    state_out_ = pbv_state_update(&state_ctx_, &cfg_, &last_snapshot_, &fault_out_, now_ms);
    safety_out_ = pbv_safety_update(&safety_ctx_, &cfg_, &last_snapshot_, &fault_out_, &state_out_);
    cmd_out_ = pbv_decision_update(&decision_ctx_, &cfg_, &last_snapshot_, &fault_out_, &state_out_, &safety_out_);

    fault_pub_->publish(to_fault_msg(fault_out_, now));
    state_pub_->publish(to_state_msg(state_out_, now));
    safety_pub_->publish(to_safety_msg(safety_out_, now));
    control_pub_->publish(to_cmd_msg(cmd_out_, state_out_, fault_out_, safety_out_, now));

    if (now_ms - last_can_tx_ms_ >= cfg_.heartbeat_period_ms) {
      emit_can(now, now_ms);
      last_can_tx_ms_ = now_ms;
    }

    if (enable_csv_log_) {
      PbvCanFeedback *feedback_ptr = last_feedback_.valid ? &last_feedback_ : nullptr;
      pbv_log_write(&logger_, now_ms, &last_snapshot_, &fault_out_, &state_out_, &safety_out_, &cmd_out_, feedback_ptr);
    }
  }

  void emit_can(const rclcpp::Time & stamp, uint64_t now_ms) {
    PbvCanFrame speed{};
    PbvCanFrame steer{};
    PbvCanFrame diag{};

    if (pbv_can_build_command_frames(&can_ctx_, &cmd_out_, &fault_out_, &state_out_, &safety_out_, &speed, &steer, &diag) != 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to build CAN frames.");
      return;
    }

    can_pub_->publish(to_can_msg(speed, stamp));
    can_pub_->publish(to_can_msg(steer, stamp));
    can_pub_->publish(to_can_msg(diag, stamp));

    if (cfg_.use_socketcan && can_ctx_.socket_fd >= 0) {
      int rc = 0;
      rc |= pbv_socketcan_send_frame(can_ctx_.socket_fd, &speed);
      rc |= pbv_socketcan_send_frame(can_ctx_.socket_fd, &steer);
      rc |= pbv_socketcan_send_frame(can_ctx_.socket_fd, &diag);
      if (rc != 0) {
        fault_out_.fault_bits |= PBV_FAULT_CAN_TX;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "SocketCAN transmit failed.");
      }
      PbvCanFeedback feedback{};
      if (pbv_can_poll_feedback(&can_ctx_, &feedback)) {
        last_feedback_ = feedback;
      }
    }

    (void)now_ms;
  }

  rclcpp::Clock system_clock_;
  PbvDcuConfig cfg_{};
  PbvFaultContext fault_ctx_{};
  PbvStateContext state_ctx_{};
  PbvSafetyContext safety_ctx_{};
  PbvDecisionContext decision_ctx_{};
  PbvCanInterfaceContext can_ctx_{};
  PbvCsvLogger logger_{};
  PbvPerceptionSnapshot last_snapshot_{};
  PbvFaultOutput fault_out_{};
  PbvStateOutput state_out_{};
  PbvSafetyOutput safety_out_{};
  PbvControlCommand cmd_out_{};
  PbvCanFeedback last_feedback_{};

  bool have_snapshot_;
  bool new_frame_pending_;
  bool enable_csv_log_;
  uint64_t last_can_tx_ms_;

  rclcpp::Subscription<pbv_dcu_interfaces::msg::PerceptionSnapshot>::SharedPtr snapshot_sub_;
  rclcpp::Publisher<pbv_dcu_interfaces::msg::FaultStatus>::SharedPtr fault_pub_;
  rclcpp::Publisher<pbv_dcu_interfaces::msg::StateStatus>::SharedPtr state_pub_;
  rclcpp::Publisher<pbv_dcu_interfaces::msg::SafetyStatus>::SharedPtr safety_pub_;
  rclcpp::Publisher<pbv_dcu_interfaces::msg::ControlCommand>::SharedPtr control_pub_;
  rclcpp::Publisher<pbv_dcu_interfaces::msg::CanTx>::SharedPtr can_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PbvDcuPipelineNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
