#include "infinite_sense.h"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <fstream>
#include <thread>

namespace infinite_sense {
class CustomCam final : public Sensor {
 public:
  ~CustomCam() override { Stop(); }
  bool Initialization() override { return true; }
  void Stop() override {}
  void Start() override {}

 private:
  void Receive(void *handle, const std::string &) override {
    // Dummy implementation to satisfy linker
  }
};
}  // namespace infinite_sense

using namespace std::chrono_literals;
using namespace infinite_sense;

class SynchronizerNode : public rclcpp::Node {
 public:
  SynchronizerNode() : Node("synchronizer_node") {
    RCLCPP_INFO(get_logger(), "Initializing SynchronizerNode...");

    synchronizer_.SetUsbLink("/dev/ttyACM0", 921600);
    // synchronizer_.SetNetLink("192.168.1.188", 8888);

    auto mv_cam = std::make_shared<CustomCam>();
    mv_cam->SetParams({
        {"basler_middle", CAM_1},
        {"basler_left", CAM_1},
    });
    synchronizer_.UseSensor(mv_cam);

    synchronizer_.Start();
    RCLCPP_INFO(this->get_logger(), "Synchronizer started!");

    log_file_.open("last_trigger_times.log", std::ios::out | std::ios::trunc);
    if (!log_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open log file for trigger times!");
    } else {
      log_file_ << "Trigger times log:\n====================\n";
    }

    timer_ = this->create_wall_timer(1s, [this]() {
      RCLCPP_INFO(this->get_logger(), "Logging trigger times...");
      log_file_ << "Trigger log at: " << this->get_clock()->now().nanoseconds() << " ns\n";

      for (const auto &[name, cam_id] : std::map<std::string, TriggerDevice>{
               {"basler_middle", CAM_1},
               {"basler_left", CAM_1},
           }) {
        uint64_t trigger_time = 0;
        if (GET_LAST_TRIGGER_STATUS(cam_id, trigger_time)) {
          log_file_ << name << ": " << trigger_time << " us\n";
        } else {
          log_file_ << name << ": (no trigger yet)\n";
        }
      }
    });
  }

  ~SynchronizerNode() {
    RCLCPP_INFO(this->get_logger(), "Closing log file...");
    if (log_file_.is_open()) {
      log_file_.close();
    }
    RCLCPP_INFO(this->get_logger(), "Shutting down synchronizer...");
    synchronizer_.Stop();
    RCLCPP_INFO(this->get_logger(), "SynchronizerNode shutdown complete.");
  }

 private:
  Synchronizer synchronizer_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream log_file_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SynchronizerNode>());
  rclcpp::shutdown();
  return 0;
}
