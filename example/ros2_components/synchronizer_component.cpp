// synchronizer_component.cpp
#include "infinite_sense.h"
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using namespace infinite_sense;

class SynchronizerComponent : public rclcpp::Node
{
public:
    SynchronizerComponent(const rclcpp::NodeOptions & options)
    : Node("synchronizer_node", options)
    {
        RCLCPP_INFO(get_logger(), "Initializing SynchronizerComponent...");

        // Set up the serial connection
        synchronizer_.SetSerialLink("/dev/ttyACM0", 460800);

        // Optionally: SetNetLink or UseMvCam
        // synchronizer_.SetNetLink("192.168.1.188", 8888);
        // synchronizer_.UseMvCam();

        synchronizer_.Start();
        Synchronizer::PrintSummary();

        // Setup a timer instead of while loop
        timer_ = this->create_wall_timer(1s, [this]() {
            RCLCPP_INFO(this->get_logger(), "Running...");
            // You can add status logging or health check here
        });
    }

    ~SynchronizerComponent()
    {
        synchronizer_.Stop();
    }

private:
    Synchronizer synchronizer_;
    rclcpp::TimerBase::SharedPtr timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(SynchronizerComponent)
