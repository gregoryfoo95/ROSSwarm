#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <iostream>

using namespace std::chrono_literals;

class TestFlight : public rclcpp::Node
{
public:
    TestFlight() : Node("test_flight")
    {
        // Publisher for sending commands
        command_pub_ = this->create_publisher<std_msgs::msg::String>("/drone/command", 10);

        // Subscribers for status monitoring
        flight_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/drone/flight_status", 10,
            std::bind(&TestFlight::flight_status_callback, this, std::placeholders::_1));

        drone_summary_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/drone/status_summary", 10,
            std::bind(&TestFlight::drone_summary_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Test Flight initialized");
        RCLCPP_INFO(this->get_logger(), "Waiting for flight commander and state monitor...");
    }

    void send_command(const std::string& command)
    {
        RCLCPP_INFO(this->get_logger(), "Sending command: %s", command.c_str());
        auto msg = std_msgs::msg::String();
        msg.data = command;
        command_pub_->publish(msg);
    }

    bool wait_for_status(double timeout_sec = 30.0)
    {
        auto start_time = this->now();
        auto timeout_duration = rclcpp::Duration::from_seconds(timeout_sec);

        while (rclcpp::ok() && (this->now() - start_time) < timeout_duration) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(100ms);
            if (!drone_summary_.empty()) {
                return true;
            }
        }
        return false;
    }

    bool run_test_flight()
    {
        RCLCPP_INFO(this->get_logger(), "=== Starting Test Flight Sequence ===");

        // Wait for initial status
        RCLCPP_INFO(this->get_logger(), "Waiting for drone status...");
        if (!wait_for_status(10.0)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to receive drone status within timeout");
            return false;
        }

        // Step 1: Set guided mode
        RCLCPP_INFO(this->get_logger(), "Step 1: Setting GUIDED mode...");
        send_command("SET_GUIDED");
        std::this_thread::sleep_for(3s);

        // Step 2: Arm the drone
        RCLCPP_INFO(this->get_logger(), "Step 2: Arming drone...");
        send_command("ARM");
        std::this_thread::sleep_for(5s);

        // Step 3: Takeoff
        RCLCPP_INFO(this->get_logger(), "Step 3: Taking off...");
        send_command("TAKEOFF");
        std::this_thread::sleep_for(15s); // Wait for takeoff to complete

        // Step 4: Hold position
        RCLCPP_INFO(this->get_logger(), "Step 4: Holding position...");
        std::this_thread::sleep_for(10s);

        // Step 5: Land
        RCLCPP_INFO(this->get_logger(), "Step 5: Landing...");
        send_command("LAND");
        std::this_thread::sleep_for(15s); // Wait for landing to complete

        // Step 6: Disarm
        RCLCPP_INFO(this->get_logger(), "Step 6: Disarming...");
        send_command("DISARM");
        std::this_thread::sleep_for(3s);

        RCLCPP_INFO(this->get_logger(), "=== Test Flight Sequence Complete ===");
        return true;
    }

    void run_interactive_mode()
    {
        RCLCPP_INFO(this->get_logger(), "=== Interactive Flight Control ===");
        RCLCPP_INFO(this->get_logger(), "Available commands:");
        RCLCPP_INFO(this->get_logger(), "  arm, disarm, takeoff, land");
        RCLCPP_INFO(this->get_logger(), "  set_guided, set_stabilize");
        RCLCPP_INFO(this->get_logger(), "  goto X Y Z (e.g., goto 5 5 10)");
        RCLCPP_INFO(this->get_logger(), "  test (run automated test)");
        RCLCPP_INFO(this->get_logger(), "  quit (exit)");
        RCLCPP_INFO(this->get_logger(), "Enter commands:");

        std::string input;
        while (rclcpp::ok()) {
            // Process ROS callbacks
            rclcpp::spin_some(this->get_node_base_interface());

            // Check for user input
            std::cout << "> ";
            if (std::getline(std::cin, input)) {
                if (input == "quit" || input == "exit") {
                    break;
                } else if (input == "test") {
                    run_test_flight();
                } else if (!input.empty()) {
                    send_command(input);
                }
            }

            std::this_thread::sleep_for(100ms);
        }
    }

private:
    void flight_status_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        flight_status_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Flight Status: %s", msg->data.c_str());
    }

    void drone_summary_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        drone_summary_ = msg->data;

        // Only log periodically to avoid spam
        auto now = this->now();
        if (last_summary_time_.seconds() == 0 ||
            (now - last_summary_time_).seconds() > 5.0) {
            RCLCPP_INFO(this->get_logger(), "Drone Summary: %s", msg->data.c_str());
            last_summary_time_ = now;
        }
    }

    // Publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr flight_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr drone_summary_sub_;

    // Status tracking
    std::string flight_status_;
    std::string drone_summary_;
    rclcpp::Time last_summary_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Check for interactive mode
    bool interactive = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--interactive") {
            interactive = true;
            break;
        }
    }

    auto node = std::make_shared<TestFlight>();

    try {
        if (interactive) {
            node->run_interactive_mode();
        } else {
            // Automated test mode
            RCLCPP_INFO(node->get_logger(), "Running automated test flight...");
            RCLCPP_INFO(node->get_logger(), "Make sure ArduPilot simulation is running!");
            std::this_thread::sleep_for(2s);

            bool success = node->run_test_flight();
            if (success) {
                RCLCPP_INFO(node->get_logger(), "Test flight completed successfully!");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Test flight failed!");
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Test flight interrupted: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}