#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

class SimpleDroneMonitor : public rclcpp::Node
{
public:
    SimpleDroneMonitor() : Node("simple_drone_monitor")
    {
        // Publishers
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/drone/status_summary", 10);

        // Subscribers for basic sensor data
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/drone/pose", 10,
            std::bind(&SimpleDroneMonitor::pose_callback, this, std::placeholders::_1));

        velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/drone/velocity", 10,
            std::bind(&SimpleDroneMonitor::velocity_callback, this, std::placeholders::_1));

        battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "/drone/battery", 10,
            std::bind(&SimpleDroneMonitor::battery_callback, this, std::placeholders::_1));

        // Timer for periodic status updates
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&SimpleDroneMonitor::publish_status_summary, this));

        RCLCPP_INFO(this->get_logger(), "Simple Drone Monitor initialized");
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        last_pose_time_ = this->now();
    }

    void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        current_velocity_ = *msg;
        last_velocity_time_ = this->now();
    }

    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        current_battery_ = *msg;
        last_battery_time_ = this->now();
    }

    void publish_status_summary()
    {
        std::stringstream ss;

        // Position info
        ss << "Pos: [" << std::fixed << std::setprecision(2)
           << current_pose_.pose.position.x << ", "
           << current_pose_.pose.position.y << ", "
           << current_pose_.pose.position.z << "], ";

        // Velocity info
        ss << "Vel: [" << std::fixed << std::setprecision(2)
           << current_velocity_.twist.linear.x << ", "
           << current_velocity_.twist.linear.y << ", "
           << current_velocity_.twist.linear.z << "], ";

        // Battery info
        if (current_battery_.percentage > 0) {
            ss << "Battery: " << std::fixed << std::setprecision(1)
               << (current_battery_.percentage * 100.0) << "%, ";
        } else {
            ss << "Battery: N/A, ";
        }

        // System status
        ss << "Status: ACTIVE";

        auto msg = std_msgs::msg::String();
        msg.data = ss.str();
        status_pub_->publish(msg);
    }

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Current state variables
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::TwistStamped current_velocity_;
    sensor_msgs::msg::BatteryState current_battery_;

    // Timestamp tracking
    rclcpp::Time last_pose_time_;
    rclcpp::Time last_velocity_time_;
    rclcpp::Time last_battery_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleDroneMonitor>());
    rclcpp::shutdown();
    return 0;
}