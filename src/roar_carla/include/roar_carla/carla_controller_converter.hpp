#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <std_msgs/msg/bool.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include "rclcpp/rclcpp.hpp"

namespace control_converter
{
    class ControlConverter : public rclcpp::Node
    {
    public:
        ControlConverter();

    private:
        carla_msgs::msg::CarlaEgoVehicleControl::SharedPtr ackermann_to_carla(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr input);
        void auto_controller_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
        void manual_controller_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
        void drive_mode_callback(const std_msgs::msg::Bool::SharedPtr msg);
        rclcpp::TimerBase::SharedPtr timer_;
        
        rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr auto_carla_control_publisher_;
        rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr manual_carla_control_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr carla_manual_override_publisher_;

        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr auto_controller_sub_;
        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr manual_controller_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_control_override_sub_;
    };

} // namespace control_converter
