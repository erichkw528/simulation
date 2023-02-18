#include <cstdio>
#include "roar_carla/carla_controller_converter.hpp"
using std::placeholders::_1;

namespace control_converter
{
    ControlConverter::ControlConverter() : Node("controller_converter")
    {
        // Initialize publisher
        auto_carla_control_publisher_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);
        manual_carla_control_publisher_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd_manual", 10);
        carla_manual_override_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/carla/ego_vehicle/vehicle_control_manual_override", 10);

        // Initialize subscribers
        auto_controller_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_drive", 10, std::bind(&ControlConverter::auto_controller_callback, this, std::placeholders::_1));
        manual_controller_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_drive_manual", 10, std::bind(&ControlConverter::manual_controller_callback, this, std::placeholders::_1));
        manual_control_override_sub_ = this->create_subscription<std_msgs::msg::Bool>("/drive_mode", 10, std::bind(&ControlConverter::drive_mode_callback, this, std::placeholders::_1));
    }

    void ControlConverter::auto_controller_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        this->auto_carla_control_publisher_->publish(*this->ackermann_to_carla(msg));
    }

    void ControlConverter::manual_controller_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        this->manual_carla_control_publisher_->publish(*this->ackermann_to_carla(msg));
    }

    void ControlConverter::drive_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        this->carla_manual_override_publisher_->publish(*msg);
    }

    carla_msgs::msg::CarlaEgoVehicleControl::SharedPtr ControlConverter::ackermann_to_carla(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr input)
    {
        carla_msgs::msg::CarlaEgoVehicleControl output;
        output.throttle = input->drive.jerk;
        output.steer = input->drive.steering_angle_velocity;
        if (input->drive.speed < 0)
        {
            output.reverse = true;
        } else 
        {
            output.reverse = false;
        }
        return std::make_shared<carla_msgs::msg::CarlaEgoVehicleControl>(output);
        
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<control_converter::ControlConverter>());
    rclcpp::shutdown();
    return 0;
}