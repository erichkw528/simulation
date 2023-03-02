#ifndef STATUS_MONITOR_NODE__CARLA_STATUS_MONITOR_NODE_HPP_
#define STATUS_MONITOR_NODE__CARLA_STATUS_MONITOR_NODE_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <std_msgs/msg/bool.hpp>
#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>

namespace roar 
{
    namespace simulation 
    {
        namespace roar_carla
        {
            class StatusMonitor : public rclcpp::Node
            {
            public:
                StatusMonitor();    
                bool is_all_components_within_max_interval();

            private:
                void publish_status();
                rclcpp::TimerBase::SharedPtr status_publisher_timer;
                rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_publisher_;

                void on_odom_received(const nav_msgs::msg::Odometry::SharedPtr msg);
                rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
                double last_odom_received;
                double max_odom_received_interval;
            };

        } // namepsace roar_carla
    } // namespace simulation
}

#endif // STATUS_MONITOR_NODE__CARLA_STATUS_MONITOR_NODE_HPP_
