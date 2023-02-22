#include "roar_carla/carla_status_monitor_node.hpp"

using namespace std::chrono_literals;
using namespace roar::simulation::roar_carla;


StatusMonitor::StatusMonitor()
    : Node("status_monitor_node")
{
  this->status_publisher_ = create_publisher<std_msgs::msg::Bool>("/status", 10);
  this->status_publisher_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&StatusMonitor::publish_status, this));

  // configure all subscribers
  subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/carla/ego_vehicle/odometry", 10, std::bind(&StatusMonitor::on_odom_received, this, std::placeholders::_1));
  max_odom_received_interval = 1.0;

}

bool StatusMonitor::is_all_components_within_max_interval()
{
  float now = this->get_clock()->now().seconds();
  if ((now - this->last_odom_received) > this->max_odom_received_interval)
  {
    return false;
  }
  return true;
  
}

void StatusMonitor::publish_status()
{
  std_msgs::msg::Bool msg;
  msg.data = this->is_all_components_within_max_interval();
  this->status_publisher_->publish(msg);
}

void StatusMonitor::on_odom_received(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  this->last_odom_received = this->get_clock()->now().seconds();
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roar::simulation::roar_carla::StatusMonitor>());
  rclcpp::shutdown();
  return 0;
}