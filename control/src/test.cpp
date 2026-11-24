#include "rclcpp/rclcpp.hpp"            // ROS 库头文件
#include "std_msgs/msg/string.hpp"    // 消息类型头文件
#include <nav_msgs/msg/odometry.hpp>
#include "igv_msgs/msg/location.hpp"
#include "igv_msgs/srv/local_path.hpp"


int main(int argc, char **argv)
{
    // 初始化 ROS 系统
    rclcpp::init(argc, argv); // 初始化 ROS 节点
    
    // 创建一个节点句柄（NodeHandle 用于与 ROS 系统进行通信）
    auto nh = std::make_shared<rclcpp::Node>("test_node");

    // 创建一个发布者（Publisher）发布到 "chatter" 话题，消息类型为 std_msgs/String
    auto odom_pub = nh->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    auto location_pub = nh->create_publisher<igv_msgs::msg::Location>("/localization", 10);

    nav_msgs::msg::Odometry temp_odom;
    temp_odom.twist.twist.linear.x = 0.5;
    temp_odom.twist.twist.angular.z = 6;
    igv_msgs::msg::Location temp_location;
    temp_location.vel.x = 0.6;
    temp_location.vel.y = 0.3;
    temp_location.vel.z = 0.3;
    temp_location.status = 1;
    // temp_location.timestamp = 
    // 设置循环频率
    rclcpp::WallRate loop_rate(50); // 10 Hz

    // 主循环，节点运行时会一直执行此循环
    while (rclcpp::ok())  // 当 ROS 系统正常工作时（例如没有收到关闭信号）
    {
       
        // 发布消息
        temp_location.timestamp = nh->now().nanoseconds() / 1000;
        temp_location.car_pose.orientation.w = 1.0;
        odom_pub->publish(temp_odom);
        location_pub->publish(temp_location);
        // 处理所有的回调函数（例如订阅者的回调）
        rclcpp::spin_some(nh);

        // 按照循环频率睡眠
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
