#include "rclcpp/rclcpp.hpp"            // ROS 库头文件
#include "std_msgs/msg/string.hpp"    // 消息类型头文件
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "igv_msgs/msg/control_information.hpp"
#include <igv_msgs/msg/control_status.hpp>
#include <algorithm>
#include <cmath>


nav_msgs::msg::Odometry current_odom;
igv_msgs::msg::ControlInformation control_result;
igv_msgs::msg::ControlStatus control_status;
bool is_stop = false;
rclcpp::Node::SharedPtr node;

void receive_odom_data(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{           
    current_odom.twist.twist.linear.x =  msg->twist.twist.linear.x;
    current_odom.twist.twist.angular.z = msg->twist.twist.angular.z *3.14159/180;       
}

void receive_escape_topic_data(const std_msgs::msg::String::ConstSharedPtr& msg, rclcpp::Time& receive_time)
{
    if(!is_stop)
    {
        is_stop = true;
        receive_time = node->now();
    }else{
        RCLCPP_INFO(rclcpp::get_logger("control"), "IS STOPING!");
    }
    
    RCLCPP_INFO(rclcpp::get_logger("control"), "escape_topic_data: %s", msg->data.c_str());
}

double SpeedToZero(double ns)
{
    auto now_speed = ns;
    double speed_commond = 0.0;
    double minus_acc = 1.0,k_minus_stop_speed = 0.2;
    

    if(now_speed >= 0.5)
    {
        speed_commond = now_speed - minus_acc;
        speed_commond = std::clamp(speed_commond,0.0,2.0);
    }
    else if(now_speed <= -0.5)
    {
        speed_commond = now_speed + minus_acc;
        speed_commond = std::clamp(speed_commond,-0.5,0.0);
    }
    else
    {
        speed_commond = now_speed * k_minus_stop_speed;
    }
        
    if(fabs(speed_commond)<0.1)
    {
        speed_commond = 0.0;
    }    

    return (int(speed_commond * 10.0)) /10.0 ;
}

double brk_press_check(double speed_cmd, double ns, const rclcpp::Time& receive_time)
{           
    double brk_press = 0;
    auto now_speed = ns;
    // if(fabs(now_speed) >= 1.0)
    // {
    //     brk_press = 0.0;
    // }
    // else if (fabs(now_speed) < 1.0)
    // {
    //     if(fabs(now_speed) < 0.05)
    //     {
    //         brk_press = 3.0;
    //     }else
    //     {
    //         brk_press = 0.05 / fabs(now_speed) + 1.0 - now_speed;
    //     }        
    //     brk_press = std::clamp(brk_press,0.0,3.0);
    // }         

    brk_press = 0.1 * (node->now() - receive_time).seconds()/0.02;
    brk_press = std::clamp(brk_press,0.0,3.0);
    return brk_press;
}
void clear(double nowspeed, const rclcpp::Time& receive_time)
{
    control_result.speed = SpeedToZero(nowspeed);
    control_result.angle = 0;
    control_result.torque = 0;
    control_result.brk_press = brk_press_check(control_result.speed, nowspeed, receive_time);
    control_result.torque_rpm_mode = 1;       
}

int main(int argc, char **argv)
{
    // 初始化 ROS 系统
    rclcpp::init(argc, argv); // 初始化 ROS 节点
    
    // 创建一个节点句柄（NodeHandle 用于与 ROS 系统进行通信）
    auto nh = std::make_shared<rclcpp::Node>("stop_test_node");

    node = nh;
    rclcpp::Time receive_time = node->now();

    auto odom_sub = nh->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, &receive_odom_data);
    auto escape_sub = nh->create_subscription<std_msgs::msg::String>(
        "/decision_to_escape",
        5,
        [&receive_time](std_msgs::msg::String::ConstSharedPtr msg) { receive_escape_topic_data(msg, receive_time); });
    
    auto cmd_vel_pub = nh->create_publisher<igv_msgs::msg::ControlInformation>("/cmd_vel",5);
    auto control_status_pub = nh->create_publisher<igv_msgs::msg::ControlStatus>("/control_status",5);

    control_result.speed = 2.0;
    control_result.angle = 0;
    control_result.brk_press = 0;
    control_result.torque = 0;
    control_result.torque_rpm_mode = 1;
    control_status.path_check = 0;
    control_status.trapped = 0;
    control_status.location_warp = 0;
    control_status.solution = 0;
    control_status.deviation = 0;
    control_status.prediction_check = 0; 

    // 设置循环频率
    rclcpp::WallRate loop_rate(50); // 10 Hz

    // 主循环，节点运行时会一直执行此循环
    while (rclcpp::ok())  // 当 ROS 系统正常工作时（例如没有收到关闭信号）
    {
       
        // 发布消息
        if(is_stop)
        {
            clear(current_odom.twist.twist.linear.x, receive_time);
            RCLCPP_INFO(rclcpp::get_logger("control"), "control speed: %f,brk_press: %f", control_result.speed, control_result.brk_press);
        }
        control_status_pub->publish(control_status);
        cmd_vel_pub->publish(control_result);
        
        // 处理所有的回调函数（例如订阅者的回调）
        rclcpp::spin_some(nh);

        // 按照循环频率睡眠
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
