#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "igv_msgs/msg/location.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include "log/log.h"
#include "igv_msgs/msg/local_trajectory.hpp"
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
igv_msgs::msg::LocalTrajectory local_trajectory;

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}

void receive_favoritism_path(const igv_msgs::msg::LocalTrajectory::SharedPtr msg)
{
    local_trajectory = *msg;
    LOG_INFO("NEW PATH!!!");
}
int FindPointByTime(igv_msgs::msg::LocalTrajectory local_trajectory, double dt)
{
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());

    uint64_t now_time = duration.count();   
    for(int i = 0;i<local_trajectory.waypoint.size()-1;i++)
    {        
        if((now_time * 1.0 / 1e6 + 0.1  >= local_trajectory.waypoint[i].timestamp  * 1.0 / 1e6) && (now_time  * 1.0 / 1e6 + 0.1 < local_trajectory.waypoint[i+1].timestamp  * 1.0 / 1e6))
        {
            LOG_INFO("index: {}", i);
            return i;
        }
    }
    return 0;
}
int main(int argc, char **argv)
{
    // 初始化 ROS 系统
    rclcpp::init(argc, argv);
    
    // 创建一个节点句柄（NodeHandle 用于与 ROS 系统进行通信）
    auto nh = rclcpp::Node::make_shared("test_node");

    auto& logger = commonlog::get();
    const auto log_path = std::filesystem::path(PLAN_PACKAGE_SOURCE_DIR) / "logs";
    if (!logger.init(log_path.string(), "plan_test.log")) {
        std::cerr << "日志初始化失败: " << logger.status().last_error << '\n';
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    // 创建一个发布者（Publisher）发布到 "chatter" 话题，消息类型为 std_msgs/String
    auto odom_pub = nh->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    auto ref_line_pub = nh->create_publisher<nav_msgs::msg::Path>("/planning_component/niit_routing_reference_line", 10);
    auto left_bound_pub = nh->create_publisher<nav_msgs::msg::Path>("/planning_component/left_bound", 10);
    auto right_bound_pub = nh->create_publisher<nav_msgs::msg::Path>("/planning_component/right_bound", 10);
    auto location_pub = nh->create_publisher<igv_msgs::msg::Location>("/localization", 10);
    auto favoritism_path_sub = nh->create_subscription<igv_msgs::msg::LocalTrajectory>("/favoritism_path", 10, &receive_favoritism_path);
    nav_msgs::msg::Odometry temp_odom;
    temp_odom.twist.twist.linear.x = 0.0;
    temp_odom.twist.twist.angular.z = 0.0;   

    nav_msgs::msg::Path temp_ref_line;
    temp_ref_line.header.stamp = nh->now();
    temp_ref_line.header.frame_id = "map";
    nav_msgs::msg::Path temp_left_bound;
    temp_left_bound.header.stamp = nh->now();
    temp_left_bound.header.frame_id = "map";
    nav_msgs::msg::Path temp_right_bound;
    temp_right_bound.header.stamp = nh->now();
    temp_right_bound.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = nh->now();
    pose.header.frame_id = "map";
    //参考线
    for(double i = 0.0 ; i < 25.0; i+= 0.2)
    {
        pose.pose.position.x = i;
        pose.pose.position.y = i;
        pose.pose.orientation = createQuaternionMsgFromYaw(0.0);
        temp_ref_line.poses.push_back(pose);
    }
    for(double i = 25.0 ; i < 45; i+= 0.2)
    {
        pose.pose.position.x = i;
        pose.pose.position.y = sqrt(10*10 - pow((i-35),2)) + 25;
        temp_ref_line.poses.push_back(pose);
    }
    for(double i = 45.0 ; i < 75; i+= 0.2)
    {
        pose.pose.position.x = i;
        pose.pose.position.y = -sqrt(15*15 - pow((i-60),2)) + 25;
        temp_ref_line.poses.push_back(pose);
    }
    for(double i = 75.0 ; i < 100; i+= 0.2)
    {
        pose.pose.position.x = i;
        pose.pose.position.y = 0.5 * i - 12.5;
        temp_ref_line.poses.push_back(pose);
    }
    
//左边界线
    for(double i = -5/sqrt(2) ; i < 20.17; i+= 0.2)
    {
        pose.pose.position.x = i;
        pose.pose.position.y = i + 5 * sqrt(2);
        temp_left_bound.poses.push_back(pose);
    }
    for(double i = 20.17 ; i < 50; i+= 0.2)
    {
        pose.pose.position.x = i;
        pose.pose.position.y = sqrt(15*15 - pow((i-35),2)) + 25;
        temp_left_bound.poses.push_back(pose);
    }
    for(double i = 50 ; i < 70; i+= 0.2)
    {
        pose.pose.position.x = i;
        pose.pose.position.y = -sqrt(10*10 - pow((i-60),2)) + 25;
        temp_left_bound.poses.push_back(pose);
    }
    for(double i = 70 ; i < 75-5/sqrt(2); i+= 0.2)
    {
        pose.pose.position.x = i;
        pose.pose.position.y = (30.3033-25)/(71.4645-70) * (i-70) + 25;
        temp_left_bound.poses.push_back(pose);
    }
    for(double i = 75-5/sqrt(2); i < 100-5/sqrt(2); i+= 0.2)
    {
        pose.pose.position.x = i;
        pose.pose.position.y = 0.5 * i + 5 * sqrt(2) - 12.5;
        temp_left_bound.poses.push_back(pose);
    }
//右边界线
    for(double i = 5/sqrt(2) ; i < 25+5/sqrt(2); i+= 0.2)
    {
        pose.pose.position.x = i;
        pose.pose.position.y = i - 5 * sqrt(2);
        temp_right_bound.poses.push_back(pose);
    }
    for(double i = 25+5/sqrt(2) ; i < 30; i+= 0.2)
    {
        pose.pose.position.x = i;
        pose.pose.position.y = (25-21.4645)/(30-28.5355) * (i- 25-5/sqrt(2)) + 21.4645;
        temp_right_bound.poses.push_back(pose);
    }
    for(double i = 30 ; i < 40; i+= 0.2)
    {
        pose.pose.position.x = i;
        pose.pose.position.y = sqrt(5*5 - pow((i-35),2)) + 25;
        temp_right_bound.poses.push_back(pose);
    }
    for(double i = 40 ; i < 79.40; i+= 0.2)
    {
        pose.pose.position.x = i;
        pose.pose.position.y = -sqrt(20*20 - pow((i-60),2)) + 25;
        temp_right_bound.poses.push_back(pose);
    }
    
    for(double i = 79.40; i < 100+5/sqrt(2); i+= 0.2)
    {
        pose.pose.position.x = i;
        pose.pose.position.y = 0.5 * i - 5 * sqrt(2) - 12.5;
        temp_right_bound.poses.push_back(pose);
    }


   

    geometry_msgs::msg::PoseStamped position;
    std::vector<geometry_msgs::msg::PoseStamped> way;

     for(double i = 0.0 ; i < 25.0; i+= 0.2)
    {
        position.pose.position.x = i + 1;
        position.pose.position.y = i  - 0.5;
        position.pose.orientation = createQuaternionMsgFromYaw(30.0 / 180.0 * 3.14159);
        way.push_back(position);
    }
    // std::copy(temp_ref_line.poses.begin(),temp_ref_line.poses.begin() + (int)(temp_ref_line.poses.size()/2),std::back_inserter(way));
    int count = 0;
    bool pub_flag = true;
    igv_msgs::msg::Location test_data;
    
    // test_data.status = 1;
    // auto c_time = ros::Time::now();
    // test_data.timestamp = uint64_t(c_time.sec * 1e9 + c_time.nsec);
    // test_data.car_pose.position.x = 25.2;
    // test_data.car_pose.position.y = 23.0;   
    // test_data.car_pose.position.z = 0.0;   
    // test_data.car_pose.orientation =  tf::createQuaternionMsgFromYaw(0.0);
    // test_data.vel.x = 0.000;
    // test_data.vel.y = 0.000;
    // test_data.vel.z = 0.0000;
    // test_data.gyr.z = 0.000; 
    // 设置循环频率
    rclcpp::Rate loop_rate(10); // 10 Hz
    // 主循环，节点运行时会一直执行此循环
    while (rclcpp::ok())
    {       
        // 发布消息
        if(pub_flag)
        {
            RCLCPP_INFO(nh->get_logger(), "count: %d", count);
            if(local_trajectory.waypoint.empty())
            {
                if (way.empty()) {
                    RCLCPP_WARN(nh->get_logger(), "test path is empty, skip localization publish");
                    rclcpp::spin_some(nh);
                    loop_rate.sleep();
                    continue;
                }
                test_data.status = 1;
                auto c_time = nh->now();
                test_data.timestamp = static_cast<uint64_t>(c_time.seconds() * 1e6);
                test_data.car_pose = way[count % way.size()].pose;
                test_data.vel.x = 0.000;
                test_data.vel.y = 0.000;
                test_data.vel.z = 0.0000;
                test_data.gyr.z = 0.000; 
                temp_odom.twist.twist.linear.x = 1.0;
                temp_odom.twist.twist.angular.z = 0.0; 
                odom_pub->publish(temp_odom);
                location_pub->publish(test_data);
            }else
            {
                test_data.status = 1;
                auto c_time = nh->now();
                test_data.timestamp = static_cast<uint64_t>(c_time.seconds() * 1e6);
                double dt = 0.0;
                auto index = FindPointByTime(local_trajectory,dt);
                test_data.car_pose.position.x = local_trajectory.waypoint[index].x;   
                test_data.car_pose.position.y = local_trajectory.waypoint[index].y;
                test_data.car_pose.orientation = createQuaternionMsgFromYaw(local_trajectory.waypoint[index].phi);
                
                test_data.vel.x = 0.000;
                test_data.vel.y = 0.000;
                test_data.vel.z = 0.0000;
                test_data.gyr.z = 0.000; 
                temp_odom.twist.twist.linear.x = local_trajectory.waypoint[index].v;
                temp_odom.twist.twist.angular.z = 0.0;   
                odom_pub->publish(temp_odom);
                location_pub->publish(test_data);
            }
            
        }     
        ref_line_pub->publish(temp_ref_line);
        left_bound_pub->publish(temp_left_bound);
        right_bound_pub->publish(temp_right_bound);
        count += 1;
        // if(count > way.size())
        if(count > 100000)
        {
            pub_flag = false;
            LOG_INFO("over!");
        }           

        // 处理所有的回调函数（例如订阅者的回调）
        rclcpp::spin_some(nh);

        // 按照循环频率睡眠
        loop_rate.sleep();
    }

    logger.shutdown();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
