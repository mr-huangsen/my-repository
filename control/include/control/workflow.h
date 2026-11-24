#pragma once
#include "datapool.h"
#include "igv_msgs/msg/control_information.hpp"
#include "igv_msgs/msg/relay_control.hpp"
#include "common/differential_tracker.h"
#include <chrono>
#include <cmath>
#include <numeric>
#include <execution> 
#include <iterator>
#include <boost/iterator/zip_iterator.hpp>
#include <boost/tuple/tuple.hpp>
#include "control/latcontrol.h"
#include "control/loncontrol.h"
#include <std_msgs/msg/string.hpp>

struct DebugInf
{
    nav_msgs::msg::Path refer_trajectory;
    nav_msgs::msg::Path real_trajectory;
    void clear();
    int max_size = 3000;
};

class Workflow{
    public:
    Workflow(rclcpp::Node::SharedPtr nh);
    ~Workflow();   

    void run();
    igv_msgs::msg::ControlInformation CalcAEBCmd();
    double SpeedToZero();
    void PointPrediction(double& x, double& y, double& phi, double v, double dot_phi);
    void FindClosestPointByDistance(int &min_idx, const LocalTrajectory m_local_trajectory, Position m_current_position);
    void FindClosestPointByTime(int &min_idx, const LocalTrajectory m_local_trajectory);    
    double LookaheadDistance(Position current_point,double min_idx,LocalTrajectory local_plan,LateralParam latp);
    int LookaheadPointIdx(int min_idx,const LocalTrajectory local_plan,double lookahead_dist,Position current_point);
    bool JudgeIsArrive(int min_idx,Position m_current_position,const LocalTrajectory m_local_trajectory);
    double brk_press_check(double speed_cmd);
    bool JudgeIsParking();
    bool IsSlope();
    double CalcBrkPress(double speed_cmd, double speed_th);
    void JudgeStartBrakeSpeed();
    
    std::shared_ptr<LatControl> m_control_lqr = std::make_shared<LatControl>(1,0);
    std::shared_ptr<LatControl> m_control_pp = std::make_shared<LatControl>(1,2);
    std::shared_ptr<LatControl> m_control_mpc = std::make_shared<LatControl>(1,1);
    std::shared_ptr<LonControl> m_lon_control_ptr = std::make_shared<LonControl>();
    void timerCallback();
    void timerCallback_pub();
    void clear();
    bool JudgeIsTrapped(Position m_current_position);
    igv_msgs::msg::ControlInformation control_result;
    int trapped_cnt;//被困判定计数
    bool flag_brk_press;
    rclcpp::Time last_change_relay_cmd;
    DebugInf debug_inf;
    std::string last_relay_cmd;
    rclcpp::Time relay_cmd_change_timestamp;
    rclcpp::Time turnlight_cmd_change_timestamp;
    int8_t last_turn_light;
    rclcpp::Time park_start_time;
    double start_brake_speed;
    private:
    rclcpp::Node::SharedPtr nh;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr timer_2;
    rclcpp::Publisher<igv_msgs::msg::ControlInformation>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<igv_msgs::msg::ControlStatus>::SharedPtr control_status_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr refer_trajectory_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr real_trajectory_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr favoritism_path_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr relay_control_pub;
    
};
