#pragma once
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <cmath>
#include <memory>
#include <mutex>
#include <chrono>
#include <ctime>
#include "igv_msgs/msg/control_information.hpp"
#include "igv_msgs/srv/local_path.hpp"
#include "igv_msgs/msg/location.hpp"
#include "control_param.h"
#include "log/log.h"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <igv_msgs/msg/control_status.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <std_msgs/msg/int8.hpp>
#include "igv_msgs/msg/iccp_status.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
typedef igv_msgs::msg::Location Location;
typedef igv_msgs::msg::ControlStatus ControlStatus;
typedef igv_msgs::msg::ICCPStatus ChassisStatus;

class DataPool{
    private:
    DataPool(rclcpp::Node::SharedPtr nh);
    static DataPool* instance;
    static std::mutex mtx; 
    
    // 删除拷贝构造函数和赋值运算符，防止拷贝
    DataPool(const DataPool&) = delete;
    DataPool& operator=(const DataPool&) = delete;

    rclcpp::Node::SharedPtr nh;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr chassis_link_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr escape_sub;
    rclcpp::Subscription<igv_msgs::msg::Location>::SharedPtr location_sub;
    rclcpp::Subscription<igv_msgs::msg::ICCPStatus>::SharedPtr chassis_status_sub;
    rclcpp::Service<igv_msgs::srv::LocalPath>::SharedPtr local_path_srv;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dispatch_sub;//调度传送的停车指令
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr turnlight_sub;
    rclcpp::Subscription<igv_msgs::msg::ControlInformation>::SharedPtr calibration_cmd_sub;
    igv_msgs::msg::ControlInformation calibration_cmd;
    rclcpp::Time last_chassis_status_receive_time;
    std::mutex mutexcalibrationcmd;
public:
    static void Initialize(const rclcpp::Node::SharedPtr nh) {
        if (instance == nullptr) {
            std::lock_guard<std::mutex> lock(mtx); 
            instance = new DataPool(nh);                        
        }        
    }
    // 获取单例实例的静态方法
    static DataPool* getInstance() {
        // if (instance == nullptr) {
        //     instance = new DataPool(nh);
        // }
        // static DataPool instance(nh);  // 静态局部变量，保证在第一次调用时创建实例
        return instance;
    }
    
    public:
        // DataPool();
        ~DataPool();      
        bool flag_new_refer_line = false;
        int stop_type;
        int vaild_path_index;
        double k_stop_speed;//停车时，缓慢降速的增益
        double acc_stop;
        double k_stop_speed_slow;//缓停        
        double acc_stop_slow;
        
        int control_alg_type;
        int8_t turnligth_command_by_decision = 0;
        bool receive_decision_turnlight_cmd = false, is_chassis_link_fail = false;
        bool is_calibration_mode = false;
        LocalTrajectory local_trajectory;
        std::string escape_topic_data;

        Location local_point;
        int path_type = 0, is_stop_turn_wheel = 0;
        Position currect_position;
        nav_msgs::msg::Odometry odom_data;
        LateralParam lat_param;
        LongitudinalParam lon_param;
        VehicleParam vehicle_param;
        ControlStatus control_status;
        ChassisStatus chassis_status;
        bool flag_refer_line_follow_type;
        rclcpp::Time location_receive_time, turnlight_command_receive_time, aeb_receive_time, path_receive_time, chassis_link_receive_time;
        rclcpp::Time calibration_cmd_receive_time;
        double dt; //控制周期 50Hz
        double prediction_time; //预测时间 
        std::mutex mutexiccpdata;
        std::mutex mutexodomdata;
        std::mutex mutexlocalpath;
        std::mutex mutexlocationpoint;        
        std::mutex mutexgetlatparam;
        std::mutex mutexgetlonparam;
        std::mutex mutexgetvehicleparam;  
        std::mutex mutexgetcontrolstatus;    
        std::mutex mutexturnlightcommand;
        std::mutex mutexescape;
    
    void receive_turnlight_command(const std_msgs::msg::Int8::ConstSharedPtr& msg);
    void receive_location_data(const igv_msgs::msg::Location::ConstSharedPtr& msg);
    void receive_dispatch_data(const std_msgs::msg::Bool::ConstSharedPtr& msg);
    void receive_chassis_link_data(const std_msgs::msg::Bool::ConstSharedPtr& msg);
    void receive_escape_topic_data(const std_msgs::msg::String::ConstSharedPtr& msg);
    bool handle_local_path(igv_msgs::srv::LocalPath::Request &req,
        igv_msgs::srv::LocalPath::Response &res);
    void receive_chassis_status(const igv_msgs::msg::ICCPStatus::ConstSharedPtr& msg);
    void receive_calibration_cmd(const igv_msgs::msg::ControlInformation::ConstSharedPtr& msg);
    void clear_path();
    void ResetChassisLinkFlag();
    bool check_path(igv_msgs::srv::LocalPath::Request temp_local_path);
    bool check_location(std::shared_ptr<const igv_msgs::msg::Location> temp_location_point);
    bool check_location_status(std::shared_ptr<const igv_msgs::msg::Location> temp_location_point);
    double norm_angle(double angle);
    LateralParam get_lat_param();
    LongitudinalParam get_lon_param();
    VehicleParam get_vehicle_param();
    Position get_currect_position();
    LocalTrajectory get_local_trajectory();
    nav_msgs::msg::Odometry get_odom_data();
    ControlStatus* get_control_status();
    igv_msgs::msg::ControlInformation get_calibration_cmd();
};
