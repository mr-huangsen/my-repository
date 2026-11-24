#pragma once
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>
#include <memory>
#include <chrono>
#include <ctime>
#include "igv_msgs/msg/location.hpp"
#include "log/log.h"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "point.h"
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include "igv_msgs/msg/local_trajectory.hpp"
#include "igv_msgs/msg/position.hpp"
#include "ilqr/ilqr_config.h"
typedef igv_msgs::msg::Location Location;
typedef igv_msgs::msg::LocalTrajectory Trajectory;

struct PredictionStatus
{
    int8_t location_warp;
};


class DataPool{
    private:
    DataPool(rclcpp::Node::SharedPtr nh);
    static DataPool* instance;
    static std::mutex mtx; 
    // {
    // }
    
    // 删除拷贝构造函数和赋值运算符，防止拷贝
    DataPool(const DataPool&) = delete;
    DataPool& operator=(const DataPool&) = delete;

    rclcpp::Node::SharedPtr nh;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<igv_msgs::msg::Location>::SharedPtr location_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr ref_line_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr right_bound_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr left_bound_sub;
    nav_msgs::msg::Odometry odom_data;

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
        ~DataPool(){};   

        int sample_point_number;
        double control_cycle;
        CIlqrConfig cilqr_config;
        LatticConfig lattic_config;
        Location local_point;
        FrenetPathPoint currect_position;
        nav_msgs::msg::OccupancyGrid occupancy_grid;
        PredictionStatus prediction_status;
        ObstacleGroup obstacle_group;
        LocalTrajectory ref_line,left_bound, right_bound;
        geometry_msgs::msg::PoseStamped start_point, end_point;
        std::vector<geometry_msgs::msg::Point> corner_points;
        int plan_type;
        double chassis_bias_angle;
        bool flag_is_new_task;        
        rclcpp::Time location_receive_time;
        double distance_forward;
        double distance_back;
        double run_dt; //运行周期 50Hz
        double L;        
        geometry_msgs::msg::TransformStamped transformStamped;

        std::mutex mutexobstacle;
        std::mutex mutexlocationpoint;
        std::mutex mutexoccupancygrid;
        std::mutex mutexkalmanconfig;  
        std::mutex mutexpfconfig; 
        std::mutex mutexrefline;
        std::mutex mutexleftbound;
        std::mutex mutexrightbound;
        std::mutex mutexodomdata;
        std::mutex mutexlatticconfig;
        std::mutex mutexilqrconfig;
    void receive_odom_data(const nav_msgs::msg::Odometry::SharedPtr msg);
    void receive_left_bound(const nav_msgs::msg::Path::SharedPtr msg);
    void receive_right_bound(const nav_msgs::msg::Path::SharedPtr msg);
    void receive_ref_line(const nav_msgs::msg::Path::SharedPtr msg);
    void receive_location_data(const igv_msgs::msg::Location::SharedPtr msg);
    void receive_occupancy_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    bool check_location(const igv_msgs::msg::Location::SharedPtr temp_location_point);
    bool check_location_status(const igv_msgs::msg::Location::SharedPtr temp_location_point);
    

    FrenetPathPoint get_currect_position(); 
    ObstacleGroup get_obstacle_group();
    nav_msgs::msg::OccupancyGrid get_occupancy_grid();
    LocalTrajectory get_ref_line();
    std::vector<FrenetPathPoint> get_left_bound();
    std::vector<FrenetPathPoint> get_right_bound();
    LatticConfig get_lattic_config();
    CIlqrConfig get_ilqr_config();
};
