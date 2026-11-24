#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "datapool.h"
#include "lattic_plan/Frenet_path.hpp"
#include "common/path_smoother.h"
#include <chrono>
#include <ctime>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ilqr/cilqr_solver.hpp"
#include "ilqr/ilqr_config.h"
class Plan{

private:
    rclcpp::Node::SharedPtr nh;
    rclcpp::TimerBase::SharedPtr run_timer;
    rclcpp::Publisher<igv_msgs::msg::LocalTrajectory>::SharedPtr trajectory_path_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_point_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_ptr, smooth_path_ptr;

    void timerCallback();

    int last_ref_line_match_point_index, last_left_bound_match_point_index, last_right_bound_match_point_index;
    
    LocalTrajectory last_local_trajectory, stitch_trajectory, plan_result, toll_trajectory, smooth_path;
    Trajectory trajectory_path;
    int point_number = 150;
    Position start_plan_point;
    std::shared_ptr<PathSmoother> path_smoother_ptr = std::make_shared<PathSmoother>(150);
    std::shared_ptr<Frenet_plan> lattic_plan_ptr = std::make_shared<Frenet_plan>(DataPool::getInstance()->get_lattic_config(),DataPool::getInstance()->control_cycle);
    std::shared_ptr<CILQRSolver> cilqr_ptr = std::make_shared<CILQRSolver>(DataPool::getInstance()->get_ilqr_config());
public:
    Plan(rclcpp::Node::SharedPtr nh);

    ~Plan(){};

    void run();    

    bool is_arrive(FrenetPathPoint m_current_position, LocalTrajectory m_ref_line);

    void LocalTrajectoryToTestPath(LocalTrajectory trajectory, Trajectory &trajectory_path);

    void pub_path(const LocalTrajectory path, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub);
    
    void CalcYawAndKappa(LocalTrajectory &local_trajectory);

    void CalcOnlyKappa(LocalTrajectory &local_trajectory, const int start_index);

    bool ControlWeak(Position desire_position, Position current_position);

    int FindPointByTime(LocalTrajectory local_trajectory, double dt);

    LocalTrajectory PathTransform(FrenetPathPoint ego_state,Eigen::MatrixX4d path);
    
    Position GetStartPlanPoint(Position current_position,LocalTrajectory last_local_trajectory,bool IsNewTask,double dt);

    std::vector<FrenetPathPoint> get_line_part(std::vector<FrenetPathPoint> path, FrenetPathPoint m_current_position, bool &IsRecNewLine, int &last_match_point_index);
   
    LocalTrajectory get_line_part(LocalTrajectory path, FrenetPathPoint m_current_position, bool &IsRecNewLine, int &last_match_point_index, int waypoint_number);

};
