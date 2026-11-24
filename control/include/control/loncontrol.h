#pragma once
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <unsupported/Eigen/KroneckerProduct>
#include <limits>
#include "osqp/osqp.h"
#include <ctype.h>
#include "log/log.h"
#include "datapool.h"
#include "common/differential_tracker.h"
#include "common/qp_solve.h"
#include "control/latcontrol.h"

class LonControl{
    private:
    LongitudinalParam m_lon_param;
    PidController adaptive_pid_;//自适应PID控制器    
    Error  car_error_;    
    int feedforward_mode = 0; // 扭矩前馈模式：0 线性拟合，1 神经网络
    double dt;       
    double norm_angle(double angle);
    void error_calculate(Position current_car_status, Position match_car_status);
    igv_msgs::msg::ControlInformation lon_contrl_calculate(Position current_car_status, Position match_car_status);
    double PID(double error, double kp, double ki, double kd, double sum_error, double &last_error);
    double SpeedCheck(double speed,LongitudinalParam lonp);
    double TorqueCheck(double torque);
    double FitTorqueTable(double speed, double acc, double pitch);
    std::shared_ptr<DifferentialTracker> differential_tracker_es = std::make_shared<DifferentialTracker>(0.9);
    std::shared_ptr<DifferentialTracker> differential_tracker_dot_es = std::make_shared<DifferentialTracker>(0.9);
    std::shared_ptr<DifferentialTracker> differential_tracker_dot_dot_es = std::make_shared<DifferentialTracker>(0.9);
    std::shared_ptr<DifferentialTracker> differential_tracker_refer_v = std::make_shared<DifferentialTracker>(0.9);
      
    public:
    LonControl();
    ~LonControl() = default;
    void clear_error();
    igv_msgs::msg::ControlInformation lon_calculate(const Position &current_car_status, const Position &match_car_status);
};
