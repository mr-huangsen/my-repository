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

struct PidController{
    double kp;
    double ki;
    double kd;
    double last_error;
    double integral;
    void init(double pp,double ii,double dd);
    void paramupdate(double err,double dt);
};

struct LatError
{
    double error_d = 0.0;
    double error_dot_d = 0.0;
    double error_phi = 0.0;
    double error_dot_phi = 0.0;
    double error_d_sum = 0.0;
    double error_phi_sum = 0.0;
    double d_sum_limit = 0.6;
    double phi_sum_limit = 0.6;
    double last_error_d = 0.0;
    double last_error_phi = 0.0;
};

struct LonError
{
    double error_s = 0.0;
    double error_dot_s = 0.0;
    double error_dot_dot_s = 0.0;
    double error_s_sum = 0.0;
    double error_dot_s_sum = 0.0;
    double error_s_last = 0.0;
    double error_dot_s_last = 0.0;
    double error_s_smooth = 0.0;
    double error_dot_s_smooth = 0.0;
    double error_dot_dot_s_smooth = 0.0;
    double error_s_sum_smooth = 0.0;
    double error_s_last_smooth = 0.0;
    double s_sum_limit = 0.6;
    double dot_s_sum_limit = 7.5;
};

struct Error
{
    LatError laterror;
    LonError lonerror;
    double kappar = 0.0;
};
enum Model{
    KinematicsModel = 1,
    DynamicalModel = 2
};
enum ControlType{
    LQR = 0,
    MPC = 1,
    PP = 2
};
class LatControl{
    private:
    LateralParam m_lat_param;
    VehicleParam m_vehicle_param;
    Error  car_error_;    
    double dt;    
    double k03_;      

    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd C_;
    Eigen::MatrixXd D_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;    
    
    Model model_select_;
    ControlType control_type_;
    
    //MPC param
    Eigen::MatrixXd H_;
    Eigen::MatrixXd g_;
    Eigen::MatrixXd X_;
    Eigen::MatrixXd Su_b_;
    Eigen::MatrixXd A_con_;
    Eigen::MatrixXd l_inf_;
    Eigen::MatrixXd u_sup_;    
    double norm_angle(double angle);
    void error_calculate(Position current_car_status, Position match_car_status);
    void status_update(Position current_car_status, Position match_car_status);
    double lat_control_calculate(Position current_car_status, Position match_car_status, Position look_ahead_car_status);
    double LimitAngle(double angle,LateralParam latp);
    double LQR(Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::VectorXd x);
    double PP(Position current_car_status, Position match_car_status);    
    std::shared_ptr<QpSolve> qp_solve_mpc = std::make_shared<QpSolve>();
    double MPC();
    void make_prediction_matrix(Eigen::VectorXd x,Eigen::VectorXd u,Eigen::MatrixXd A,Eigen::MatrixXd B,Eigen::MatrixXd C,Eigen::MatrixXd D,Eigen::MatrixXd Q,Eigen::MatrixXd R);
    void make_constraint_conditions(Eigen::VectorXd x,Eigen::VectorXd u);    
    Eigen::MatrixXd multi_matrix(Eigen::MatrixXd Matrix_1, Eigen::MatrixXd Matrix_2, const int n);
    Eigen::MatrixXd calc_power(Eigen::MatrixXd Matrix, const int m, const int n);
    

    public:
    LatControl(const int &model_type, const int &control_type);
    ~LatControl() = default;
    Eigen::VectorXd u_lat_;
    void clear_error();
    double lookahead_dist;
    double lat_calculate(const Position &current_car_status, const Position &match_car_status, const Position &look_ahead_car_status);
};
