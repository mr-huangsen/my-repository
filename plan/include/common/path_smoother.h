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
#include "log/log.h"
#include "common/qp_solve.h"
#include "point.h"

class PathSmoother{
    private:
    Eigen::MatrixXd A_s, A_d, A_r;    

    Eigen::MatrixXd temp_A_s, temp_A_d, temp_A_r;    

    Eigen::MatrixXd AA_s, AA_d, AA_r;    

    Eigen::MatrixXd H;
    Eigen::VectorXd g;

    Eigen::MatrixXd A_con_1, A_con_k, A_con;
    Eigen::VectorXd l_inf_1, u_sup_1, l_inf_k, u_sup_k, l_inf, u_sup;
    Eigen::MatrixXd A_f;
    Eigen::VectorXd temp_xy;
    
    std::shared_ptr<QpSolve> qp_solve_path_smoother = std::make_shared<QpSolve>();

    public:
    PathSmoother(double w_s,double w_d,double w_r,double buffer_x,double buffer_y,double max_kappa, int pn);
    PathSmoother(int pn);
    ~PathSmoother();
    double w_s,w_d,w_r;         //平滑权重、距离权重、形变权重   
    double buffer_x, buffer_y;  //路径点偏移量  
    double stack;             //计算松弛因子
    double max_kappa;        //最大曲率
    double point_number;        //路点数量
    LocalTrajectory result;//平滑之后的路径
    bool run(LocalTrajectory local_trajectory);
};
