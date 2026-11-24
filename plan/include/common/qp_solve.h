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
class QpSolve{
    public:
    QpSolve();
    ~QpSolve();
    Eigen::VectorXd osqp_solution;
    bool run(Eigen::MatrixXd P,Eigen::VectorXd q,Eigen::MatrixXd A,Eigen::VectorXd l_inf,Eigen::VectorXd u_sup);
    void createCSC(const Eigen::MatrixXd& P, std::vector<c_int>& rowind, std::vector<c_int>& colptr, std::vector<c_float>& values); 
    void print_csc_matrix(csc* matrix); 


};
