#pragma once
#include <Eigen/Core>

class CIlqrConfig{
public:
    double dt;
    int N;
    int nx;
    int nu;
    double w_pos;
    double w_vel;
    double w_yaw;
    double w_acc;
    double w_stl;
    bool use_last_solution;

    std::string solve_type_str;

    double obstacle_exp_q1;
    double obstacle_exp_q2;
    double state_exp_q1;
    double state_exp_q2;

    double alm_rho_init;
    double alm_gamma;
    double max_rho;
    double max_mu;

    int max_iter;
    double init_lamb;
    double lamb_decay;
    double lamb_amplify;
    double max_lamb;
    double convergence_threshold;
    double accept_step_threshold;    

    double wheelbase;
    double width;
    double length;
    double velo_max;
    double velo_min;
    double yaw_lim;
    double acc_max;
    double acc_min;
    double stl_lim;
    double d_safe;
    std::string reference_point;   
    double target_velocity;

};