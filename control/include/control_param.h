#pragma once
#include <cmath>
#include <cstdint>
#include <vector>
struct VehicleParam
{
    double Cr;      //后轮侧偏刚度
    double Cf;      //前轮侧偏刚度
    double m;       //车辆质量
    double I;       //绕z轴的转动惯量
    double lr;      //后轴长
    double lf;      //前轴长
    double L;      //轴长
    VehicleParam():Cr(0.0),Cf(0.0),m(0.0),I(0.0),lr(0.0),lf(0.0),L(0.0){};
};
struct LongitudinalParam
{
    int torque_rpm_mode;        //控制模式 0-扭矩 1-速度
    double kp_lon;              //速度模式下，位置环kp
    double ki_lon;              //速度模式下，位置环ki
    double kd_lon;              //速度模式下，位置环kd
    double k_pressure;          //制动时，制动压力与目标速度之间的比例系数
    double kp_press;            //制动时,位置环kp
    double k_torque;            //扭矩模式下，目标扭矩与目标速度之间的比例系数
    double kp_t_p;              //扭矩模式下，位置环kp
    double ki_t_p;              //扭矩模式下，位置环ki
    double kd_t_p;              //扭矩模式下，位置环kd
    double kp_t_s;              //扭矩模式下，速度环kp
    double ki_t_s;              //扭矩模式下，速度环ki
    double kd_t_s;              //扭矩模式下，速度环kd
    double max_v;               //最大线速度
    double min_v;               //最小线速度
    double max_v_delta;         //最大加速度
    double max_v_back;          //最大的后退速度
    bool is_use_adaptive_pid;    //是否使用自适应pid
    bool slope_mode;             //是否启用斜坡模式
    double k_slope_compensate;  //坡道起步补偿倍率
    LongitudinalParam():torque_rpm_mode(1),kp_lon(0.0),ki_lon(0.0),kd_lon(0.0),k_pressure(0.0),kp_press(0.0),k_torque(0.0),kp_t_p(0.0),ki_t_p(0.0),kd_t_p(0.0),kp_t_s(0.0),ki_t_s(0.0),kd_t_s(0.0),
                        max_v(0.0),min_v(0.0),max_v_delta(0.0),max_v_back(0.0),is_use_adaptive_pid(false),slope_mode(false),k_slope_compensate(0.0){};
};
struct LateralParam
{
    double manufacturing_accuracy_compensation;             //制造精度补偿角
    double model_compensate;                                //转弯模型补偿倍率
    std::vector<double> Q_matrix_kinematics;                           //lqr算法运动学模型权重矩阵Q的迹
    std::vector<double> R_matrix_kinematics_lat;                       //lqr算法运动学模型权重矩阵R的迹
    std::vector<double> Q_matrix_dynamical;                            //lqr算法动力学模型权重矩阵Q的迹
    std::vector<double> R_matrix_dynamical_lat;                        //lqr算法动力学模型权重矩阵R的迹
    std::vector<double> Q_matrix_kinematics_mpc;                       //mpc算法运动学模型权重矩阵Q的迹
    std::vector<double> R_matrix_kinematics_lat_mpc;                   //mpc算法运动学模型权重矩阵R的迹
    std::vector<double> Q_matrix_dynamical_mpc;                        //mpc算法动力学模型权重矩阵Q的迹
    std::vector<double> R_matrix_dynamical_lat_mpc;                    //mpc算法动力学模型权重矩阵R的迹
    double ki_lat;                                          //横向控制ki
    double ki_phi;                                          //角度积分增益ki
    double kd_lat;                                          //横向控制kd
    double kd_phi;                                          //航向角kd
    int Hp;                                                 //mpc预测视界
    int Hu;                                                 //mpc控制视界
    int max_iter;                                           //最大的求解迭代次数
    double eps_iter;                                        //迭代精度
    double min_lookahead_dist;                              //最小的前视距离
    double max_lookahead_dist;                              //最大的前视距离
    double max_w;                                           //最大的车轮转角
    double min_w;                                           //最小的车轮转角
    double max_w_delta;                                     //最大的车轮转角速度
    double max_kappa;                                       //可通过的最大道路曲率
    double max_diff_kappa;                                  //两路点的最大曲率变化
    double max_diff_phi;                                    //两路点的最大航向角变化
    double max_dist_between_two_points;                     //两点间最大距离
    LateralParam():manufacturing_accuracy_compensation(0.0),model_compensate(0.0),Q_matrix_kinematics(),R_matrix_kinematics_lat(),Q_matrix_dynamical(),R_matrix_dynamical_lat(),
                   Q_matrix_kinematics_mpc(),R_matrix_kinematics_lat_mpc(),Q_matrix_dynamical_mpc(),R_matrix_dynamical_lat_mpc(),ki_lat(0.0),ki_phi(0.0),kd_lat(0.0),kd_phi(0.0),
                   Hp(0),Hu(0),max_iter(0),eps_iter(0.0),min_lookahead_dist(0.0),max_lookahead_dist(0.0),max_w(0.0),min_w(0.0),max_w_delta(0.0),max_kappa(0.0),max_diff_kappa(0.0),
                   max_diff_phi(0.0),max_dist_between_two_points(0.0){};
};

struct Position
{
    std::uint64_t timestamp = 0; // 微秒时间戳
    double x = 0.0;
    double y = 0.0;
    double phi = 0.0;
    double dot_phi = 0.0;
    double v = 0.0;
    double v_last = 0.0;
    double a = 0.0;
    double kappa = 0.0;
    double steer = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    int status = 0;
};

struct LocalTrajectory
{
    std::vector<Position> waypoint;
    std::int32_t flag_stop = 0;
};

