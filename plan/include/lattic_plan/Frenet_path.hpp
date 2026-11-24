#ifndef FRENET_INCLUDED
#define FRENET_INCLUDED
#include "Eigen/Dense"
#include "common/sample_curves.h"
#include <vector>
#include <tuple>
#include <chrono>
#include <cmath>
#include <queue>
#include <memory>
#include "point.h"
#include "common/OBB.h"
#include "common/frenet_change.h"
#include <rclcpp/rclcpp.hpp>
using namespace Eigen;

enum SampleType{
    Cruise = 0,
    Stop,
    Follow,
    Overtake
};

class quintic_polynomial{
public:
    double xs;
    double vxs;
    double axs;
    double xe;
    double vxe;
    double axe;
    double a0;
    double a1;
    double a2;
    double a3;
    double a4;
    double a5;
public:
    quintic_polynomial();
    ~quintic_polynomial();
    quintic_polynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double T);
    quintic_polynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double start_T, double end_T);   
    double calc_point(double);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);
    void set_param(double b1, double b2, double b3, double b4, double b5, double b6);
};

class quartic_polynomial{
public:
    // calc coefficient of quintic polynomial
    double xs;
    double vxs;
    double axs;
    double vxe;
    double axe;

    double a0;
    double a1;
    double a2;
    double a3;
    double a4;
public:
    quartic_polynomial();
    ~quartic_polynomial();
    quartic_polynomial(double xs, double vxs, double axs, double vxe, double axe, double T);
    double calc_point(double);
    double calc_first_derivative(double t);
    double calc_second_derivative(double t);
    double calc_third_derivative(double t);
    void set_param(double b1, double b2, double b3, double b4, double b5);

};

class Frenet_path{
public:
    Frenet_path();
    ~Frenet_path();
    Frenet_path(const Frenet_path&);

    VectorXd t;
    VectorXd d;
    VectorXd d_d;
    VectorXd d_dd;
    VectorXd d_ddd;
    VectorXd s;
    VectorXd s_d;
    VectorXd s_dd;
    VectorXd s_ddd;
    double cd;
    double cv;
    double cf;

    VectorXd x;
    VectorXd y;
    VectorXd yaw;
    VectorXd ds;
    VectorXd c;
    VectorXd v;
    VectorXd a;    
    
    SampleType sample_type;

    quintic_polynomial lat_line, lon_line_5;
    quartic_polynomial lon_line_4;
     
    bool valid;
    
};

struct CompareCost {
    bool operator()(const Frenet_path& a, const Frenet_path& b) {
        return a.cf > b.cf;  // 小的 cost 优先
    }
};
class Frenet_plan{
    private:

    int sample_type;

    double MAX_SPEED; // maximum speed [m/s]
    double MAX_SPEED_LAT; //maximum angluar 
    double MAX_ACCEL;  // maximum acceleration [m/ss]
    double MAX_CURVATURE;  // maximum curvature [1/m]
    double MAX_ROAD_WIDTH;  // maximum road width [m]
    double D_ROAD_W;  // road width sampling length [m]
    double DT;  // time tick [s]
    double MAXT;  // max prediction time [m]
    double MINT;  // min prediction time [m]
    double DTIME;
    double TARGET_SPEED;  // target speed [m/s]
    double D_T_S;  // target speed sampling length [m/s]
    double SMALL_SPEED;
    double DS;  // distance tick [s]
    double MAXS;  // max prediction time [m]
    double MINS;  // min prediction time [m]
    // cost weights
    double KJ;
    double KT;
    double KJD;
    double KD;
    double KLAT;
    double KLON;
    double KCAC;
    double KEFF;

    double dt;//控制周期
    
public:
    Frenet_plan(LatticConfig l_c, double con_cyc);

    ~Frenet_plan(){};    

    LocalTrajectory smooth_path;

    Frenet_path trajectory_interp(Frenet_path &path);

    std::priority_queue<Frenet_path, std::vector<Frenet_path>, CompareCost> path_sample(Position start_point);

    std::priority_queue<Frenet_path, std::vector<Frenet_path>, CompareCost> calc_frenet_paths(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0);

    std::priority_queue<Frenet_path, std::vector<Frenet_path>, CompareCost> generate_frenet_paths(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0);

    std::priority_queue<Frenet_path, std::vector<Frenet_path>, CompareCost> generate_frenet_paths_2(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0);

    bool calc_global_paths(Frenet_path &, Spline2D &);
    
    bool check_collision(Frenet_path& fp, ObstacleGroup obstacle_group);

    bool check_paths(Frenet_path &fp, LocalTrajectory left_bound, 
    LocalTrajectory right_bound, ObstacleGroup obstacle_group);

    LocalTrajectory frenet_optimal_planning(Spline2D &csp, Position start_point, LocalTrajectory left_bound, LocalTrajectory right_bound, ObstacleGroup obstacle_group);

    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, Spline2D>
    generate_target_course(std::vector<double>&, std::vector<double>&);

    /// calculate from x,y,theta to frenet s,d
    int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
    
    int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
    std::pair<double, double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

    LocalTrajectory run(FrenetPathPoint s_p,LocalTrajectory ref_line,LocalTrajectory left_bound, 
    LocalTrajectory right_bound, ObstacleGroup obstacle_group);

    Frenet_path get_optimal_path(std::priority_queue<Frenet_path, std::vector<Frenet_path>, CompareCost> fplist, Spline2D csp,
    LocalTrajectory left_bound, LocalTrajectory right_bound, ObstacleGroup obstacle_group);

    std::shared_ptr<OBBType> obb_check = std::make_shared<OBBType>();

    bool check_over_bound(Frenet_path &fp, LocalTrajectory left_bound, LocalTrajectory right_bound);
};
#endif
