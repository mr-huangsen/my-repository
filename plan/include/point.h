#pragma once
#include <vector>
#include <cstdint>
class VehicleParam{
    public:
    double l; //长
    double w; //宽
    double h; //高
};

class CartesianPathPoint: public VehicleParam{

    public:
        double x;
        double y;
        double z;
        double yaw; 
        double cur;    
        uint64_t time;
        CartesianPathPoint()
        {
            x = 0;
            y = 0;
            z = 0;
            yaw = 0; 
            cur = 0; 
        } 
    };
    
    class CartesianCarInfo : public CartesianPathPoint{
    
    public:
        double vx; 
        double vy; 
        double vz;
        double v;
        double ax;
        double ay;
        double az;
        double a; 
        double dot_yaw;
        CartesianCarInfo()
        {
            vx = 0; 
            vy = 0; 
            vz = 0;
            v = 0;
            ax = 0;
            ay = 0;
            a = 0; 
            dot_yaw = 0;

        }
    };
    
    class FrenetPathPoint : public CartesianCarInfo{
    public:
        double s;
        double l;
        double s_d; 
        double l_d; 
        double s_dd;
        double l_dd;
        double s_ddd;
        double l_ddd;
    
        double l_ds;
        double l_dds;
        double l_ddds;
           
        FrenetPathPoint()
        {
            s = 0;
            l = 0;
            s_d = 0; 
            l_d = 0; 
            s_dd = 0;
            l_dd = 0;
            s_ddd = 0;
            l_ddd = 0;
        
            l_ds = 0;
            l_dds = 0;
            l_ddds = 0;
        }
    };
enum ObsStatus{
    untreated = 0,      //未处理的障碍物
    handled_right,      //已处理的障碍物,右转
    handled_left,       //已处理的障碍物,左转
    unknown             //未知

};

    class Obstacle{
        public:
                ObsStatus obs_status;   //  障碍物处理状态
                FrenetPathPoint center; // 中心点
                FrenetPathPoint halfExtents; // 半边长   x:长 y：宽
                float rotation; // 旋转角度，弧度                      
                std::vector<FrenetPathPoint> collision_box;
        };

// class FrenetPath{

// public:    
//     std::vector<FrenetPathPoint> waypoint;
//     int size = 0; 
//     double max_speed;
//     double max_acc;
//     double max_curvature;
// };

typedef std::vector<FrenetPathPoint> LocalTrajectory;
typedef FrenetPathPoint Position;
typedef std::vector<Obstacle> ObstacleGroup;

struct LatticConfig
{
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
    int sample_type;
};