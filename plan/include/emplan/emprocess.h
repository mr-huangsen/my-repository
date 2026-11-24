#include "emplan/osqpSolver.h"
#include "emplan/smoothSolver.h"
#include "emplan/frenetUtils.h"
#include "emplan/EmplanConfig.h"
#include "point.h"
typedef struct costElement{
    double miniCost = 1e8;
    int preRow = -1;
    int preCol = -1;
    Position pose;
}costElement; 

enum cardesian2FrentMode {normal,firstInfo,secondInfo};

class EmPlan{
    private:    

    double NormalizeAngle(double angle);

    // get match_point or project_point of trajectory
    int getMatchPoint(Position hostPoint, int prePointIdx,const std::vector<Position> &vecTraj);
    void getProjectPoint(Position hostPoint, const Position &matchPoint, Position &projectPoint);
    void getProjectPoint(const frenet &frenet_point, const std::vector<Position> &ref_trajectory, const Eigen::VectorXd &index_S,Position &projectPoint);

    // prepare for frenet trajectory
    double calcSFromIndex2S(int matchPoint_index_local,const Position &project_point,const std::vector<Position> &frenet_trajectory,const Eigen::VectorXd &index_S);
    void index2S(int matchPoint_index_local,const Position &project_point,const std::vector<Position> &frenet_trajectory,Eigen::VectorXd &index_S);

    double getDistance(const Position &p1,const Position &p2);

    // get frenet trajectory
    // single point: cardesian(Position type) -> frenet
    void cardesian2Frenet(const Position &cardesian_point, int match_point_index, const Position &project_point, const std::vector<Position> &ref_trajectory,const Eigen::VectorXd &index_S,frenet &frenet_pos,cardesian2FrentMode mode = secondInfo);
    // point array: frenet_array -> cardesian(Position type)
    void frenet2Cardesian(const std::vector<frenet> &dense_init_path, const std::vector<Position> &ref_trajectory,const Eigen::VectorXd &index_S, std::vector<Position> &init_path_cardesian);

    // select obstacles
    void obsSelect(const Position &currentvehicle,const std::vector<Position> &obstacle_array_original, std::vector<Position> &obstacle_array_selected);

    //dynamic programming
    void calcQuinticCoeff(const frenet &start_point, const frenet &end_point, Eigen::VectorXd &coeff);
    double calcNeighborCost(const std::vector<frenet> &obstacle_array,const frenet &start_point, const frenet &end_point, const DPConfig &config);
    void getInitPlanValue(const std::vector<frenet> &obstacle_array,const frenet &start_point, const DPConfig &config, std::vector<frenet> &init_path);
    double calcObsCost(double w_cost_obs, double square_dist);
    void trajectoryInterp(std::vector<frenet> &init_path);

    //osqp programming
    void getCovexBound(const std::vector<frenet> &init_path,const std::vector<frenet> &obstacle_array,const QPConfig &config,Eigen::VectorXd &low_bound, Eigen::VectorXd &upper_bound);
    int getMatchSIndex(const std::vector<frenet> &init_path,double s);


    // time and velocity set
    void timeAndVelocity(std::vector<Position> &init_path_cardesian, double startPoint_time);

     /* create road edge */
    void createEdge(const std::vector<Position> &global_path, std::vector<Position> &upper_edge, std::vector<Position> &low_edge);


    /* support resample mode */
    void resampleTraje(double resample_interval, std::vector<Position> &vecTraj);
    double calcPathLength(const std::vector<Position> &vecTraj);
    void calcCurvePara(std::vector<Position> &curve_points);
    void resampleOnStraight(std::vector<Position> &vecTraj,std::vector<Position> &curve_points,double resample_interval);
    void resampleOnCurve(std::vector<Position> &vecTraj,std::vector<Position> &curve_points,double resample_interval);

   
    public:
    EmPlan();
    ~EmPlan(){};
    void emplan_process();
    
}