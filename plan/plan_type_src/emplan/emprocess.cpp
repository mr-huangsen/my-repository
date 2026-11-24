#include "emplan/emprocess.h"

EmPlan::EmPlan()
{

}
void EmPlan::emplan_process(Position vehicle, Position start_plan_point, LocalTrajectory ref_path, ObstacleGroup obs_group, LocalTrajectory left_bound, LocalTrajectory right_bound, DPConfig dp_config, QPConfig qp_config){

    // covert start point and obstacles to frenet coordinate
    //get index_s, 0 is current position
    int fre_match_index = getMatchPoint(vehicle,-1,ref_path);
    Position veh_match_point = ref_path.at(fre_match_index);
    Position veh_project_point;
    getProjectPoint(vehicle,veh_match_point,veh_project_point);
    Eigen::VectorXd index_S;
    index2S(fre_match_index,veh_project_point,ref_path,index_S);
    //    cout<<index_S<<endl;

    //start plan point 
    int spp_match_index = getMatchPoint(start_plan_point,-1,ref_path);
    Position spp_match_point = ref_path.at(spp_match_index);
    Position spp_project_point;
    getProjectPoint(start_plan_point,spp_match_point,spp_project_point);
    Position spp_frenet;
    cardesian2Frenet(start_plan_point,spp_match_index,spp_project_point,ref_path,index_S,spp_frenet);

    // vector<Position> obs_array_frenet;
    // for(auto &obs:new_obs_array)
    // {
    //     int obs_match_index = getMatchPoint(obs,-1,ref_path);
    //     Position obs_match_point = ref_path.at(obs_match_index);
    //     Position obs_project_point;
    //     getProjectPoint(obs,obs_match_point,obs_project_point);
    //     Position obs_frenet;
    //     cardesian2Frenet(obs,obs_match_index,obs_project_point,ref_path,index_S,obs_frenet);
    //     obs_array_frenet.push_back(obs_frenet);
    // }
    
    auto static_obs_group = obsSelect(vehicle,obs_group);

    for(auto &obs:static_obs_group)
    {
        getFrenetPoint(obs.center,ref_path,index_S);
        for(auto &point:obs.collision_box)
        {
            getFrenetPoint(point,ref_path,index_S);
        }        
    }
    // get init plan value
    // DP
    auto init_path = getInitPlanValue(static_obs_group,spp_frenet,dp_config);
    trajectoryInterp(init_path);
    vector<Position> DP_path;
    frenet2Cardesian(init_path, ref_path,index_S,DP_path);

    // QP plan
    Eigen::VectorXd low_bound,upper_bound;
    getCovexBound(init_path,static_obs_group,qp_config,low_bound,upper_bound);


    final_path_solver.updateBound(low_bound,upper_bound,spp_frenet);
    // final_path_solver.updateBoundWithDpPath(low_bound,upper_bound,init_path,spp_frenet);
    vector<Position> qp_frenet_path;
    final_path_solver.getNewPath(qp_frenet_path,spp_frenet);
    vector<Position> qp_final_path;
    frenet2Cardesian(qp_frenet_path,ref_path,index_S,qp_final_path);
    timeAndVelocity(qp_final_path,vehicle.time+0.1);


    // stitch the pre_plan path
    vector<Position> final_path;
    copy(stitch_path.begin(),stitch_path.end(), back_inserter(final_path));
    final_path.insert(final_path.end(),qp_final_path.begin(),qp_final_path.end());

}

double EmPlan::NormalizeAngle(const double angle) {
    
    angle = std::fmod (angle,2*M_PI);
    angle = angle > M_PI ? (angle - 2*M_PI):angle;
    angle = angle < -M_PI ? (angle + 2*M_PI):angle;
    return angle;
}
void getFrenetPoint(Position &point, LocalTrajectory ref_path, Eigen::VectorXd index_S)
{
    int obs_match_index = getMatchPoint(point,-1,ref_path);
    Position obs_match_point = ref_path.at(obs_match_index);
    Position obs_project_point;
    getProjectPoint(point,obs_match_point,obs_project_point);
    cardesian2Frenet(point,obs_match_index,obs_project_point,ref_path,index_S);
    
}

int EmPlan::getMatchPoint(Position hostPoint, int prePointIdx,const std::vector<Position> &vecTraj)
{
    if(prePointIdx == -1)
    {
        std::vector<double> distVec;
        for(int i = 0;i<vecTraj.size()-1;++i)
        {
            double tempDist = pow(hostPoint.x - vecTraj.at(i).x,2)+ pow(hostPoint.y - vecTraj.at(i).y,2);
            distVec.push_back(tempDist);
        }
        auto itr = std::min_element(distVec.begin(),distVec.end());
        return static_cast<int>(std::distance(distVec.begin(),itr));
    }
    else{

        double preDist = 0,dist = 0,nextDist = 0;
        int process = 0;
        int minIdx = 0;
        int dir = 0;

        // determine direction
        std::vector<double> vec = {hostPoint.x - vecTraj.at(prePointIdx).x, hostPoint.y - vecTraj.at(prePointIdx).y};
        std::vector<double> matchDir = {cos(vecTraj.at(prePointIdx).yaw), sin(vecTraj.at(prePointIdx).yaw)};
        double innerProd = vec.at(0)*matchDir.at(0)+vec.at(1)*vec.at(1);
        dir = innerProd > 0? 1:(innerProd < 0)?-1:0;

        // find match point forward or backward
        if(dir != 0)
        {
            int idx = prePointIdx;
            preDist = 1e10;
            dist = pow(hostPoint.x - vecTraj.at(prePointIdx).x,2)+ pow(hostPoint.y - vecTraj.at(prePointIdx).y,2);
            nextDist = pow(hostPoint.x - vecTraj.at(prePointIdx+dir).x,2)+ pow(hostPoint.y - vecTraj.at(prePointIdx+dir).y,2);
            while (idx >= 0 && idx <vecTraj.size())
            {
                preDist = dist;
                dist = pow(hostPoint.x - vecTraj.at(idx).x,2)+ pow(hostPoint.y - vecTraj.at(idx).y,2);
                if( dist<preDist)
                {
                    process = 0;
                    minIdx = idx;
                }
                else if(dist > preDist)
                {
                    process++;
                }
                else{}
                if (process >= 5) break;
                idx = idx + dir;
            }
        } else{
            minIdx = prePointIdx;
        }
        return minIdx;
    }
}

void EmPlan::getProjectPoint(Position hostPoint, const Position &matchPoint, Position &projectPoint)
{
    Eigen::Vector2d hostPos = {hostPoint.x,hostPoint.y},matchPointPos = {matchPoint.x,matchPoint.y};
    Eigen::Vector2d d = hostPos - matchPointPos;
    Eigen::Vector2d tao = {cos(matchPoint.yaw), sin(matchPoint.yaw)};
    Eigen::Vector2d projectPose = matchPointPos+tao*(d.transpose()*tao);
    projectPoint.cur = matchPoint.cur;
    projectPoint.yaw = matchPoint.yaw+matchPoint.cur*static_cast<double>((d.transpose()*tao));
    projectPoint.x = projectPose[0];
    projectPoint.y = projectPose[1];
}

void EmPlan::getProjectPoint(const Position &frenet_point,  const std::vector<Position> &ref_trajectory, const Eigen::VectorXd &index_S,Position &projectPoint)
{
    int match_index = -1;
    for(int i = 0;i<index_S.size()-1;++i)
    {
        if(index_S[i]<frenet_point.s && index_S[i+1]>frenet_point.s)
            match_index = i;
    }
    if(match_index == -1)
        return;
    Position match_point = ref_trajectory.at(match_index);
    Eigen::Vector2d r_n = {match_point.x,match_point.y};
    Eigen::Vector2d tao_n = {cos(match_point.yaw),sin(match_point.yaw)};
    double ds = frenet_point.s - index_S[match_index];

    Eigen::Vector2d r_r = r_n + ds*tao_n;
    projectPoint.x = r_r[0];
    projectPoint.y = r_r[1];
    projectPoint.yaw = match_point.yaw + ds*match_point.cur;
    projectPoint.cur = (match_point.cur + ref_trajectory.at(match_index+1).cur)/2;
}

double EmPlan::getDistance(const Position &p1,const Position &p2)
{
    return std::hypot(p1.x - p2.x,p1.y-p2.y);
}

double EmPlan::calcSFromIndex2S(int matchPoint_index_local,const Position &project_point,const std::vector<Position> &frenet_trajectory,const Eigen::VectorXd &index_S)
{
    double heading = frenet_trajectory.at(matchPoint_index_local).yaw;
    Eigen::Vector2d tangent = {cos(heading), sin(heading)};
    Eigen::Vector2d dir = {project_point.x - frenet_trajectory.at(matchPoint_index_local).x,project_point.y - frenet_trajectory.at(matchPoint_index_local).y};
    double s_temp = getDistance(frenet_trajectory.at(matchPoint_index_local),project_point);
    double s0 = dir.dot(tangent) >0?index_S[matchPoint_index_local]+s_temp:index_S[matchPoint_index_local]-s_temp;
    return s0;
}

void EmPlan::index2S(int matchPoint_index_local,const Position &project_point,const std::vector<Position> &frenet_trajectory,Eigen::VectorXd &index_S)
{
    int size = static_cast<int>(frenet_trajectory.size());
    index_S.resize(size);
    index_S[0] = 0;
    for(int i = 1;i<frenet_trajectory.size();++i)
    {
        index_S[i] = getDistance(frenet_trajectory.at(i),frenet_trajectory.at(i-1))+index_S[i-1];
    }
    double s0 = calcSFromIndex2S(matchPoint_index_local,project_point,frenet_trajectory,index_S);
    index_S = index_S - s0*Eigen::VectorXd::Ones(index_S.size());
}

void EmPlan::cardesian2Frenet(const Position &cardesian_point, int match_point_index, const Position &project_point, const std::vector<Position> &ref_trajectory,const Eigen::VectorXd &index_S,Position &frenet_pos,cardesian2FrentMode mode)
{
    if(ref_trajectory.empty())
    {
        std::cout<<"reference line is empty!"<<std::endl;
        return;
    }
    Eigen::Vector2d r_h = {cardesian_point.x,cardesian_point.y};
    Eigen::Vector2d r_r = {project_point.x,project_point.y};
    Eigen::Vector2d tao_r = {cos(project_point.yaw),sin(project_point.yaw)};
    Eigen::Vector2d n_r = {-sin(project_point.yaw), cos(project_point.yaw)};
    Eigen::Vector2d n_h = {-sin(cardesian_point.yaw), cos(cardesian_point.yaw)};


    frenet_pos.s = calcSFromIndex2S(match_point_index,project_point,ref_trajectory,index_S);

    frenet_pos.l = (r_h - r_r).dot(n_r);

    


    if(mode == firstInfo || mode == secondInfo)
    {
        double delta_theta = cardesian_point.yaw - project_point.yaw;

        double v = cardesian_point.v;

        frenet_pos.l_d = v * sin(delta_theta);

        frenet_pos.s_d = (1/(1-frenet_pos.l*project_point.cur))* v * cos(delta_theta);

        frenet_pos.l_ds = tan(delta_theta) * (1 - project_point.cur * frenet_pos.l);

        if(mode == secondInfo)
        {
            double dkappa = 0.0;

            Eigen::Vector2d a = {cardesian_point.ax,cardesian_point.ay};

            frenet_pos.l_dd = a.dot(n_r) - project_point.cur*(1-project_point.cur*frenet_pos.l)*pow(frenet_pos.s_d,2);
            // because d_k/ds is small, use 0 replace
            
            frenet_pos.s_dd =(1/(1-frenet_pos.l*project_point.cur))*(a.dot(tao_r)+2*project_point.cur*frenet_pos.l_ds*pow(frenet_pos.s_d,2) + dkappa * frenet_pos.l * pow(frenet_pos.s_d, 2));
            if(fabs(frenet_pos.s_d)<1e-8)
                frenet_pos.l_dds = 0;
            else
                frenet_pos.l_dds = (1/pow(frenet_pos.s_d,2))*(frenet_pos.l_dd - frenet_pos.l_ds*frenet_pos.s_dd);
        }
        return;
    }
}
void EmPlan::cardesian2Frenet(const Position &point, int match_point_index, const Position &project_point, const std::vector<Position> &ref_trajectory,const Eigen::VectorXd &index_S,cardesian2FrentMode mode)
{
    if(ref_trajectory.empty())
    {
        std::cout<<"reference line is empty!"<<std::endl;
        return;
    }
    Eigen::Vector2d r_h = {point.x,point.y};
    Eigen::Vector2d r_r = {project_point.x,project_point.y};
    Eigen::Vector2d tao_r = {cos(project_point.yaw),sin(project_point.yaw)};
    Eigen::Vector2d n_r = {-sin(project_point.yaw), cos(project_point.yaw)};
    Eigen::Vector2d n_h = {-sin(point.yaw), cos(point.yaw)};


    point.s = calcSFromIndex2S(match_point_index,project_point,ref_trajectory,index_S);

    point.l = (r_h - r_r).dot(n_r);

    


    if(mode == firstInfo || mode == secondInfo)
    {
        double delta_theta = point.yaw - project_point.yaw;

        double v = point.v;

        point.l_d = v * sin(delta_theta);

        point.s_d = (1/(1-point.l*project_point.cur))* v * cos(delta_theta);

        point.l_ds = tan(delta_theta) * (1 - project_point.cur * point.l);

        if(mode == secondInfo)
        {
            double dkappa = 0.0;

            Eigen::Vector2d a = {point.ax,point.ay};

            point.l_dd = a.dot(n_r) - project_point.cur*(1-project_point.cur*point.l)*pow(point.s_d,2);
            // because d_k/ds is small, use 0 replace
            
            point.s_dd =(1/(1-point.l*project_point.cur))*(a.dot(tao_r)+2*project_point.cur*point.l_ds*pow(point.s_d,2) + dkappa * point.l * pow(point.s_d, 2));
            if(fabs(point.s_d)<1e-8)
                point.l_dds = 0;
            else
                point.l_dds = (1/pow(point.s_d,2))*(point.l_dd - point.l_ds*point.s_dd);
        }
        return;
    }
}

// using dp algorithm to calculate the init path.

double EmPlan::calcNeighborCost(const ObstacleGroup &obstacle_array,const Position &start_point, const Position &end_point, const DPConfig &config)
{
    Eigen::VectorXd coeff(6);
    calcQuinticCoeff(start_point,end_point,coeff);
    
    //use 10 points to calculate the cost
    double cost_sum = 0;
    for(int i = 0;i<10;++i)
    {
        double ds = start_point.s + i*config.sample_s/10;
        double ds_2 = ds*ds, ds_3 = ds_2*ds, ds_4 = ds_3*ds, ds_5 = ds_4*ds;
        double l = coeff[0] + coeff[1]*ds + coeff[2]*ds_2 + coeff[3]*ds_3 + coeff[4]*ds_4 + coeff[5]*ds_5;
        double l_diff = coeff[1] + 2*coeff[2]*ds + 3*coeff[3]*ds_2 + 4*coeff[4]*ds_3 + 5*coeff[5]*ds_4;
        double l_2diff = 2*coeff[2] + 6*coeff[3]*ds + 12*coeff[4]*ds_2 + 20*coeff[5]*ds_3;
        double l_3diff = 6*coeff[3] + 24*coeff[4]*ds + 60*coeff[5]*ds_2;
        
        double cost_smooth = config.w_cost_dl*pow(l_diff,2) + config.w_cost_ddl*pow(l_2diff,2)+config.w_cost_dddl*pow(l_3diff,2);
        double cost_ref = config.w_cost_ref* pow(l,2);
        double cost_collision = 0;
        for(const auto &obstacle:obstacle_array)
        {
            Eigen::Vector2d pose_error = {obstacle.center.s - ds, obstacle.center.l - l};
            double square_dist = pose_error.dot(pose_error); // Not Euclidean distance, just approximate
            double cost_collision_once = calcObsCost(config.w_cost_obs,square_dist);
            cost_collision += cost_collision_once;
        }

        cost_sum = cost_sum + cost_smooth + cost_ref + cost_collision;
    }

}


double EmPlan::calcObsCost(double w_cost_obs, double square_dist)
{
    // if dist>4 cost = 0; if dist in [3,4], cost = 1000/square_dist; if dist<3, cost = w_cost_obs
    double cost = 0;
    if(square_dist >= 9 && square_dist <= 16)
        cost = 1000/square_dist;
    else if(square_dist < 9)
        cost = w_cost_obs;
    else
        cost = 0;
    return cost;

}

void EmPlan::calcQuinticCoeff(const Position &start_point, const Position &end_point, Eigen::VectorXd &coeff)
{
    // l = a0 + a1*s +a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5;
    // l' = a1 + 2*a2*s + 3*a3*s^2 + 4*a4*s^3 + 5*a5*s^4
    // l'' = 2*a2 + 6*a3*s + 12*a4*s^2 + 20*a5*s^3

    Eigen::Matrix<double,6,6> A;
    Eigen::VectorXd B(6);
    double S_2 = start_point.s * start_point.s;
    double S_3 = S_2 * start_point.s;
    double S_4 = S_3 * start_point.s;
    double S_5 = S_4 * start_point.s;

    double E_2 = end_point.s * end_point.s;
    double E_3 = E_2 * end_point.s;
    double E_4 = E_3 * end_point.s;
    double E_5 = E_4 * end_point.s;

    A<<1, start_point.s,   S_2,   S_3, S_4, S_5,
       0,  1, 2*start_point.s, 3*S_2,  4*S_3, 5*S_4,
       0, 0, 2, 6*start_point.s, 12*S_2, 20*S_3,
       1, end_point.s,   E_2,   E_3, E_4, E_5,
       0, 1, 2*end_point.s, 3*E_2,  4*E_3, 5*E_4,
       0, 0, 2, 6*end_point.s, 12*E_2, 20*E_3;
    
    B<<start_point.l, start_point.l_ds,start_point.l_dds,end_point.l,end_point.l_ds,end_point.l_dds;
    coeff = A.inverse()*B;
}


LocalTrajectory EmPlan::getInitPlanValue(const ObstacleGroup &obstacle_array,const Position &start_point, const DPConfig &config)
{
    LocalTrajectory init_path;
    init_path.clear();
    std::vector<std::vector<costElement> > cost_matrix;
    cost_matrix.resize(config.rows);
    for(int i = 0;i<config.rows;++i)
    {
        cost_matrix[i].resize(config.cols);
    }
    for(int i = 0;i<config.rows;i++)
    {
        double l = (config.rows/2 - i)*config.sample_l;
        for(int j = 0;j<config.cols;j++)
        {
            cost_matrix[i][j].pose.l = l;
            cost_matrix[i][j].pose.s = start_point.s+(j+1)*config.sample_s;
            cost_matrix[i][j].pose.l_ds = 0;
            cost_matrix[i][j].pose.l_dds = 0;
        }
    }

    // calculate the first column cost
    for(int i = 0;i<config.rows;++i)
    {
        double cost = calcNeighborCost(obstacle_array,start_point,cost_matrix[i][0].pose,config);
        cost_matrix[i][0].miniCost = cost;
    }

    for(int j = 1;j<config.cols;++j)
    {
        for(int i = 0;i<config.rows;++i)
        {
            for(int k = 0;k<config.rows;++k)
            {
                double cost_temp = calcNeighborCost(obstacle_array,cost_matrix[k][j-1].pose,cost_matrix[i][j].pose,config);
                double pre_mini_cost = cost_matrix[k][j-1].miniCost;
                double cost_cur = pre_mini_cost+cost_temp;
                if(cost_cur < cost_matrix[i][j].miniCost)
                {
                    cost_matrix[i][j].miniCost = cost_cur;
                    cost_matrix[i][j].preRow = k;
                    cost_matrix[i][j].preCol = j-1;
                }
            }
        }
    }

    int mini_cost_row = 0;
    int mini_cost_col = config.cols -1;
    double mini_cost_last = 1e8;
    for(int i = 0;i<config.rows;++i)
    {

        if(cost_matrix[i][mini_cost_col].miniCost < mini_cost_last)
        {
            mini_cost_last = cost_matrix[i][mini_cost_col].miniCost;
            mini_cost_row = i;
        }
    }

    init_path.push_back(cost_matrix[mini_cost_row][mini_cost_col].pose);
    for(int i = 0;i<config.cols-1;++i)
    {
        mini_cost_row = cost_matrix[mini_cost_row][mini_cost_col].preRow;
        mini_cost_col = cost_matrix[mini_cost_row][mini_cost_col].preCol;
        init_path.push_back(cost_matrix[mini_cost_row][mini_cost_col].pose);
    }
    init_path.push_back(start_point);
    std::reverse(init_path.begin(),init_path.end());
    return init_path;
}

// interpolate the init_path, point is taken every 1m of s, maximum 60m
void EmPlan::trajectoryInterp(std::vector<Position> &init_path)
{
    if (init_path.empty())
        return;
    std::vector<Position> ori_path = init_path;
    init_path.clear();
    init_path.reserve(61);
    double ds = 1;
    init_path.push_back(ori_path.at(0));
    double s_cur = ori_path.at(0).s + ds;
    int count = 1;
    for(int i = 0;i<ori_path.size()-1;++i)
    {
        Eigen::VectorXd coeff(6);
        Position p1 = ori_path.at(i), p2 = ori_path.at(i+1);
        calcQuinticCoeff(p1,p2,coeff);
        while(s_cur<ori_path.at(i+1).s)
        {
            Position point_curr;
            point_curr.s = s_cur;
            double s_2 = s_cur*s_cur , s_3 = s_2*s_cur , s_4 = s_3*s_cur , s_5 = s_4*s_cur ;
            point_curr.l= coeff[0] + coeff[1]*s_cur + coeff[2]*s_2 + coeff[3]*s_3 + coeff[4]*s_4 + coeff[5]*s_5;
            point_curr.l_ds = coeff[1] + 2*coeff[2]*s_cur + 3*coeff[3]*s_2 + 4*coeff[4]*s_3 + 5*coeff[5]*s_4;
            point_curr.l_dds = 2*coeff[2] + 6*coeff[3]*s_cur + 12*coeff[4]*s_2 + 20*coeff[5]*s_3;
            init_path.push_back(point_curr);
            s_cur += ds;
            count++;
            if(count>61)
                return;
        }
    }
}

void EmPlan::frenet2Cardesian(const std::vector<Position> &dense_init_path, const std::vector<Position> &ref_trajectory,const Eigen::VectorXd &index_S, std::vector<Position> &init_path_cardesian)
{
    if(dense_init_path.empty())
        return;
    init_path_cardesian.clear();
    init_path_cardesian.reserve(dense_init_path.size());
    for(const auto &frenet_point:dense_init_path)
    {
        Position project_point,cur_point;
        getProjectPoint(frenet_point,ref_trajectory,index_S,project_point);
        Eigen::Vector2d r_r = {project_point.x,project_point.y};
        Eigen::Vector2d tau_r = {cos(project_point.yaw),sin(project_point.yaw)};
        Eigen::Vector2d n_r = {-sin(project_point.yaw), cos(project_point.yaw)};

        Eigen::Vector2d r_h = r_r + frenet_point.l*n_r;
        cur_point.x = r_h[0];
        cur_point.y = r_h[1];
        cur_point.yaw = project_point.yaw+ atan2(frenet_point.l_ds,(1-frenet_point.l*project_point.cur));
        cur_point.yaw = NormalizeAngle(cur_point.yaw);

        double d_theta = cur_point.yaw - project_point.yaw;
        double dkappa = 0.0;// dk_r/ds is thought to be 0
        cur_point.cur = ((frenet_point.l_dds+(project_point.cur*frenet_point.l_ds + dkappa * frenet_point.l)* tan(d_theta))*
                      pow(cos(d_theta),2)*(1/(1 - project_point.cur*frenet_point.l))+project_point.cur)*
                              cos(d_theta)*(1/(1 - project_point.cur*frenet_point.l));
        init_path_cardesian.push_back(cur_point);
    }
}

void EmPlan::timeAndVelocity(std::vector<Position> &init_path_cardesian, double startPoint_time)
{
    if(init_path_cardesian.empty())
        return;
    int count = 0;
    for(auto &waypoint_:init_path_cardesian)
    {
        waypoint_.time = startPoint_time + count*0.02;
        count++;
    }
}
ObstacleGroup EmPlan::obsSelect(Position currentvehicle, ObstacleGroup obs_group)
{
    ObstacleGroup static_obs_group;
    static_obs_group.clear();    
    Eigen::Vector2d tor = {cos(currentvehicle.yaw),sin(currentvehicle.yaw)};
    Eigen::Vector2d nor = {-sin(currentvehicle.yaw),cos(currentvehicle.yaw)};
    for(const auto &obstacle : obs_group)
    {
        if(obstacle.center.v > 0.2)
            continue;
        // if(obstacle.obs_status == ObsStatus::handled_left || obstacle.obs_status == ObsStatus::handled_right)
        //     continue;
        Eigen::Vector2d pose_error = {obstacle.center.x - currentvehicle.x,obstacle.center.y - currentvehicle.y};
        double lon_error = pose_error.dot(tor);
        double lat_error = pose_error.dot(nor);
        if((lon_error > -10 && lon_error<60) && (lat_error > -10 && lat_error<10))
            static_obs_group.emplace_back(obstacle);
    }
    return static_obs_group;
}

void EmPlan::obsSelect(const Position &currentvehicle,const std::vector<Position> &obstacle_array_original, std::vector<Position> &obstacle_array_selected)
{
    obstacle_array_selected.clear();
    obstacle_array_selected.reserve(32);
    Eigen::Vector2d tor = {cos(currentvehicle.yaw),sin(currentvehicle.yaw)};
    Eigen::Vector2d nor = {-sin(currentvehicle.yaw),cos(currentvehicle.yaw)};
    for(const auto &obstacle : obstacle_array_original)
    {
        Eigen::Vector2d pose_error = {obstacle.x - currentvehicle.x,obstacle.y - currentvehicle.y};
        double lon_error = pose_error.dot(tor);
        double lat_error = pose_error.dot(nor);
        if((lon_error > -10 && lon_error<60) && (lat_error > -10 && lat_error<10))
            obstacle_array_selected.push_back(obstacle);
    }
}

int EmPlan::getMatchSIndex(const std::vector<Position> &init_path,double s)
{
    int count = 0,index = -1;
    if(s<init_path.at(0).s)
        return index = 0;
    else if(s>init_path.back().s)
        return index = 59;
    else
    {
        for(auto &point:init_path)
        {
            if(point.s>s)
                break;
            count++;
        }
        if((init_path.at(count).s - s) > (s - init_path.at(count-1).s))
            return index = count;
        else
            return index = count-1;
    }
}

// obstacle_length = 5, obstacle_width = 2
void EmPlan::getCovexBound(const std::vector<Position> &init_path, ObstacleGroup &obstacle_array,const QPConfig &config,Eigen::VectorXd &low_bound, Eigen::VectorXd &upper_bound)
{
    if(init_path.empty())
        return;
    low_bound = -6*Eigen::VectorXd::Ones(60); //right bound of vehicle
    upper_bound = 6*Eigen::VectorXd::Ones(60); //left bound of vehicle
    for(auto &obs:obstacle_array)
    {
        double obs_s_min = obs.center.s - obs.halfExtents.x;
        double obs_s_max = obs.center.s + obs.halfExtents.x;
        int index_s_min = getMatchSIndex(init_path,obs_s_min);
        int index_s_max = getMatchSIndex(init_path,obs_s_max);
        int index_obs = getMatchSIndex(init_path,obs.s);

        if(index_s_min==0 || index_s_max == 0)
            continue;
        if(obs.obs_status == ObsStatus::handled_left)//由上一帧计算左转
        {
            for(int i = index_s_min;i<=index_s_max;++i)
                low_bound[i] =std::max(obs.l + config.obs_width/2.0,low_bound[i]);
        }else if(obs.obs_status == ObsStatus::handled_right)//由上一帧计算右转
        {
            for(int i = index_s_min;i<=index_s_max;++i)
                upper_bound[i] = std::min(obs.l - config.obs_width/2.0,upper_bound[i]);
        }else{//本帧的计算结果
            if(init_path.at(index_obs).l>obs.l) // dp-path on left of obstacles
            {
                obs.obs_status = ObsStatus::handled_left;
                for(int i = index_s_min;i<=index_s_max;++i)
                    low_bound[i] =std::max(obs.l + config.obs_width/2.0,low_bound[i]);
            }
            else // dp-path on right of obstacles
            {
                obs.obs_status = ObsStatus::handled_right;
                for(int i = index_s_min;i<=index_s_max;++i)
                    upper_bound[i] = std::min(obs.l - config.obs_width/2.0,upper_bound[i]);
            }
        }
        
    }
}

void EmPlan::createEdge(const std::vector<Position> &global_path, std::vector<Position> &upper_edge, std::vector<Position> &low_edge)
{
    upper_edge.reserve(global_path.size());
    low_edge.reserve(global_path.size());

    for(const auto & point : global_path)
    {
        Position temp_point;
        Eigen::Vector2d pos = {point.x,point.y};
        Eigen::Vector2d tor = {-sin(point.yaw), cos(point.yaw)};
        Eigen::Vector2d upper_point = pos+6*tor;
        temp_point.x = upper_point[0];
        temp_point.y = upper_point[1];
        temp_point.yaw = point.yaw;
        temp_point.cur = point.cur;
        upper_edge.push_back(temp_point);

        upper_point = pos-6*tor;
        temp_point.x = upper_point[0];
        temp_point.y = upper_point[1];
        temp_point.yaw = point.yaw;
        temp_point.cur = point.cur;
        low_edge.push_back(temp_point);
    }
}

/*







*/
/* support resample mode */
void EmPlan::resampleTraje(double resample_interval, std::vector<Position> &vecTraj)
{
    std::vector<Position> original_trajectory(vecTraj);
    vecTraj.clear();
    vecTraj.push_back(original_trajectory.at(0));
    vecTraj.reserve(ceil(1.5*calcPathLength(original_trajectory)/resample_interval));
    for(int i = 1;i<original_trajectory.size();++i)
    {
        std::vector<Position> curve_points = {vecTraj.back(),
                                              original_trajectory.at(i),
                                              i<original_trajectory.size()-1? original_trajectory.at(i+1):original_trajectory.back()};
        calcCurvePara(curve_points);
        if(curve_points.at(1).cur == 0)
        {
            resampleOnStraight(vecTraj,curve_points,resample_interval);
        }
        else{
            resampleOnCurve(vecTraj,curve_points,resample_interval);
        }
    }

}

double EmPlan::calcPathLength(const std::vector<Position> &vecTraj)
{
    double length = 0;
    for(int i = 0;i<vecTraj.size()-1;++i)
    {
        Eigen::Vector2d point_vector = {vecTraj.at(i+1).x - vecTraj.at(i).x,vecTraj.at(i+1).y - vecTraj.at(i).y};
        length += point_vector.norm();
    }
    return length;
}

void EmPlan::calcCurvePara(std::vector<Position> &curve_points){
    if(curve_points.size() != 3)
        return;
    Position &p0 = curve_points.at(0), &p1 = curve_points.at(1),&p2 = curve_points.at(2);
    double d =2*((p0.y-p2.y)*(p0.x-p1.x)-(p0.y-p1.y)*(p0.x-p2.x));
    if(fabs(d)<1e-8)
    {
        p1.cur = 0;
        std::vector<double> Line = {(p1.x - p0.x),(p1.y - p0.y)};
        p1.yaw = atan2(Line.at(1),Line.at(0));
        return;

    }
    double a = p0.y*p0.y - p1.y*p1.y + p0.x*p0.x-p1.x*p1.x;
    double b = p0.y*p0.y - p2.y*p2.y + p0.x*p0.x-p2.x*p2.x;

    double cx = ((p0.y - p2.y)*a - (p0.y - p1.y)*b)/d;
    double cy = ((p0.x - p2.x)*a - (p0.x - p1.x)*b)/(-d);

    double dx = cx - p1.x;
    double dy = cy - p1.y;
    double R = sqrt(dx*dx+dy*dy);
    p1.cur = 1/R;
    p0.cur = 1/R;

    std::vector<double> differVector = {p2.x - p1.x,p2.y - p1.y};
    std::vector<double> dirVec = {-dy/R,dx/R};
    if((differVector.at(0)*dirVec.at(0)+ differVector.at(1)*dirVec.at(1))<0)
    {
        dirVec.at(0) = -dirVec.at(0);
        dirVec.at(1) = -dirVec.at(1);
    }
    p1.yaw = atan2(dirVec.at(1),dirVec.at(0));

    // calculate k and dir for p0
    dx = cx - p0.x;
    dy = cy - p0.y;
    differVector = {p1.x - p0.x,p1.y - p0.y};
    dirVec = {-dy/R,dx/R};
    if((differVector.at(0)*dirVec.at(0)+ differVector.at(1)*dirVec.at(1))<0)
    {
        dirVec.at(0) = -dirVec.at(0);
        dirVec.at(1) = -dirVec.at(1);
    }
    p0.yaw = atan2(dirVec.at(1),dirVec.at(0));
}

void EmPlan::resampleOnStraight(std::vector<Position> &vecTraj,std::vector<Position> &curve_points,double resample_interval){
    if(curve_points.size() != 3)
        return;
    Position prePoint(vecTraj.back());
    Eigen::Vector2d vecDiff = {curve_points.at(1).x - prePoint.x,curve_points.at(1).y - prePoint.y};
    double dist = vecDiff.norm();
    double coeff = resample_interval/dist;
    vecDiff[0] *= coeff;
    vecDiff[1] *= coeff;
    for(;dist>resample_interval;dist -= resample_interval)
    {
        prePoint.x += vecDiff[0];
        prePoint.y += vecDiff[1];
        vecTraj.push_back(prePoint);
    }
}

void EmPlan::resampleOnCurve(std::vector<Position> &vecTraj,std::vector<Position> &curve_points,double resample_interval)
{
    if(curve_points.size() != 3)
        return;
    Position prePoint(curve_points.at(0));
    double R = 1/curve_points.at(1).cur;
    int dir = 0;

    // judge whither clockwise or anticlockwise
    Eigen::Vector2d p0 = {cos(prePoint.yaw), sin(prePoint.yaw)};
    Eigen::Vector2d p1 = {cos(curve_points.at(1).yaw), sin(curve_points.at(1).yaw)};

    double cross = p0[0]*p1[1]- p0[1]*p1[0];
    double dot = p0.dot(p1);
    double theta = acos(dot/(p0.norm()*p1.norm()));
    if(cross>0)
    {
        dir = 1;
    }
    else
    {
        dir = -1;//clockwise
    }

    double dist = fabs(theta)*R;
    double theta_diff = resample_interval*curve_points.at(1).cur;
    for(;dist>resample_interval;dist -= resample_interval){

        if(vecTraj.size() == vecTraj.capacity())
            break;

        Eigen::Vector2d vec = {cos(prePoint.yaw+dir*theta_diff/2), sin(prePoint.yaw+dir*theta_diff/2)};
        vec = 2*R* sin(theta_diff/2)*vec;
        prePoint.yaw += dir*theta_diff;
        prePoint.x += vec[0];
        prePoint.y += vec[1];
        vecTraj.push_back(prePoint);
    }
}