#include "common/frenet_change.h"

namespace frenet_change{

std::vector<double> CalcS(const std::vector<FrenetPathPoint> &ref_path)
{
    std::vector<double> path_point_s;
    path_point_s.push_back(0.0);
    double t_s = 0;
    double t_sum;
    for(int i = 1;i<ref_path.size();i++)
    {
        t_s = CalcDist(ref_path[i].x,ref_path[i].y,ref_path[i-1].x,ref_path[i-1].y);
        t_sum += t_s;
        path_point_s.push_back(t_sum);
    }
    return path_point_s;
}

double CalcDist(double x1, double y1, double x2, double y2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::hypot(dx,dy);
}

int AcquireMatchIdx(const double &cur_x, const double &cur_y,
                           const std::vector<FrenetPathPoint> &path,
                           int &pre_match_index, bool &IsNewPath)
{
    double dist;
    double min_dist = DBL_MAX;
    int match_index = 0;
    if(IsNewPath)
    {
        for (int i = 0; i < path.size(); i++)
        {        
            dist = CalcDist(path[i].x, path[i].y, cur_x, cur_y);
            if (dist < min_dist)
            {
                min_dist = dist;
                match_index = i;
            }
        }
    }
    else
    {
        for (int i = pre_match_index-10; i < pre_match_index +10; i++)
        {
            if(i < 0)
            {
                i = 0;
            }
            if(i>path.size()-1)
            {
                break;
            }
            dist = CalcDist(path[i].x, path[i].y, cur_x, cur_y);
            if (dist < min_dist)
            {
                min_dist = dist;
                match_index = i;
            }
        }
    }
    
    pre_match_index = match_index;
    return match_index;
}
FrenetPathPoint Cartesian2FrenetOnlyPosition(const CartesianCarInfo &point,const std::vector<FrenetPathPoint> &path,
    const int match_index, const std::vector<double> path_point_s)
    {
        FrenetPathPoint frenet_point;
        CartesianPathPoint match_point = path[match_index]; 

        Eigen::Vector2d temp_v;
        temp_v << point.x - match_point.x, point.y - match_point.y;
        Eigen::Vector2d v_tou;
        v_tou << cos(match_point.yaw),sin(match_point.yaw);
        double es = v_tou.dot(temp_v);
        Eigen::Vector2d v_n;
        v_n << -sin(match_point.yaw),cos(match_point.yaw);        
        double el = v_n.dot(temp_v);
        
        // CartesianPathPoint projection_point;
        // projection_point.x = match_point.x + es * cos(match_point.yaw);
        // projection_point.y = match_point.y + es * sin(match_point.yaw);
        // projection_point.yaw = match_point.yaw + match_point.cur * es;
        // Eigen::Vector2d v_n_p;
        // v_n_p << -sin(projection_point.yaw),cos(projection_point.yaw);
        // Eigen::Vector2d temp_v_p;
        // temp_v_p << point.x - projection_point.x, point.y - projection_point.y;
        // double el_p = v_n_p.dot(temp_v_p);
        
        frenet_point.x = point.x;
        frenet_point.y = point.y;
        frenet_point.z = point.z;
        frenet_point.yaw = point.yaw;
        frenet_point.cur = point.cur;

        frenet_point.vx = point.vx;
        frenet_point.vy = point.vy;
        frenet_point.v = point.v;
        frenet_point.ax = point.ax;
        frenet_point.ay = point.ay;
        frenet_point.a = point.a;
        frenet_point.s = path_point_s[match_index] + es;
        frenet_point.l = el;
        return frenet_point;
    }

CartesianPathPoint AcquireProjectPoint(const CartesianCarInfo &cur_pose,
                                   const CartesianPathPoint &match_point){
      
    CartesianPathPoint projection_point = match_point;    
    Eigen::Matrix<double, 2, 1> tor;
    tor << cos(match_point.yaw), sin(match_point.yaw);   
    Eigen::Matrix<double, 2, 1> d;
    d << cur_pose.x - match_point.x, cur_pose.y - match_point.y;
    double e_s = tor.transpose() * d;
    projection_point.x = match_point.x + e_s * cos(match_point.yaw);
    projection_point.y = match_point.y + e_s * sin(match_point.yaw);
    projection_point.yaw = match_point.yaw + match_point.cur * e_s;
    
    return projection_point;
}



FrenetPathPoint Cartesian2Frenet(const CartesianCarInfo &global_point,
                                 const std::vector<FrenetPathPoint> &path,const int match_index, const std::vector<double> &path_point_s){
       
    FrenetPathPoint frenet_point;
        
    CartesianPathPoint match_point = path[match_index]; 

    Eigen::Vector2d temp_v;
    temp_v << global_point.x - match_point.x, global_point.y - match_point.y;
    Eigen::Vector2d v_tou;
    v_tou << cos(match_point.yaw),sin(match_point.yaw);
    double es = v_tou.dot(temp_v);
    Eigen::Vector2d v_n;
    v_n << -sin(match_point.yaw),cos(match_point.yaw);        
    double el = v_n.dot(temp_v);
        
    CartesianPathPoint projection_point;
    projection_point.x = match_point.x + es * cos(match_point.yaw);
    projection_point.y = match_point.y + es * sin(match_point.yaw);
    projection_point.yaw = match_point.yaw + match_point.cur * es;      
    projection_point.cur = match_point.cur;
    frenet_point.x = global_point.x;
    frenet_point.y = global_point.y;
    frenet_point.z = global_point.z;
    frenet_point.yaw = global_point.yaw;
    frenet_point.cur = global_point.cur;
    frenet_point.time = global_point.time;
    frenet_point.vx = global_point.vx;
    frenet_point.vy = global_point.vy;
    frenet_point.v = global_point.v;
    frenet_point.ax = global_point.ax;
    frenet_point.ay = global_point.ay;
    frenet_point.a = global_point.a;
    frenet_point.s = path_point_s[match_index] + es;
     if (frenet_point.s > 1000) {
        LOG_ERROR("frenet_point.s: {}, es: {}, v_tou: [{}, {}], temp_v: [{}, {}]",
            frenet_point.s, es, v_tou.x(), v_tou.y(), temp_v.x(), temp_v.y());
        for(int i = 0; i < path_point_s.size() ; i++)
        {
            LOG_ERROR("s: {}, refer path x : {}, y: {}", path_point_s[i], path[i].x, path[i].y);
        }
        
        throw std::invalid_argument("s is so big!!!");
    }
    double delta_theta = global_point.yaw - projection_point.yaw;

    Eigen::Matrix<double, 2, 1> x_or;
    x_or << global_point.x, global_point.y;
    
    Eigen::Matrix<double, 2, 1> x_r_or;
    x_r_or << projection_point.x, projection_point.y;

    Eigen::Matrix<double, 2, 1> n_or;
    n_or << -sin(projection_point.yaw), cos(projection_point.yaw);

    int sign_frenet_point_l = (x_or - x_r_or).transpose() * n_or >0?1:-1;
    frenet_point.l = sign_frenet_point_l*(x_or - x_r_or).transpose().norm();


    frenet_point.s_d = global_point.v * std::cos(delta_theta) /
                           (1 - projection_point.cur * frenet_point.l);
    frenet_point.l_d = global_point.v * std::sin(delta_theta);
   
    // if (fabs(frenet_point.s_d) < 1e-6)
    // {
    //     frenet_point.l_ds = 0;
    //     std::cout<<"The dos(s) is smaller than 1e-6, let l_ds be 0."<<std::endl;
    // }
    // else
    // {
    //     frenet_point.l_ds = frenet_point.l_d / frenet_point.s_d;
    // }
  
    frenet_point.l_ds = tan(delta_theta) * (1 - projection_point.cur * frenet_point.l);

    Eigen::Matrix<double, 2, 1> t_or;
    t_or << cos(projection_point.yaw), sin(projection_point.yaw);
    n_or << -sin(projection_point.yaw), cos(projection_point.yaw);
    Eigen::Matrix<double, 2, 1> a_or;
    a_or << global_point.ax, global_point.ay;

    //忽略曲率变化
    double dkappa = 0;
    frenet_point.s_dd = (a_or.transpose() * t_or +
                              2 * projection_point.cur * frenet_point.l_ds * pow(frenet_point.s_d, 2) +
                            dkappa * frenet_point.l * pow(frenet_point.s_d, 2)) /
                             (1 - projection_point.cur * frenet_point.l);

    frenet_point.l_dd = a_or.dot(n_or) - projection_point.cur*(1-projection_point.cur*frenet_point.l)*pow(frenet_point.s_d,2);

    if(fabs(frenet_point.s_d < 1e-6))
    {
        frenet_point.l_dds = 0;
    }
    else
    {
        frenet_point.l_dds = (frenet_point.l_dd - frenet_point.l_ds * frenet_point.s_dd) /pow(frenet_point.s_d, 2);
    }
    return frenet_point;
}

FrenetPathPoint CalcFrenet(const CartesianCarInfo &global_point,
                            const std::vector<FrenetPathPoint> &ref_path){ 
    bool IsNewPath = true;                                
    auto path_point_s_list = CalcS(ref_path);
    int pre_match_index = 0;
    int frenet_match_index = AcquireMatchIdx(global_point.x, global_point.y, ref_path, pre_match_index,IsNewPath); 
    FrenetPathPoint frenet_point = Cartesian2Frenet(global_point, ref_path,frenet_match_index,path_point_s_list);
    return frenet_point;
}

std::vector<FrenetPathPoint> CalcFrenet(const std::vector<FrenetPathPoint> &target_path,
    const std::vector<FrenetPathPoint> &ref_path)
    {
        int pre_match_index;
        bool IsNewPath = true; 
        std::vector<FrenetPathPoint> result_path;
        auto path_point_s_list = CalcS(ref_path);
        for(auto global_point:target_path)
        {
            int frenet_match_index = AcquireMatchIdx(global_point.x, global_point.y, ref_path, pre_match_index,IsNewPath);   
            FrenetPathPoint frenet_point = Cartesian2Frenet(global_point, ref_path,frenet_match_index,path_point_s_list);
            result_path.emplace_back(frenet_point);
        }
        return result_path;
        
    } 

FrenetPathPoint CalcFrenetOnlyPosition(const CartesianCarInfo &global_point,
                            const std::vector<FrenetPathPoint> &ref_path){ 
    bool IsNewPath = true;                                
    auto path_point_s_list = CalcS(ref_path);
    int pre_match_index = 0;
    int frenet_match_index = AcquireMatchIdx(global_point.x, global_point.y, ref_path, pre_match_index,IsNewPath);   
    auto frenet_point = Cartesian2FrenetOnlyPosition(global_point,ref_path,frenet_match_index,path_point_s_list);
    return frenet_point;
} 

FrenetPathPoint CalcFrenet(const CartesianCarInfo &global_point,
    const std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, Spline2D> &ref_path)
    {
        auto rx = std::get<0>(ref_path);
        auto ry = std::get<1>(ref_path);
        auto ryaw = std::get<2>(ref_path);
        auto rc = std::get<3>(ref_path);
        std::vector<FrenetPathPoint> temp_line;
        FrenetPathPoint t_p;
        for(int i = 0; i < rx.size(); i++)
        {
            t_p.x = rx[i];
            t_p.y = ry[i];
            t_p.yaw = ryaw[i];
            t_p.cur = rc[i];
            temp_line.emplace_back(t_p);
        }
        return CalcFrenet(global_point,temp_line);
    }
  
}
