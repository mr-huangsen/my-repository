#include "plan.h"
#include <tf2/LinearMath/Quaternion.h>

namespace
{
geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}
}

Plan::Plan(rclcpp::Node::SharedPtr nh):nh(nh)
{    
    run_timer = nh->create_wall_timer(std::chrono::duration<double>(DataPool::getInstance()->run_dt), std::bind(&Plan::timerCallback, this));
    trajectory_path_pub = nh->create_publisher<igv_msgs::msg::LocalTrajectory>("/trajectory_path",5);
    start_point_pub = nh->create_publisher<geometry_msgs::msg::PoseStamped>("/plan_start_point",5);
    path_pub_ptr = nh->create_publisher<nav_msgs::msg::Path>("/visualized_trajectory_path",5);
    smooth_path_ptr = nh->create_publisher<nav_msgs::msg::Path>("/smooth_path",5);
}
void Plan::run()
{       
    auto m_left_bound = DataPool::getInstance()->get_left_bound();
    auto m_right_bound = DataPool::getInstance()->get_right_bound();
    auto m_obstacle_group = DataPool::getInstance()->get_obstacle_group();
    auto m_ref_line = DataPool::getInstance()->get_ref_line();
    auto m_occupancy_grid = DataPool::getInstance()->get_occupancy_grid();
    auto m_current_position = DataPool::getInstance()->get_currect_position();
    LOG_INFO("receive location data,x: {}, y: {}, phi: {}, v: {}",
        m_current_position.x,
        m_current_position.y,
        m_current_position.yaw * 180 / 3.14159,
        m_current_position.v);
    if((nh->now().seconds() - DataPool::getInstance()->location_receive_time.seconds()) > 0.2)
    {
        LOG_ERROR("no location data!!!!");
        return ;
    }
    if(m_ref_line.empty())
    {
        LOG_ERROR("no ref line!!!!");
        return ;
    }
    if(m_left_bound.empty())
    {
        LOG_ERROR("no left bound!!!!");
        return ;
    }
    if(m_right_bound.empty())
    {
        LOG_ERROR("no right bound!!!!");
        return ;
    }
    if(is_arrive(m_current_position,m_ref_line))
    {
        trajectory_path.flag_stop = 2;
        trajectory_path.waypoint.clear();
        return ;
    }
    //获取靠近当前的路径点前20米、后10米的中心线及道路边界
    //根据当前车辆位置，获取合适的路径切片
    auto ref_line_section = get_line_part(m_ref_line,m_current_position,DataPool::getInstance()->flag_is_new_task, last_ref_line_match_point_index,point_number);
    auto left_bound_section = get_line_part(m_left_bound,m_current_position,DataPool::getInstance()->flag_is_new_task, last_left_bound_match_point_index,point_number); 
    auto right_bound_section = get_line_part(m_right_bound,m_current_position,DataPool::getInstance()->flag_is_new_task, last_right_bound_match_point_index,point_number);        
    // for(int i = 0;i<ref_line_section.size();i++)
    // {
    // }
    //参考线平滑
    // if(path_smoother_ptr->run(ref_line_section))
    // {
    //     ref_line_section = path_smoother_ptr->result;
    //     smooth_path = ref_line_section;
    //     // for(int i = 0;i<ref_line_section.size();i++)
    //     // {
    //     // }
    // }else
    // {
    // }
    // CalcYawAndKappa(ref_line_section);   

    // 获取规划的起始点、待拼接轨迹
    start_plan_point = GetStartPlanPoint(m_current_position, last_local_trajectory, DataPool::getInstance()->flag_is_new_task, DataPool::getInstance()->run_dt);
    LOG_INFO("get start point! x,y,yaw: {}, {}, {}",
        start_plan_point.x,
        start_plan_point.y,
        start_plan_point.yaw);
    //规划局部路径（lattic plan）
    rclcpp::Time t1 = nh->now();
    if(DataPool::getInstance()->plan_type == 1)
    {
        plan_result = lattic_plan_ptr->run(start_plan_point, ref_line_section, left_bound_section, right_bound_section, m_obstacle_group);
        smooth_path = lattic_plan_ptr->smooth_path;
    }else if(DataPool::getInstance()->plan_type == 2)
    {
        Eigen::Vector4d x0 = {start_plan_point.x, start_plan_point.y, start_plan_point.v, start_plan_point.yaw};
        
        ReferenceLine ref_waypoints(ref_line_section);
        double target_v = DataPool::getInstance()->get_ilqr_config().target_velocity;
        std::vector<RoutingLine> obs_preds;
        obs_preds.clear();
        Eigen::Vector2d road_boaders = {10,-10};
        auto cilqr_start_time = nh->now();
        auto[u_new,x_new] = cilqr_ptr->solve(x0, ref_waypoints, target_v, obs_preds, road_boaders);
        LOG_INFO("ilqr cost time: {}", (nh->now().seconds() - cilqr_start_time.seconds()));
        plan_result = PathTransform(start_plan_point,x_new);
    }
    auto plan_time = (nh->now().seconds() - t1.seconds());
    LOG_INFO("plan time : {}", plan_time);
    
    // if(plan_time > 0.0 )
    // {
    //     for(int i = 0; i < smooth_path.size(); i++)
    //     {
    //     }        
    // }
    //拼接轨迹
    if(!plan_result.empty())
    {
        toll_trajectory = stitch_trajectory;
        toll_trajectory.insert(toll_trajectory.end(), plan_result.begin(), plan_result.begin() + plan_result.size());
        CalcOnlyKappa(toll_trajectory,stitch_trajectory.size());
    }
    else
    {
        stitch_trajectory.clear();
        toll_trajectory.clear();
    }
    LOG_INFO("stitch_trajectory size : {}plan_result size : {}toll_trajectory size: {}",
        stitch_trajectory.size(),
        plan_result.size(),
        toll_trajectory.size());
    for(int i= 0;i<toll_trajectory.size();i++)
    {
        LOG_INFO("toll_trajectory,x,y,yaw,cur,v,a: {}, {}, {}, {}, {}, {}, {}",
            i,
            toll_trajectory[i].x,
            toll_trajectory[i].y,
            toll_trajectory[i].yaw  * 180 / 3.14159,
            toll_trajectory[i].cur,
            toll_trajectory[i].v,
            toll_trajectory[i].a);
    }
    //
    LocalTrajectoryToTestPath(toll_trajectory,trajectory_path);
    last_local_trajectory = toll_trajectory;
    DataPool::getInstance()->flag_is_new_task = false;    
}
LocalTrajectory Plan::PathTransform(Position ego_state, Eigen::MatrixX4d path)
{
    LocalTrajectory local_path;
    Position temp_position;
    for(int i = 0;i<path.rows();i++)
    {
        temp_position.x = path(i,0);
        temp_position.y = path(i,1);
        temp_position.v = path(i,2);
        temp_position.yaw = path(i,3);
        temp_position.time = DataPool::getInstance()->run_dt * i * 1e6 + ego_state.time;
        local_path.emplace_back(temp_position);
    }
    return local_path;
}

bool Plan::is_arrive(FrenetPathPoint m_current_position, LocalTrajectory m_ref_line)
{
    auto the_last_point = m_ref_line.back();
    double dist = std::hypot(m_current_position.x - the_last_point.x,m_current_position.y - the_last_point.y);
    double d_phi = fabs(m_current_position.yaw - the_last_point.yaw);
    if(dist <2.0 && d_phi < 1.0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
void Plan::timerCallback()
{   
    
    run(); 
    //可视化的路径
    geometry_msgs::msg::PoseStamped temp_point;
    temp_point.header.stamp = nh->now();
    temp_point.header.frame_id = "map";
    temp_point.pose.position.x = start_plan_point.x;
    temp_point.pose.position.y = start_plan_point.y;
    temp_point.pose.orientation = createQuaternionMsgFromYaw(start_plan_point.yaw);
    start_point_pub->publish(temp_point);
    trajectory_path_pub->publish(trajectory_path);
    pub_path(toll_trajectory,path_pub_ptr);
    if(!smooth_path.empty())
        pub_path(smooth_path, smooth_path_ptr);
    
 }
void Plan::LocalTrajectoryToTestPath(LocalTrajectory trajectory, Trajectory &trajectory_path)
{
    if(trajectory.empty())
    {
        trajectory_path.waypoint.clear();
        trajectory_path.flag_stop = 2;
    }
    else
    {
        trajectory_path.waypoint.clear();
        trajectory_path.flag_stop = 0;
        igv_msgs::msg::Position temp_point;
        for(int i = 0; i < trajectory.size(); i++)
        {
            temp_point.timestamp = trajectory[i].time;
            temp_point.x = trajectory[i].x;
            temp_point.y = trajectory[i].y;
            temp_point.phi = trajectory[i].yaw;
            temp_point.v = trajectory[i].v;
            temp_point.a = trajectory[i].a;
            temp_point.kappa = trajectory[i].cur;
            trajectory_path.waypoint.emplace_back(temp_point);          
        }
        
    }
}

 void Plan::pub_path(const LocalTrajectory path, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub)
{   
    nav_msgs::msg::Path temp_path;
    temp_path.header.stamp = nh->now();
    temp_path.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped temp_point;
    for(int i = 0;i < path.size();i++)
    {
        temp_point.pose.position.x = path[i].x;
        temp_point.pose.position.y = path[i].y;
        temp_point.pose.orientation = createQuaternionMsgFromYaw(path[i].yaw);
        temp_path.poses.push_back(temp_point);
    }
    pub->publish(temp_path);
    //可视化的路径
 }
  
Position Plan::GetStartPlanPoint(Position current_position,LocalTrajectory last_local_trajectory,bool IsNewTask,double dt)
{
    stitch_trajectory.clear(); 
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());

    current_position.time = duration.count(); 

    if(IsNewTask)
    {             
        current_position.time += 0.1*1e6;
        return current_position;
    }
    else
    {
        Position new_position;
        if(last_local_trajectory.empty())
        {
            LOG_ERROR("last_local_trajectory is empty !");
            current_position.time += 0.1*1e6;
            return current_position;
        }
        auto desire_position_index = FindPointByTime(last_local_trajectory,dt);
        auto desire_position = last_local_trajectory[desire_position_index];  

        LOG_INFO("desire_position_index: {}", desire_position_index);  

        if(ControlWeak(desire_position,current_position))
        {
            new_position = desire_position;  
            LOG_INFO("desire_position, x,y,v,yaw,cur: {}, {}, {}, {}, {}",
                desire_position.x,
                desire_position.y,
                desire_position.v,
                desire_position.yaw,
                desire_position.cur);
            // desire_position_index = desire_position_index-1;    
            if(desire_position_index >= 20)
            {
                std::copy(last_local_trajectory.begin() + desire_position_index - 20, last_local_trajectory.begin() + desire_position_index, std::back_inserter(stitch_trajectory));
            }else{
                std::copy(last_local_trajectory.begin(), last_local_trajectory.begin() + desire_position_index, std::back_inserter(stitch_trajectory));
            }                
        }
        else
        {
            auto now = std::chrono::system_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());

            new_position.x = current_position.x + current_position.v * cos(current_position.yaw) * dt;
            new_position.y = current_position.y + current_position.v * sin(current_position.yaw) * dt;
            new_position.yaw = current_position.yaw + current_position.dot_yaw * dt;
            new_position.v = current_position.v;
            new_position.vx = current_position.vx;
            new_position.vy = current_position.vy;
            new_position.vz = current_position.vz;
            new_position.a = current_position.a;
            new_position.ax = current_position.ax;
            new_position.ay = current_position.ay;
            new_position.dot_yaw = current_position.dot_yaw;  
            new_position.cur = 0.0; 
            new_position.time = duration.count() + 0.1*1e6;  
            LOG_INFO("location point is the start point.");              
        }
        return new_position;
    }
}


bool Plan::ControlWeak(Position desire_position, Position current_position)
{
    double error_d = std::hypot(desire_position.x - current_position.x,desire_position.y - current_position.y);
    double error_p = fabs(desire_position.yaw - current_position.yaw);
    if((error_d < 5.0) && (error_p < 30.0 / 180.0 * 3.14159))
    {
        return true;//控住了
    }
    else
    {
        return false;//没控住
    }
}
int Plan::FindPointByTime(LocalTrajectory local_trajectory, double dt)
{
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());    
    uint64_t now_time = duration.count();   
    for(int i = 0;i<local_trajectory.size()-1;i++)
    {        
        if((now_time * 1.0 / 1e6 + dt  >= local_trajectory[i].time  * 1.0 / 1e6) && (now_time  * 1.0 / 1e6 + dt < local_trajectory[i+1].time  * 1.0 / 1e6))
        {
            return i;
        }
    }
    return 0;
}

LocalTrajectory Plan::get_line_part(LocalTrajectory path, FrenetPathPoint m_current_position,bool &IsRecNewLine, int &last_match_point_index, int waypoint_number){
    
    double min_distance = 1e6;
    double dx,dy,distance,last_distance;
    float distance_forward = DataPool::getInstance()->distance_forward;
    float distance_back = DataPool::getInstance()->distance_back;
    int index,increas_number;
    //暂时不考虑倒车、后退
    if(IsRecNewLine)
    {        
        for(int i = 0; i<path.size();i++)
        {            
            dx = path[i].x - m_current_position.x;
            dy = path[i].y - m_current_position.y;
            distance = std::hypot(dx,dy);
            if(distance < min_distance)
            {
                min_distance = distance;
                index = i;
            }
            if(last_distance < distance)
            {
                increas_number += 1; 
            }
            else
            {
                increas_number = 0;
            }
            if(increas_number > 50)
            {
                break;
            }
            last_distance = distance;
        }        
        // IsRecNewLine = false;
    }
    else
    {
        index = last_match_point_index;
        for(int i = index-20; i<index+20;i++)
        {
            if(i < 0)
            {
                i = 0;
            }
            if(i > path.size()-1)
            {                
                break;
            }
            dx = path[i].x - m_current_position.x;
            dy = path[i].y - m_current_position.y;
            distance = std::hypot(dx,dy);
            if(distance < min_distance)
            {
                min_distance = distance;
                index = i;
            }
        }        
    }
    last_match_point_index = index;   
    //从index开始，往前取100个点，往后取50个点
    int forword_point_number = static_cast<int> (2 * waypoint_number / 3);
    int back_point_number = waypoint_number - forword_point_number;

    if(path.size() < waypoint_number)
    {
        LOG_WARN(" path point number is smaller than {} !", waypoint_number);
        return path;
    }
    std::vector<FrenetPathPoint> result_path;
    

    if(index <  back_point_number)
    {
        std::copy(path.begin(), path.begin() + waypoint_number, std::back_inserter(result_path));
    }else if(index > path.size() - forword_point_number)
    {
        std::copy(path.begin() + path.size() - waypoint_number, path.begin() + path.size(), std::back_inserter(result_path));
    }else
    {
        std::copy(path.begin() + index - back_point_number, path.begin() + index + forword_point_number, std::back_inserter(result_path));
    }   
    if(result_path.size() != waypoint_number)
    {
        LOG_INFO("result_path.size: {}", result_path.size());
        throw std::invalid_argument("path point number is error");
    }
    return result_path;

}

std::vector<FrenetPathPoint> Plan::get_line_part(std::vector<FrenetPathPoint> path, FrenetPathPoint m_current_position,bool &IsRecNewLine, int &last_match_point_index){
    
    double min_distance = 1e6;
    double dx,dy,distance,last_distance;
    float distance_forward = DataPool::getInstance()->distance_forward;
    float distance_back = DataPool::getInstance()->distance_back;
    int index,increas_number;
    //暂时不考虑倒车、后退
    if(IsRecNewLine)
    {        
        for(int i = 0; i<path.size();i++)
        {            
            dx = path[i].x - m_current_position.x;
            dy = path[i].y - m_current_position.y;
            distance = std::hypot(dx,dy);
            if(distance < min_distance)
            {
                min_distance = distance;
                index = i;
            }
            if(last_distance < distance)
            {
                increas_number += 1; 
            }
            else
            {
                increas_number = 0;
            }
            if(increas_number > 50)
            {
                break;
            }
            last_distance = distance;
        }        
        // IsRecNewLine = false;
    }
    else
    {
        index = last_match_point_index;
        for(int i = index-20; i<index+20;i++)
        {
            if(i < 0)
            {
                i = 0;
            }
            if(i > path.size()-1)
            {                
                break;
            }
            dx = path[i].x - m_current_position.x;
            dy = path[i].y - m_current_position.y;
            distance = std::hypot(dx,dy);
            if(distance < min_distance)
            {
                min_distance = distance;
                index = i;
            }
        }        
    }
    last_match_point_index = index;   
    
    //从index开始，往前取30米，往后取15米    
    int start_index;
    for(int i = index;i>=0;i--)
    {
        if(i == 0)
        {
            start_index = i;
            break;
        }
        dx = path[i].x - path[index].x;
        dy = path[i].y - path[index].y;
        distance = std::hypot(dx,dy);
        if(distance >= distance_back)
        {            
            start_index = i;
            break;
        }
    }    
    int end_index;
    for(int i = index;i<path.size();i++)
    {
        if(i == (path.size()-1))
        {
            end_index = i;
            break;
        }
        dx = path[i].x - path[index].x;
        dy = path[i].y - path[index].y;
        distance = std::hypot(dx,dy);
        if(distance >= distance_forward)
        {            
            end_index = i;
            break;
        }
    }
    std::vector<FrenetPathPoint> result_path;
    std::copy(path.begin()+start_index, path.begin() + end_index + 1, std::back_inserter(result_path));
    
    return result_path;

}

void Plan::CalcYawAndKappa(LocalTrajectory &local_trajectory)
{
    //使用中值欧拉计算
    std::vector<double> dx;
    std::vector<double> dy;
    dx.resize(local_trajectory.size()-1,0);
    dy.resize(local_trajectory.size()-1,0);
    for(int i = 0;i<local_trajectory.size()-1;i++)
    {
        dx[i] = local_trajectory[i+1].x - local_trajectory[i].x;
        dy[i] = local_trajectory[i+1].y - local_trajectory[i].y;
    }
    std::vector<double> dx_1 = dx;
    std::vector<double> dx_2 = dx;
    std::vector<double> dy_1 = dy;
    std::vector<double> dy_2 = dy;
    dx_1.insert(dx_1.begin(), dx.front());
    dx_2.push_back(dx.back());
    dy_1.insert(dy_1.begin(), dy.front());
    dy_2.push_back(dy.back());    
    std::vector<double> ds;
    double dx_f,dy_f;
    for(int i = 0;i<local_trajectory.size();i++)
    {
        dx_f = (dx_1[i] + dx_2[i])/2;
        dy_f = (dy_1[i] + dy_2[i])/2;        
        ds.push_back(std::hypot(dx_f,dy_f));
        local_trajectory[i].yaw = atan2(dy_f,dx_f);
    }
    
    std::vector<double> dphi;

    for(int i = 0;i<local_trajectory.size()-1;i++)
    {
        dphi.push_back(local_trajectory[i+1].yaw - local_trajectory[i].yaw);
    }
    std::vector<double> dphi_1 = dphi;
    std::vector<double> dphi_2 = dphi;
    dphi_1.insert(dphi_1.begin(),dphi.front());
    dphi_2.push_back(dphi.back());
    double dphi_f;
    for(int i = 0;i<local_trajectory.size();i++)
    {
        dphi_f = (dphi_1[i] + dphi_2[i]) / 2;
        local_trajectory[i].cur = sin(dphi_f) / ds[i];
    }
}
void Plan::CalcOnlyKappa(LocalTrajectory &local_trajectory, const int start_index)
{
    const std::size_t point_count = local_trajectory.size();
    if (point_count == 0)
    {
        return;
    }
    if (point_count == 1)
    {
        local_trajectory.front().cur = 0.0;
        return;
    }

    constexpr double kMinDistance = 1e-8;
    for (std::size_t i = 0; i < point_count; ++i)
    {
        const std::size_t left_begin = (i == 0) ? 0 : i - 1;
        const std::size_t left_end = (i == 0) ? 1 : i;
        const std::size_t right_begin = (i + 1 == point_count) ? i - 1 : i;
        const std::size_t right_end = (i + 1 == point_count) ? i : i + 1;

        const auto &left_p0 = local_trajectory[left_begin];
        const auto &left_p1 = local_trajectory[left_end];
        const auto &right_p0 = local_trajectory[right_begin];
        const auto &right_p1 = local_trajectory[right_end];

        const double dx = 0.5 * ((left_p1.x - left_p0.x) +
                                 (right_p1.x - right_p0.x));
        const double dy = 0.5 * ((left_p1.y - left_p0.y) +
                                 (right_p1.y - right_p0.y));
        const double dphi = 0.5 * ((left_p1.yaw - left_p0.yaw) +
                                   (right_p1.yaw - right_p0.yaw));
        const double ds = std::hypot(dx, dy);

        local_trajectory[i].cur =
            (ds > kMinDistance) ? std::sin(dphi) / ds : 0.0;
    }
}
