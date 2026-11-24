#include "control/workflow.h"
#include <rclcpp/create_timer.hpp>

namespace {
geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
}
}

Workflow::Workflow(rclcpp::Node::SharedPtr nh):nh(nh){
    timer = rclcpp::create_timer(nh, nh->get_clock(), std::chrono::milliseconds(20),
            std::bind(&Workflow::timerCallback, this));
    timer_2 = rclcpp::create_timer(nh, nh->get_clock(), std::chrono::seconds(1),
            std::bind(&Workflow::timerCallback_pub, this));
    cmd_vel_pub = nh->create_publisher<igv_msgs::msg::ControlInformation>("/cmd_vel",5);
    control_status_pub = nh->create_publisher<igv_msgs::msg::ControlStatus>("/control_status",5);
    refer_trajectory_pub = nh->create_publisher<nav_msgs::msg::Path>("/visualized_reference_trajectory",10);
    real_trajectory_pub = nh->create_publisher<nav_msgs::msg::Path>("/visualized_real_trajectory",10);
    favoritism_path_pub = nh->create_publisher<nav_msgs::msg::Path>("/favoritism_trajectory",10);
    relay_control_pub = nh->create_publisher<std_msgs::msg::String>("/voice_keyword",10);
    trapped_cnt = 0;
    relay_cmd_change_timestamp = nh->now();
    turnlight_cmd_change_timestamp = nh->now();
    park_start_time = nh->now();
    last_relay_cmd = "stop";
    last_turn_light = 0;
    start_brake_speed = 0.0;
    flag_brk_press = false; 

}
Workflow::~Workflow(){

}
void Workflow::run(){
    
    auto m_current_position = DataPool::getInstance()->get_currect_position();  
    auto m_odom_data = DataPool::getInstance()->get_odom_data();
    auto m_max_v = DataPool::getInstance()->get_lon_param();
    auto m_max_angle = DataPool::getInstance()->get_lat_param();
    auto m_local_trajectory = DataPool::getInstance()->get_local_trajectory();
    
    if(JudgeIsTrapped(m_current_position))
    {
        clear();        
        return ;
    } 
    // if((nh->now().seconds() - DataPool::getInstance()->location_receive_time.seconds()) > 0.2)
    // {
    //     clear();
    //     LOG_ERROR("not receive location data");
    //     return ;        
    // }
    if (DataPool::getInstance()->is_calibration_mode) 
    {
        control_result = DataPool::getInstance()->get_calibration_cmd();
        if ((nh->now() - DataPool::getInstance()->calibration_cmd_receive_time).seconds() > 0.5) {
            LOG_INFO("EXIT calibration mode, no calibration cmd received for 0.5s");
            DataPool::getInstance()->is_calibration_mode = false;
            clear();
        }
        return;
    }
    if(!m_local_trajectory.waypoint.empty() && DataPool::getInstance()->get_control_status()->location_warp == 0)    
    {  
        LOG_INFO("location receive success, x: ,y: ,yaw: {}, {}, {}",
                  m_current_position.x, m_current_position.y, m_current_position.phi);

        PointPrediction(m_current_position.x,m_current_position.y,m_current_position.phi,m_current_position.v,m_current_position.dot_phi);
        
        int min_idx = 0;
        
        if(DataPool::getInstance()->flag_refer_line_follow_type)
        {
            FindClosestPointByTime(min_idx,m_local_trajectory);
        }
        else{
            FindClosestPointByDistance(min_idx,m_local_trajectory,m_current_position);
        }
        
        
        LOG_INFO("track point index: {}", min_idx);
        if(JudgeIsArrive(min_idx, m_current_position,m_local_trajectory))
        {
            DataPool::getInstance()-> clear_path();
            clear();                            
            return ;
        }
             

        Position ref_point,ref_point_center;
        ref_point.x = m_local_trajectory.waypoint[min_idx].x;    
        ref_point.y = m_local_trajectory.waypoint[min_idx].y; 
        ref_point.phi = m_local_trajectory.waypoint[min_idx].phi;
        ref_point.v = m_local_trajectory.waypoint[min_idx].v; 
        ref_point.a = m_local_trajectory.waypoint[min_idx].a;
        ref_point.kappa = m_local_trajectory.waypoint[min_idx].kappa;
        ref_point.dot_phi = ref_point.kappa * ref_point.v;

        int temp_index;        
        ref_point_center.x = m_local_trajectory.waypoint[min_idx].x + DataPool::getInstance()->get_vehicle_param().lr * cos(m_local_trajectory.waypoint[min_idx].phi);
        ref_point_center.y = m_local_trajectory.waypoint[min_idx].y + DataPool::getInstance()->get_vehicle_param().lr * sin(m_local_trajectory.waypoint[min_idx].phi);
        ref_point_center.phi = m_local_trajectory.waypoint[min_idx].phi;
        ref_point_center.v = m_local_trajectory.waypoint[min_idx].v;
        ref_point_center.a = m_local_trajectory.waypoint[min_idx].a;        
        FindClosestPointByDistance(temp_index,m_local_trajectory,ref_point_center);
        ref_point_center.kappa = m_local_trajectory.waypoint[temp_index].kappa;
        ref_point_center.dot_phi = ref_point_center.kappa * ref_point_center.v;
        // LOG_INFO("ref_point.kappa: {}, ref_point_center.kappa: {}", ref_point.kappa, ref_point_center.kappa);
        Position current_position_center;
        current_position_center.x = m_current_position.x + DataPool::getInstance()->get_vehicle_param().lr * cos(m_current_position.phi);
        current_position_center.y = m_current_position.y + DataPool::getInstance()->get_vehicle_param().lr * sin(m_current_position.phi);
        current_position_center.phi = m_current_position.phi;
        current_position_center.dot_phi = m_current_position.dot_phi;
        current_position_center.v = m_current_position.v;
        current_position_center.a = m_current_position.a;
        current_position_center.steer = m_current_position.steer;
        current_position_center.roll = m_current_position.roll;
        current_position_center.pitch = m_current_position.pitch;
        current_position_center.phi = m_current_position.phi;
        //debug
        geometry_msgs::msg::PoseStamped debug_refer_point;
        debug_refer_point.header.frame_id = "map";
        debug_refer_point.header.stamp = nh->now();
        debug_refer_point.pose.position.x = ref_point.x;
        debug_refer_point.pose.position.y = ref_point.y;    
        debug_refer_point.pose.orientation = quaternion_from_yaw(ref_point.phi);
        geometry_msgs::msg::PoseStamped debug_real_point;
        debug_real_point.header.frame_id = "map";
        debug_real_point.header.stamp = nh->now();
        debug_real_point.pose.position.x = m_current_position.x;
        debug_real_point.pose.position.y = m_current_position.y;    
        debug_real_point.pose.orientation = quaternion_from_yaw(m_current_position.phi);
        debug_inf.refer_trajectory.header.frame_id = "map";
        debug_inf.refer_trajectory.header.stamp = nh->now();
        debug_inf.real_trajectory.header.frame_id = "map";
        debug_inf.real_trajectory.header.stamp = nh->now();
        if(debug_inf.refer_trajectory.poses.size() <= debug_inf.max_size)
        {            
            debug_inf.refer_trajectory.poses.emplace_back(debug_refer_point);
            debug_inf.real_trajectory.poses.emplace_back(debug_real_point);
        }
        else
        {
            debug_inf.refer_trajectory.poses.erase(debug_inf.refer_trajectory.poses.begin());
            debug_inf.real_trajectory.poses.erase(debug_inf.real_trajectory.poses.begin());
            debug_inf.refer_trajectory.poses.emplace_back(debug_refer_point);
            debug_inf.real_trajectory.poses.emplace_back(debug_real_point);
        }
        Position lookahead_point;
        //lon
        control_result = m_lon_control_ptr->lon_calculate(m_current_position,ref_point);
        //lat
        if(2 == DataPool::getInstance()->control_alg_type || DataPool::getInstance()->path_type != 0)// pure pursuit type
        {
            // lookfor lookahead point        
            double lookahead_dist = LookaheadDistance(m_current_position,min_idx,m_local_trajectory,m_max_angle);
            m_control_pp->lookahead_dist = lookahead_dist;
            int lookahead_point_idx = LookaheadPointIdx(min_idx,m_local_trajectory,lookahead_dist,m_current_position);
            lookahead_point.x = m_local_trajectory.waypoint[lookahead_point_idx].x;
            lookahead_point.y = m_local_trajectory.waypoint[lookahead_point_idx].y;
            lookahead_point.phi = m_local_trajectory.waypoint[lookahead_point_idx].phi;
            lookahead_point.v = m_local_trajectory.waypoint[lookahead_point_idx].v;
            lookahead_point.a = m_local_trajectory.waypoint[lookahead_point_idx].a;    
            lookahead_point.kappa = m_local_trajectory.waypoint[lookahead_point_idx].kappa;
            lookahead_point.dot_phi = lookahead_point.kappa * lookahead_point.v;        
            control_result.angle = m_control_pp->lat_calculate(m_current_position,ref_point,lookahead_point);
        }else{
            if(0 == DataPool::getInstance()->control_alg_type)// lqr type   
            {
                // control_result.angle = m_control_lqr->lat_calculate(current_position_center, ref_point_center, lookahead_point);
                control_result.angle = m_control_lqr->lat_calculate(m_current_position, ref_point, lookahead_point);
            }else if(1 == DataPool::getInstance()->control_alg_type)// mpc type
            {
                m_control_mpc->u_lat_ << m_current_position.steer;      
                control_result.angle = m_control_mpc->lat_calculate(m_current_position,ref_point,lookahead_point); 
                // control_result.angle = m_control_mpc->lat_calculate(current_position_center,ref_point_center,lookahead_point); 
            }
        }
       
        control_result.angle = control_result.angle * 180 / 3.14159; 
        
        flag_brk_press = false;

        if(1 == DataPool::getInstance()->get_control_status()->deviation) 
        {
            clear(); 
            DataPool::getInstance()-> clear_path();
            LOG_ERROR("Vehicle deviation!!!!");
        }
                     
    }
    else
    {
        clear();
        debug_inf.clear();
        // LOG_INFO("no trajectory!!!!!!");
    }
    
}

double Workflow::LookaheadDistance(Position current_point,double min_idx,LocalTrajectory local_plan,LateralParam latp){
    double dl ;
    double k_speed = 1.0;
    double k_kappa = 4.0;
    double k_dist = 1.0;
    double dl_speed = k_speed * current_point.v;
    double dl_kappa = -k_kappa * std::fabs(local_plan.waypoint[min_idx].kappa);
    double dist = std::hypot(local_plan.waypoint[min_idx].x - current_point.x,local_plan.waypoint[min_idx].y - current_point.y);
    double dl_dist = 0;
    if(dist > 0.5)
    {
        dl_dist = k_dist * dist;
    }
    dl = latp.min_lookahead_dist + dl_speed + dl_kappa + dl_dist;    
    dl = std::clamp(dl,latp.min_lookahead_dist,latp.max_lookahead_dist);  
    LOG_INFO("Look ahead Distance:  {}", dl);
    return dl;
}
int Workflow::LookaheadPointIdx(int min_idx,const LocalTrajectory local_plan,double lookahead_dist,Position current_point)
{
    double pp_dist;
    for(int i = min_idx;i<local_plan.waypoint.size();i++)
    {
        if(i == (local_plan.waypoint.size()-1))
        {
            LOG_INFO("LookaheadPoint is the last point");
            return i;            
        }
        pp_dist = std::hypot(local_plan.waypoint[i].x-current_point.x,local_plan.waypoint[i].y-current_point.y); 
        if(pp_dist > lookahead_dist)
        {
            return i;            
        }
    }
    throw std::out_of_range("LookaheadPointIdx requires a valid index in a nonempty trajectory");
}

void Workflow::PointPrediction(double& x, double& y, double& phi, double v, double dot_phi)
{
    double delay_time = DataPool::getInstance()->prediction_time;
    x = x + v * delay_time * cos(phi);
    y = y + v * delay_time * sin(phi);
    phi = phi + dot_phi * delay_time;
}

void Workflow::FindClosestPointByTime(int &min_idx, const LocalTrajectory local_trajectory)
{
    double delay_time = DataPool::getInstance()->prediction_time;
    uint64_t now_time = nh->now().nanoseconds() / 1000;
    for(int i = 0;i<local_trajectory.waypoint.size()-1;i++)
    {        
        if((now_time * 1.0 / 1e6 + delay_time  >= local_trajectory.waypoint[i].timestamp  * 1.0 / 1e6) && (now_time  * 1.0 / 1e6 + delay_time < local_trajectory.waypoint[i+1].timestamp  * 1.0 / 1e6))
        {
            min_idx = i;
            return ;
        }
    }           
}
void Workflow::FindClosestPointByDistance(int &min_idx,const LocalTrajectory m_local_trajectory,Position m_current_position)
{
    // double min_dist = 1e10;    
    // for(int i=0;i<m_local_trajectory.waypoint.size();i++){
            
    //     double dist = std::hypot(m_local_trajectory.waypoint[i].x-m_current_position.x,m_local_trajectory.waypoint[i].y-m_current_position.y);
    //     if(dist<min_dist){
    //         min_dist = dist;
    //         min_idx = i;    
    //     }
    // }
    std::vector<int> indices(m_local_trajectory.waypoint.size());
    std::iota(indices.begin(),indices.end(),0);

    boost::tuple<Position, int> min_pair = std::reduce(

            boost::make_zip_iterator(
                boost::make_tuple(m_local_trajectory.waypoint.begin(), indices.begin())
            ),
            boost::make_zip_iterator(
                boost::make_tuple(m_local_trajectory.waypoint.end(), indices.end())
            ),
            boost::make_tuple(m_local_trajectory.waypoint.front(), indices.front()),
            [=] (const boost::tuple<Position, int>& a,
                                   const boost::tuple<Position, int>& b) {
                auto dist_a = std::hypot(boost::get<0>(a).x - m_current_position.x, boost::get<0>(a).y - m_current_position.y); 
                auto dist_b = std::hypot(boost::get<0>(b).x - m_current_position.x, boost::get<0>(b).y - m_current_position.y);

                return dist_a < dist_b ? a : b;
                
            }
    );
    min_idx = boost::get<1>(min_pair);

    if((m_current_position.v < 0.5) && (min_idx < m_local_trajectory.waypoint.size()-2))
        {
            min_idx = min_idx + 1;
        }
}
bool Workflow::JudgeIsArrive(int min_idx,Position m_current_position,const LocalTrajectory m_local_trajectory)
{
    if(min_idx == (m_local_trajectory.waypoint.size()-1))
    {
        double temp_dist = std::hypot(m_current_position.x - m_local_trajectory.waypoint.back().x, m_current_position.y - m_local_trajectory.waypoint.back().y);
        if(temp_dist < 0.15 && fabs(m_current_position.phi - m_local_trajectory.waypoint.back().phi) < 0.095 && fabs(m_current_position.v) < 0.5)
        {
            LOG_INFO("Arriving at the finish line !!!!!!! ");
            DataPool::getInstance()->stop_type = 2;
            return true;
        }        
    }
    if(DataPool::getInstance()->path_type != 0)
    {
        if(min_idx >= DataPool::getInstance()->vaild_path_index)
        {
            LOG_INFO("Arriving at the park line ! ");
            DataPool::getInstance()->stop_type = 2;
            return true;
        }        
    }
    
    return false;    
}

void Workflow::timerCallback()
    {        
        run();
        
        std_msgs::msg::String relay_control;
        if(control_result.speed >= 0.2 && fabs(control_result.angle) <= 7.5)
        {
            relay_control.data = "move_ahead";
        }
        else if(control_result.speed > 0.0 && control_result.angle > 7.5)
        {
            relay_control.data = "turn_left";
            control_result.turnlight = 1;
        }else if(control_result.speed > 0.0 && control_result.angle < -7.5)
        {
            relay_control.data = "turn_right";
            control_result.turnlight = 2;
        }else if(control_result.speed < 0.0)
        {
            relay_control.data = "move_back";
            control_result.turnlight = 0;
        }else{
            relay_control.data = "stop";
            control_result.turnlight = 0;
        }

        if((nh->now() - DataPool::getInstance()->turnlight_command_receive_time).seconds() < 0.2)
        {
            if((control_result.angle > 3.0 && 2 == DataPool::getInstance()->turnligth_command_by_decision) || (control_result.angle < -3.0 && 1 == DataPool::getInstance()->turnligth_command_by_decision))
            {
                LOG_WARN(" control angle and turnlight command mismatch !!!!, use control angle to control turnlight");
            }
            else
            {
                if(1 == DataPool::getInstance()->turnligth_command_by_decision)
                {
                    relay_control.data = "turn_left";
                    control_result.turnlight = 1;  
                }else if(2 == DataPool::getInstance()->turnligth_command_by_decision)
                {
                    relay_control.data = "turn_right";
                    control_result.turnlight = 2;
                }
            }
            
        }

        if(last_relay_cmd != relay_control.data)
        {
            if((nh->now()-relay_cmd_change_timestamp).seconds()<1.0)
            {
                relay_control.data = last_relay_cmd;                
            }else
            {
                last_relay_cmd = relay_control.data;
                relay_cmd_change_timestamp = nh->now();
            }
        }else
        {
            last_relay_cmd = relay_control.data;
        }

        if(last_turn_light != control_result.turnlight)
        {
            if((nh->now()-turnlight_cmd_change_timestamp).seconds()<1.0)
            {
                control_result.turnlight = last_turn_light;
            }else
            {
                last_turn_light = control_result.turnlight;
                turnlight_cmd_change_timestamp = nh->now();
            }
        }else
        {
            last_turn_light = control_result.turnlight;
        }

        auto temp_status = DataPool::getInstance()->get_control_status();
        std_msgs::msg::String stop_relay_control;
        stop_relay_control.data = "stop";
        if(DataPool::getInstance()->chassis_status.vcu_mode != 2)
        {
            relay_control_pub->publish(stop_relay_control);
        }else
        {
            relay_control_pub->publish(relay_control);
        }

        if(DataPool::getInstance()->path_type != 0 && 
           DataPool::getInstance()->is_stop_turn_wheel == 1 && 
           (nh->now() - DataPool::getInstance()->path_receive_time).seconds() < 2.0)
        {
            control_result.speed = 0.0;
        }


        if(DataPool::getInstance()->escape_topic_data == "AEB_run" && !JudgeIsParking() && !DataPool::getInstance()->is_calibration_mode)
        {
            // control_result = CalcAEBCmd();
            clear();
            DataPool::getInstance()->clear_path();
            control_result.turnlight = 3;
            LOG_WARN(" aeb run");
        }else{
            DataPool::getInstance()->escape_topic_data = "null";
        }
        if(DataPool::getInstance()->is_chassis_link_fail && !JudgeIsParking()) 
        {
            clear();
            DataPool::getInstance()->clear_path();
            control_result.turnlight = 3;
            LOG_WARN(" chassis link fail");
        }else
        {
            DataPool::getInstance()->ResetChassisLinkFlag();
        }
        if(1 == DataPool::getInstance()->get_control_status()->solution)
        {
            control_result.speed = 0.0;
            control_result.angle = 0.0;
        }
        LOG_IF(spdlog::level::info,(std::fabs(control_result.speed) > 0.01 || std::fabs(control_result.angle) > 0.01 || std::fabs(control_result.brk_press) > 0.01 || std::fabs(control_result.torque) > 0.01),
                    "control_result.speed : {}, control_result.angle: {}, control_result.brk_press: {}, control_result.torque: {}, control_result.turnlight: {}, relay_control: {}",
                    control_result.speed, control_result.angle, control_result.brk_press, control_result.torque,
                    (int)control_result.turnlight, relay_control.data);
        
        cmd_vel_pub->publish(control_result);
        control_status_pub->publish(*temp_status);
        refer_trajectory_pub->publish(debug_inf.refer_trajectory);
        real_trajectory_pub->publish(debug_inf.real_trajectory);
        
    }

void Workflow::timerCallback_pub()
{
    // nav_msgs::msg::Path test_path;
    // if(!DataPool::getInstance()->get_refer_line().waypoint.empty())
    // {
    //     geometry_msgs::msg::PoseStamped test_point;
    //     test_point.header.frame_id = "map";
    //     test_point.header.stamp = nh->now();
    //     for(int i = 0 ;i < DataPool::getInstance()->get_refer_line().waypoint.size();i++)
    //     {
    //         test_point.pose.position.x = DataPool::getInstance()->get_refer_line().waypoint[i].x;
    //         test_point.pose.position.y = DataPool::getInstance()->get_refer_line().waypoint[i].y;    
    //         test_point.pose.orientation = quaternion_from_yaw(DataPool::getInstance()->get_refer_line().waypoint[i].phi);
            
            
    //         test_path.header.frame_id = "map";
    //         test_path.header.stamp = nh->now();
                    
    //         test_path.poses.emplace_back(test_point);
    //     }
        
        
    // }
    // if(DataPool::getInstance()->flag_new_refer_line)
    // {
    //     favoritism_path_pub->publish(test_path);
    //     DataPool::getInstance()->flag_new_refer_line = false;
    // }
        
}
bool Workflow::JudgeIsTrapped(Position m_current_position)
{
    double time_threshold = 120.0;
    if(fabs(control_result.speed >= 0.1) && fabs(m_current_position.v) < 0.1)
    {
        trapped_cnt += 1; 
    }
    else
    {
        trapped_cnt = 0; 
    }            
    double time_gap = trapped_cnt * DataPool::getInstance()->dt;
    if(time_gap > time_threshold)
    {
        LOG_ERROR("Trapped !!");
        DataPool::getInstance()->get_control_status()->trapped = 1;
        trapped_cnt = 0; 
        return true;
    }
    DataPool::getInstance()->get_control_status()->trapped = 0;
    return false;
}
void Workflow::clear()
{
    JudgeStartBrakeSpeed();
    control_result.speed = SpeedToZero();
    control_result.angle = 0;
    control_result.torque = 0;
    control_result.brk_press = brk_press_check(control_result.speed);
    control_result.torque_rpm_mode = 1; // 速度模式
    m_control_lqr->clear_error();
    m_control_mpc->clear_error();
    m_control_pp->clear_error();
    m_lon_control_ptr->clear_error();
}
double Workflow::SpeedToZero()
{
    auto now_speed = DataPool::getInstance()->get_currect_position().v;
    // LOG_INFO("origin speed: {}", now_speed);
    double speed_commond = 0.0;
    double minus_acc,k_minus_stop_speed;
    if(DataPool::getInstance()->escape_topic_data == "AEB_run")
    {
        minus_acc = 1.0;
        k_minus_stop_speed = 0.2;
    }
    else
    {
        if(1 == DataPool::getInstance()->stop_type)
        {
            minus_acc = DataPool::getInstance()->acc_stop;
            k_minus_stop_speed = DataPool::getInstance()->k_stop_speed;
        }else if(2 == DataPool::getInstance()->stop_type)
        {
            minus_acc =  DataPool::getInstance()->acc_stop_slow;
            k_minus_stop_speed = DataPool::getInstance()->k_stop_speed_slow;
        } 
    }
      

    if(now_speed >= 0.5)
    {
        speed_commond = now_speed - minus_acc;
        speed_commond = std::clamp(speed_commond,0.0,DataPool::getInstance()->get_lon_param().max_v);
    }
    else if(now_speed <= -0.5)
    {
        speed_commond = now_speed + minus_acc;
        speed_commond = std::clamp(speed_commond,DataPool::getInstance()->get_lon_param().max_v_back,0.0);
    }
    else
    {
        // LOG_INFO("k_minus_stop_speed: {}", k_minus_stop_speed);
        speed_commond = now_speed * k_minus_stop_speed;
    }
    
    if(IsSlope() && fabs(speed_commond)<0.14)
    {
        LOG_INFO("slope !!!! pitch: {}", DataPool::getInstance()->get_currect_position().pitch*180 / 3.14159);
        speed_commond = 0.0;
    }

    if(fabs(speed_commond)<0.1)
    {
        speed_commond = 0.0;
    }    
    // LOG_INFO("speed_commond: {}", speed_commond);
    // LOG_INFO("std::floor(speed_commond): {}", std::floor(speed_commond * 10.0) / 10.0);
    return (int(speed_commond * 10.0)) /10.0 ;
}
void DebugInf::clear()
{
    // refer_trajectory.poses.clear();
    // real_trajectory.poses.clear();
}

double  Workflow::brk_press_check(double speed_cmd)
{
    //当发送停车指令时，加入制动压力2MPa，防止遛坡
           
    double brk_press = 0;
    
    if(DataPool::getInstance()->is_chassis_link_fail)
    {
        brk_press = 0.05 * (nh->now() - DataPool::getInstance()->chassis_link_receive_time).seconds()/0.02;
        brk_press = std::clamp(brk_press,0.0,3.0);
    }else
    {
        brk_press = CalcBrkPress(speed_cmd, start_brake_speed);
    }
    if(JudgeIsParking())
    {
        if((nh->now() - park_start_time).seconds() > 7.0)
        {
            brk_press = 0.0;
        }
    }
    
    return brk_press;
}
double Workflow::CalcBrkPress(double speed_cmd, double speed_th)
{
    double brk_press = 0;
    auto now_speed = DataPool::getInstance()->get_currect_position().v;

    if(fabs(now_speed) >= speed_th)
    {
        brk_press = 0.0;
    }
    else if (fabs(now_speed) < speed_th)
    {
        if(fabs(now_speed) < 0.05)
        {
            brk_press = 3.0;
        }else
        {
            brk_press = 0.05 / fabs(now_speed) + speed_th - now_speed;
        }
        if(IsSlope() || DataPool::getInstance()->path_type != 0)
        {
            brk_press += 0.2 + (speed_th - fabs(now_speed));
        }
        brk_press = std::clamp(brk_press,0.0,3.0);
    }

    if(fabs(speed_cmd) < 0.01 && fabs(now_speed) < 0.3)
    {
        auto pitch = fabs(DataPool::getInstance()->get_currect_position().pitch);
        if(IsSlope())
        {
            LOG_INFO("brk_press comp, pitch: {} !", pitch*180 / 3.14159);
            brk_press += std::fabs((pitch * 180 /3.14159 - 2.5) * 1.0);
            brk_press = std::clamp(brk_press,0.0,8.0);
        }       
    }
    return brk_press;
}
bool Workflow::JudgeIsParking()
{
    if(fabs(DataPool::getInstance()->get_odom_data().twist.twist.linear.x) > 0.15)
    {
        park_start_time = nh->now();
        return false;
    }
    else{
        
        if((nh->now().seconds() - park_start_time.seconds()) > 0.5)
        {
            return true;
        }
    }
    return false;
}
igv_msgs::msg::ControlInformation Workflow::CalcAEBCmd(){

    auto odom_data = DataPool::getInstance()->get_odom_data();
    igv_msgs::msg::ControlInformation res;
    res.torque_rpm_mode = 1;
    res.turnlight = 3;
    if(odom_data.twist.twist.linear.x >= 1.5)
    {
        res.speed = 0.0;
        res.angle = odom_data.twist.twist.angular.z;
        res.brk_press = 6.0;
    }
    else if(odom_data.twist.twist.linear.x >= 0.5 && odom_data.twist.twist.linear.x < 1.5)
    {
        res.speed = 0.0;
        res.angle = odom_data.twist.twist.angular.z;
        res.brk_press = 3.0;
    }
    else if(odom_data.twist.twist.linear.x >= 0.0 && odom_data.twist.twist.linear.x < 0.5)
    {
        res.speed = 0.0;
        res.angle = odom_data.twist.twist.angular.z;
        res.brk_press = 1.0;
    }

    return res;
}

bool Workflow::IsSlope()
{
    return (fabs(DataPool::getInstance()->get_currect_position().pitch) > 4.50/180*3.14159);
}

void Workflow::JudgeStartBrakeSpeed()
{
    if(!flag_brk_press)
    {
        auto now_speed = DataPool::getInstance()->get_currect_position().v;
        if(fabs(now_speed) > 1.7)
        {
            start_brake_speed = 1.3;
        }else if(fabs(now_speed) > 1.0 && fabs(now_speed) <= 1.7)
        {
            start_brake_speed = 0.8;
        }else if(fabs(now_speed) <= 1.0)
        {
            start_brake_speed = 0.5;
        }
        flag_brk_press = true;
    }  
}
