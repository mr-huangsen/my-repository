#include "datapool.h"
#include <algorithm>

DataPool* DataPool::instance = nullptr;
std::mutex DataPool::mtx;

DataPool::DataPool(rclcpp::Node::SharedPtr nh):nh(nh){

    control_alg_type = nh->declare_parameter<int>("control_alg_type", 0);
    LOG_INFO("control_alg_type: {}", control_alg_type);
    flag_refer_line_follow_type = nh->declare_parameter<bool>("flag_refer_line_follow_type", false);
    vehicle_param.L = nh->declare_parameter<double>("L", 0.65);
    vehicle_param.Cf = nh->declare_parameter<double>("Cf", -8000.0);
    vehicle_param.Cr = nh->declare_parameter<double>("Cr", -8000.0);
    vehicle_param.m = nh->declare_parameter<double>("m", 150.0);
    vehicle_param.I = nh->declare_parameter<double>("I", 125.0);
    vehicle_param.lr = nh->declare_parameter<double>("lr", 0.32);
    vehicle_param.lf = nh->declare_parameter<double>("lf", 0.33);
    prediction_time = nh->declare_parameter<double>("prediction_time", 0.1);
//lat
    lat_param.min_lookahead_dist = nh->declare_parameter<double>("min_lookahead_dist", 1.45);
    lat_param.max_lookahead_dist = nh->declare_parameter<double>("max_lookahead_dist", 3.5);
    lat_param.manufacturing_accuracy_compensation = nh->declare_parameter<double>("manufacturing_accuracy_compensation", 2.5);
    lat_param.model_compensate = nh->declare_parameter<double>("model_compensate", 2.5);
    lat_param.ki_lat = nh->declare_parameter<double>("ki_lat", 0.02);
    lat_param.ki_phi = nh->declare_parameter<double>("ki_phi", 0.0);
    lat_param.kd_lat = nh->declare_parameter<double>("kd_lat", 0.1);
    lat_param.kd_phi = nh->declare_parameter<double>("kd_phi", 0.5);
    //MPC param
    lat_param.Hp = nh->declare_parameter<int>("Hp", 6);
    lat_param.Hu = nh->declare_parameter<int>("Hu", 6);
    // angular velocity
    lat_param.max_w = nh->declare_parameter<double>("max_w", 0.5);
    lat_param.min_w = nh->declare_parameter<double>("min_w", 0.0);
    lat_param.max_w_delta = nh->declare_parameter<double>("max_w_delta", 0.5);

    // weight matrix trace
    lat_param.Q_matrix_kinematics = nh->declare_parameter<std::vector<double>>("Q_matrix_kinematics", {0.72, 0.22});
    lat_param.R_matrix_kinematics_lat = nh->declare_parameter<std::vector<double>>("R_matrix_kinematics_lat", {10.88});
    lat_param.Q_matrix_dynamical = nh->declare_parameter<std::vector<double>>("Q_matrix_dynamical", {10.0, 1.5, 10.0, 1.5});
    lat_param.R_matrix_dynamical_lat = nh->declare_parameter<std::vector<double>>("R_matrix_dynamical_lat", {100.0});
    lat_param.Q_matrix_kinematics_mpc = nh->declare_parameter<std::vector<double>>("Q_matrix_kinematics_mpc", {0.05, 0.06});
    lat_param.R_matrix_kinematics_lat_mpc = nh->declare_parameter<std::vector<double>>("R_matrix_kinematics_lat_mpc", {6.5});
    lat_param.Q_matrix_dynamical_mpc = nh->declare_parameter<std::vector<double>>("Q_matrix_dynamical_mpc", {0.5, 0.5, 0.5, 0.5});
    lat_param.R_matrix_dynamical_lat_mpc = nh->declare_parameter<std::vector<double>>("R_matrix_dynamical_lat_mpc", {10.0});
    
    // Validate array sizes before the controllers index matrix weights.
    if (lat_param.Q_matrix_kinematics.size() != 2) throw std::invalid_argument("Q_matrix_kinematics must contain 2 weights");
    if (lat_param.Q_matrix_dynamical.size() != 4) throw std::invalid_argument("Q_matrix_dynamical must contain 4 weights");
    if (lat_param.Q_matrix_kinematics_mpc.size() != 2) throw std::invalid_argument("Q_matrix_kinematics_mpc must contain 2 weights");
    if (lat_param.Q_matrix_dynamical_mpc.size() != 4) throw std::invalid_argument("Q_matrix_dynamical_mpc must contain 4 weights");
    if (lat_param.R_matrix_kinematics_lat.size() != 1) throw std::invalid_argument("R_matrix_kinematics_lat must contain 1 weights");
    if (lat_param.R_matrix_dynamical_lat.size() != 1) throw std::invalid_argument("R_matrix_dynamical_lat must contain 1 weights");
    if (lat_param.R_matrix_kinematics_lat_mpc.size() != 1) throw std::invalid_argument("R_matrix_kinematics_lat_mpc must contain 1 weights");
    if (lat_param.R_matrix_dynamical_lat_mpc.size() != 1) throw std::invalid_argument("R_matrix_dynamical_lat_mpc must contain 1 weights");

    // iteration 
    lat_param.max_iter = nh->declare_parameter<int>("max_iter", 500);
    lat_param.eps_iter = nh->declare_parameter<double>("eps_iter", 1e-4);
    lat_param.max_kappa = nh->declare_parameter<double>("max_kappa", 0.4);
    lat_param.max_diff_kappa = nh->declare_parameter<double>("max_diff_kappa", 0.1);
    lat_param.max_diff_phi = nh->declare_parameter<double>("max_diff_phi", 0.1);
    lat_param.max_dist_between_two_points = nh->declare_parameter<double>("max_dist_between_two_points", 0.2);
//lon
    // linear velocity
    lon_param.max_v = nh->declare_parameter<double>("max_v", 1.5);
    lon_param.max_v_back = nh->declare_parameter<double>("max_v_back", -0.6);
    lon_param.min_v = nh->declare_parameter<double>("min_v", 0.0);
    lon_param.max_v_delta = nh->declare_parameter<double>("max_v_delta", 0.5);

    //lon_control param
    lon_param.kp_lon = nh->declare_parameter<double>("kp_lon", 1.0);
    lon_param.ki_lon = nh->declare_parameter<double>("ki_lon", 0.01);
    lon_param.kd_lon = nh->declare_parameter<double>("kd_lon", 0.05);
    lon_param.torque_rpm_mode = nh->declare_parameter<int>("torque_rpm_mode", 1);
    
    lon_param.k_pressure = nh->declare_parameter<double>("k_pressure", 100.0);
    lon_param.kp_press = nh->declare_parameter<double>("kp_press", 10.0);
    lon_param.k_torque = nh->declare_parameter<double>("k_torque", 4.5);
    lon_param.kp_t_p = nh->declare_parameter<double>("kp_t_p", 1.0);
    lon_param.ki_t_p = nh->declare_parameter<double>("ki_t_p", 0.1);
    lon_param.kd_t_p = nh->declare_parameter<double>("kd_t_p", 0.1);
    lon_param.kp_t_s = nh->declare_parameter<double>("kp_t_s", 5.0);
    lon_param.ki_t_s = nh->declare_parameter<double>("ki_t_s", 0.1);
    lon_param.kd_t_s = nh->declare_parameter<double>("kd_t_s", 0.1);
    bool temp_bool_value;
    temp_bool_value = nh->declare_parameter<bool>("is_use_adaptive_pid", false);
    lon_param.is_use_adaptive_pid = temp_bool_value;
    temp_bool_value = nh->declare_parameter<bool>("slope_mode", false);
    lon_param.slope_mode = temp_bool_value;  
    lon_param.k_slope_compensate = nh->declare_parameter<double>("k_slope_compensate", 0.028);
    
    //停车逻辑    
    stop_type = 2;
    k_stop_speed = nh->declare_parameter<double>("k_stop_speed", 0.6);
    acc_stop = nh->declare_parameter<double>("acc_stop", 0.2);
    k_stop_speed_slow = nh->declare_parameter<double>("k_stop_speed_slow", 0.8);
    acc_stop_slow = nh->declare_parameter<double>("acc_stop_slow", 0.1);
    LOG_INFO("k_stop_speed: {}, acc_stop: {}, k_stop_speed_slow: {}, acc_stop_slow: {}",
              k_stop_speed, acc_stop, k_stop_speed_slow, acc_stop_slow);
    //
    control_status.path_check = 0;
    control_status.trapped = 0;
    control_status.location_warp = 0;
    control_status.solution = 0;
    control_status.deviation = 0;
    control_status.prediction_check = 0;    
    location_receive_time = turnlight_command_receive_time = aeb_receive_time =
        path_receive_time = chassis_link_receive_time = calibration_cmd_receive_time =
        last_chassis_status_receive_time = rclcpp::Time(0, 0, nh->get_clock()->get_clock_type());
    dt = 1 / 50.0;
    flag_new_refer_line = false;
    is_chassis_link_fail = false;
    
    location_sub = nh->create_subscription<igv_msgs::msg::Location>("/localization", 10, std::bind(&DataPool::receive_location_data, this, std::placeholders::_1));
    local_path_srv = nh->create_service<igv_msgs::srv::LocalPath>("/local_path",
        [this](const std::shared_ptr<igv_msgs::srv::LocalPath::Request> req,
               std::shared_ptr<igv_msgs::srv::LocalPath::Response> res) {
            handle_local_path(*req, *res);
        });
    dispatch_sub = nh->create_subscription<std_msgs::msg::Bool>("/quick_brake", 10, std::bind(&DataPool::receive_dispatch_data, this, std::placeholders::_1));
    chassis_status_sub = nh->create_subscription<igv_msgs::msg::ICCPStatus>("/iccp_status", 10, std::bind(&DataPool::receive_chassis_status, this, std::placeholders::_1));
    turnlight_sub = nh->create_subscription<std_msgs::msg::Int8>("/turnlight_command", 10, std::bind(&DataPool::receive_turnlight_command, this, std::placeholders::_1));
    escape_sub = nh->create_subscription<std_msgs::msg::String>("/decision_to_escape", 5, std::bind(&DataPool::receive_escape_topic_data, this, std::placeholders::_1));
    chassis_link_sub = nh->create_subscription<std_msgs::msg::Bool>("/chassis_link", 10, std::bind(&DataPool::receive_chassis_link_data, this, std::placeholders::_1));
    calibration_cmd_sub = nh->create_subscription<igv_msgs::msg::ControlInformation>(
        "/calibration_cmd", 10, std::bind(&DataPool::receive_calibration_cmd, this, std::placeholders::_1));
}
DataPool::~DataPool()
{
}
void DataPool::receive_calibration_cmd(const igv_msgs::msg::ControlInformation::ConstSharedPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutexcalibrationcmd);
    calibration_cmd = *msg;
    calibration_cmd_receive_time = nh->now();
    is_calibration_mode = true;
    LOG_EVERY_N(spdlog::level::info, 50, "get calibration cmd: speed: {}, brake:{}", msg->speed, msg->brk_press);
}
void DataPool::receive_escape_topic_data(const std_msgs::msg::String::ConstSharedPtr& msg)
{
    std::unique_lock<std::mutex> lock(mutexescape);
    escape_topic_data = msg->data;
    aeb_receive_time = nh->now();
    LOG_INFO("escape_topic_data: {}", msg->data);
}
void DataPool::receive_turnlight_command(const std_msgs::msg::Int8::ConstSharedPtr& msg)
{
    std::unique_lock<std::mutex> lock(mutexturnlightcommand);
    turnligth_command_by_decision = msg->data;
    receive_decision_turnlight_cmd = true;   
    turnlight_command_receive_time = nh->now();
    LOG_INFO("get decision turnlight cmd: {}", (int)msg->data);

}
void DataPool::receive_chassis_status(const igv_msgs::msg::ICCPStatus::ConstSharedPtr& msg)
{
    std::scoped_lock lock(mutexiccpdata, mutexodomdata, mutexlocationpoint);
    chassis_status = *msg;
    odom_data.twist.twist.linear.x = msg->current_speed;
    odom_data.twist.twist.angular.z = msg->current_angle * 3.14159 / 180.0;
    currect_position.steer = odom_data.twist.twist.angular.z;
    currect_position.v_last = currect_position.v;
    currect_position.v = msg->current_speed;
    currect_position.dot_phi = currect_position.v * std::tan(currect_position.steer) / vehicle_param.L;

    const auto now = nh->now();
    const double elapsed = (now - last_chassis_status_receive_time).seconds();
    currect_position.a = 0.0;
    if (last_chassis_status_receive_time.nanoseconds() != 0 && elapsed > 0.0) {
        currect_position.a = std::clamp((currect_position.v - currect_position.v_last) / elapsed, -3.0, 3.0);
    }
    last_chassis_status_receive_time = now;
}
void DataPool::receive_dispatch_data(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    if(msg->data)
    {
        local_trajectory.waypoint.clear();
        stop_type = 1;
        LOG_INFO("clear path by dispatch !!!");
    }
}
void DataPool::receive_chassis_link_data(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    if(!is_chassis_link_fail)
    {
        chassis_link_receive_time = nh->now();
        is_chassis_link_fail = true;
        local_trajectory.waypoint.clear();
        LOG_INFO("clear path by chassis !!! chassis link faliure");
    }
    else
    {
        if(msg->data)
        {
            local_trajectory.waypoint.clear();
            LOG_INFO("clear path by chassis !!! chassis link faliure");
        }
    }
}
void DataPool::receive_location_data(const igv_msgs::msg::Location::ConstSharedPtr& msg)
{
    std::unique_lock<std::mutex> lock(mutexlocationpoint);      
    if(check_location(msg))
    {
        // LOG_INFO("location receive success");
        currect_position.x = msg->car_pose.position.x;
        currect_position.y = msg->car_pose.position.y;        

        tf2::Quaternion quat;
        tf2::fromMsg(msg->car_pose.orientation, quat);
        double roll, pitch, yaw;//定义存储roll,pitch and yaw的容器
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); //进行转换
        currect_position.roll = roll;
        currect_position.pitch = pitch;
        currect_position.phi = yaw;

        auto qq = msg->car_pose.orientation;
        Eigen::Quaterniond q(qq.w, qq.x, qq.y, qq.z);
        Eigen::Matrix3d temp = q.toRotationMatrix();
        Eigen::Vector3d vel(msg->vel.x, msg->vel.y, msg->vel.z);
        auto local_v = temp.inverse() * vel;
        // currect_position.v = local_v.x();
        // currect_position.dot_phi = msg->gyr.z;
        currect_position.status = msg->status;
        location_receive_time = nh->now();
        // LOG_INFO("location receive success, x: {}, y: {}, yaw: {}, speed: {}",
        //           currect_position.x, currect_position.y, currect_position.phi, local_v.x());
    }
    else
    {
        currect_position.status = msg->status; 
        LOG_ERROR("location fail!!!currect_position.status: {}", currect_position.status);
    }    
}

bool DataPool::handle_local_path(igv_msgs::srv::LocalPath::Request &req,
    igv_msgs::srv::LocalPath::Response &res)
{
    std::unique_lock<std::mutex> lock(mutexlocalpath);  
    if (req.clear_path == 0) {
        const auto n = req.x.size();
        if (n < 2 || req.y.size() != n || req.phi.size() != n || req.v.size() != n ||
            req.a.size() != n || req.kappa.size() != n || req.timestamp.size() != n ||
            req.vaild_path_index < 0 || static_cast<size_t>(req.vaild_path_index) >= n) {
            control_status.path_check = 4;
            res.res = 0;
            return false;
        }
    }
    igv_msgs::srv::LocalPath::Request temp_local_path = req;
    if(0 == temp_local_path.path_type)
    {        
        LOG_INFO("plan, forward!");
    }
    else if(1 == temp_local_path.path_type)
    {
        LOG_INFO("escape, back!is_stop_turn_wheel: {}", temp_local_path.is_stop_turn_wheel);
    }
    else if(2 == temp_local_path.path_type)
    {
        LOG_INFO("escape, forward!is_stop_turn_wheel: {}", temp_local_path.is_stop_turn_wheel);
    }
    is_stop_turn_wheel = temp_local_path.is_stop_turn_wheel;
    path_type = temp_local_path.path_type;
    vaild_path_index = temp_local_path.vaild_path_index;
    if(0 == temp_local_path.clear_path )
    {
        LOG_INFO("receive path, the point of path is: {}", temp_local_path.x.size());
        for(int i = 0;i<temp_local_path.x.size();i++)
        {
            LOG_INFO("local_path.index: {}, local_path.x: {}, local_path.y: {}, local_path.phi: {}, "
                      "local_path.kappa: {}, local_path.v: {}, local_path.a: {}, local_path.timestamp: {}",
                      i, temp_local_path.x[i], temp_local_path.y[i], temp_local_path.phi[i],
                      temp_local_path.kappa[i], temp_local_path.v[i], temp_local_path.a[i],
                      temp_local_path.timestamp[i]);
        }
        local_trajectory.waypoint.clear();
        if(check_path(temp_local_path))
        {
            Position temp_position;
            for(int i = 0;i<temp_local_path.x.size();i++)
            {
                temp_position.timestamp = temp_local_path.timestamp[i];
                temp_position.x = temp_local_path.x[i];
                temp_position.y = temp_local_path.y[i];
                temp_position.phi = temp_local_path.phi[i];
                temp_position.v = temp_local_path.v[i];
                temp_position.a = temp_local_path.a[i];
                temp_position.kappa = temp_local_path.kappa[i];
                local_trajectory.waypoint.push_back(temp_position);
            }
            
        }
        else
        {
            LOG_ERROR("check path is not pass!!!!");
        }
    }else if(1 == temp_local_path.clear_path)
    {
        stop_type = temp_local_path.clear_path;
        local_trajectory.waypoint.clear();
        LOG_ERROR("clear path over, EMERGENCY STOP !!!");
    
    }else if (2 == temp_local_path.clear_path)
    {
        stop_type = temp_local_path.clear_path;
        local_trajectory.waypoint.clear();
        LOG_WARN("clear path, by clear path commond!!!");
    }
    path_receive_time = nh->now();
    res.res = 1;//接收成功
    return true;
}
bool DataPool::check_path(igv_msgs::srv::LocalPath::Request temp_local_path)
{    
    int cnt = 0;
    if((nh->now().seconds() - temp_local_path.timestamp[0]/1.0/1e6) > 1)
    {
        control_status.path_check = 1;
        LOG_ERROR("local path time is serious lag!!!time gap: {}",
                   (nh->now().seconds() - temp_local_path.timestamp[0]/1.0/1e6));
        // return false;
        return true;
    }
    for(auto kappa:temp_local_path.kappa)
    {
        if(kappa > lat_param.max_kappa)
        {
            cnt = cnt + 1;
            LOG_INFO("local path kappa: {}", kappa);
        }
    }
    if(0 != cnt)
    {
        control_status.path_check = 2;
        LOG_ERROR("the number of local path which kappa is over maxkappa : {}, maxkappa: {}",
                   cnt, lat_param.max_kappa);
        // return false;
        return true;
    }
    for(int i = 0;i<temp_local_path.kappa.size()-1;i++)
    {
        double diff_kappa = temp_local_path.kappa[i+1] - temp_local_path.kappa[i];
        double diff_phi = norm_angle(temp_local_path.phi[i+1] - temp_local_path.phi[i]);
        double diff_dist = std::hypot(temp_local_path.x[i+1] - temp_local_path.x[i],temp_local_path.y[i+1] - temp_local_path.y[i]);
        if((diff_kappa > lat_param.max_diff_kappa) || (diff_phi > lat_param.max_diff_phi) || diff_dist > lat_param.max_dist_between_two_points)
        {
            control_status.path_check = 3;
            LOG_ERROR("the phi and kappa of local path change too fast!!diff_kappa:  {}, diff_phi: {}, "
                       "diff_dist{}, point index: {}",
                       diff_kappa, diff_phi, diff_dist, i);
            // return false;
            return true;
        }
    }
    if((temp_local_path.x.size() != temp_local_path.y.size()) || (temp_local_path.x.size() != temp_local_path.phi.size()) ||
       (temp_local_path.x.size() != temp_local_path.phi.size()) || (temp_local_path.x.size() != temp_local_path.v.size()) ||
       (temp_local_path.x.size() != temp_local_path.a.size()) || (temp_local_path.x.size() != temp_local_path.a.size()) ||
       (temp_local_path.x.size() != temp_local_path.kappa.size()))
    {
            LOG_ERROR("the size is not inconsistent:  {}{}{}{}{}{}{}",
                       temp_local_path.x.size(), temp_local_path.y.size(), temp_local_path.kappa.size(),
                       temp_local_path.phi.size(), temp_local_path.v.size(), temp_local_path.a.size(),
                       temp_local_path.kappa.size());
            control_status.path_check = 4;          
            // return false;
            return true;
    }
    for(int i = 0;i<temp_local_path.x.size();i++)
    {
        if(isnan(temp_local_path.timestamp[i])||isinf(temp_local_path.timestamp[i]) || 
            isnan(temp_local_path.x[i])||isinf(temp_local_path.x[i]) ||
            isnan(temp_local_path.y[i])||isinf(temp_local_path.y[i]) ||
            isnan(temp_local_path.phi[i])||isinf(temp_local_path.phi[i]) ||
            isnan(temp_local_path.v[i])||isinf(temp_local_path.v[i]) ||
            isnan(temp_local_path.a[i])||isinf(temp_local_path.a[i]) ||
            isnan(temp_local_path.kappa[i])||isinf(temp_local_path.kappa[i]))
        {
            control_status.path_check = 5;
            LOG_ERROR("path exist nan or inf !!!");
            return true;
        }    

        
    }
    control_status.path_check = 0;   
    return true;

}
bool DataPool::check_location(std::shared_ptr<const igv_msgs::msg::Location> temp_location_point)
{    
    if(!check_location_status(temp_location_point))
    {
        control_status.location_warp = 3;
        
        return false;
    }
    if((nh->now().seconds() - temp_location_point->timestamp /1.0 / 1e6) > 1.0)
    {
        control_status.location_warp = 1;
        LOG_ERROR("location is failure!!!time gap: {}",
                   (temp_location_point->timestamp /1.0 / 1e6 - nh->now().seconds()));
        return false;
    }

    auto qq = temp_location_point->car_pose.orientation;
    Eigen::Quaterniond q(qq.w, qq.x, qq.y, qq.z);
    Eigen::Matrix3d temp = q.toRotationMatrix();
    Eigen::Vector3d vel(temp_location_point->vel.x, temp_location_point->vel.y, temp_location_point->vel.z);
    auto local_v = temp.inverse() * vel;
    double c_spd =  sqrt(local_v.x()* local_v.x() + local_v.y()*local_v.y() + local_v.z()*local_v.z());
    
    if(fabs(c_spd - fabs(odom_data.twist.twist.linear.x)) > 0.3)
    {
        control_status.location_warp = 0;
        // LOG_ERROR("local_speed: {}, odom_speed: {}", c_spd, odom_data.twist.twist.linear.x);
        // LOG_ERROR("There is a significant difference between location data and odom data!!!");
        // return false;
    }    
    control_status.location_warp = 0;
    return true;
}
bool DataPool::check_location_status(std::shared_ptr<const igv_msgs::msg::Location> temp_location_point)
{
    if(2 == temp_location_point->status || 8 == temp_location_point->status || 9 == temp_location_point->status)
    {
        LOG_ERROR("location is failure!!!status: {}", temp_location_point->status);
        return false;
    }
    if(3 == temp_location_point->status || 4 == temp_location_point->status ||
        5 == temp_location_point->status || 6 == temp_location_point->status ||
        7 == temp_location_point->status)
    {
        // LOG_WARN("location is unstable, status: {}", temp_location_point->status);
    }
    return true;
}
void DataPool::clear_path()
{
    std::unique_lock<std::mutex> lock(mutexlocalpath);
    local_trajectory.waypoint.clear();
    LOG_WARN("clear path ! by control node!");
}
void DataPool::ResetChassisLinkFlag()
{
    is_chassis_link_fail = false;
}
double DataPool::norm_angle(double angle)
{
    angle = std::fmod (angle,2*M_PI);
    angle = angle > M_PI ? (angle - 2*M_PI):angle;
    angle = angle < -M_PI ? (angle + 2*M_PI):angle;
    return angle;
}
LateralParam DataPool::get_lat_param(){
    std::unique_lock<std::mutex> lock(mutexgetlatparam);  
    return lat_param;
}
LongitudinalParam DataPool::get_lon_param(){
    std::unique_lock<std::mutex> lock(mutexgetlonparam);
    return lon_param;
}
VehicleParam DataPool::get_vehicle_param(){
    std::unique_lock<std::mutex> lock(mutexgetvehicleparam);
    return vehicle_param;
}

Position DataPool::get_currect_position(){
    std::unique_lock<std::mutex> lock(mutexlocationpoint);
    return currect_position;
}
LocalTrajectory DataPool::get_local_trajectory(){
    std::unique_lock<std::mutex> lock(mutexlocalpath);
    return local_trajectory;
}
nav_msgs::msg::Odometry DataPool::get_odom_data(){
    std::unique_lock<std::mutex> lock(mutexodomdata);
    return odom_data;
}
ControlStatus* DataPool::get_control_status(){
    std::unique_lock<std::mutex> lock(mutexgetcontrolstatus);
    return &control_status;
}

igv_msgs::msg::ControlInformation DataPool::get_calibration_cmd()
{
    std::lock_guard<std::mutex> lock(mutexcalibrationcmd);
    return calibration_cmd;
}
