#include "datapool.h"
DataPool* DataPool::instance = nullptr;
std::mutex DataPool::mtx;

DataPool::DataPool(rclcpp::Node::SharedPtr nh):nh(nh){
   
    plan_type = nh->declare_parameter<int>("plan_type", 1);
    L = nh->declare_parameter<double>("L", 1.45);
    currect_position.l = nh->declare_parameter<double>("l", 2.0);
    currect_position.w = nh->declare_parameter<double>("w", 1.0);
    currect_position.h = nh->declare_parameter<double>("h", 1.8);
    distance_forward = nh->declare_parameter<double>("distance_forward", 30.0);
    distance_back = nh->declare_parameter<double>("distance_back", 15.0);
    sample_point_number = nh->declare_parameter<int>("sample_point_number", 150);
    control_cycle = nh->declare_parameter<double>("control_cycle", 0.02);
    //lattic
    lattic_config.sample_type = nh->declare_parameter<int>("sample_type", 0);
    lattic_config.MAX_SPEED = nh->declare_parameter<double>("MAX_SPEED", 12.5);
    lattic_config.MAX_SPEED_LAT = nh->declare_parameter<double>("MAX_SPEED_LAT", 100.0);
    lattic_config.MAX_ACCEL = nh->declare_parameter<double>("MAX_ACCEL", 10.0);
    lattic_config.MAX_CURVATURE = nh->declare_parameter<double>("MAX_CURVATURE", 0.4);
    lattic_config.MAX_ROAD_WIDTH = nh->declare_parameter<double>("MAX_ROAD_WIDTH", 5.0);
    lattic_config.D_ROAD_W = nh->declare_parameter<double>("D_ROAD_W", 1.0);
    lattic_config.DT = nh->declare_parameter<double>("DT", 0.1);
    lattic_config.MAXT = nh->declare_parameter<double>("MAXT", 15.0);
    lattic_config.MINT = nh->declare_parameter<double>("MINT", 1.0);
    lattic_config.DTIME = nh->declare_parameter<double>("DTIME", 1.0);
    lattic_config.TARGET_SPEED = nh->declare_parameter<double>("TARGET_SPEED", 1.5);
    lattic_config.D_T_S = nh->declare_parameter<double>("D_T_S", 0.3);
    lattic_config.SMALL_SPEED = nh->declare_parameter<double>("SMALL_SPEED", 0.3);
    lattic_config.DS = nh->declare_parameter<double>("DS", 5.0);
    lattic_config.MAXS = nh->declare_parameter<double>("MAXS", 15.0);
    lattic_config.MINS = nh->declare_parameter<double>("MINS", 5.0);
    lattic_config.KJ = nh->declare_parameter<double>("KJ", 0.1);
    lattic_config.KT = nh->declare_parameter<double>("KT", 0.1);
    lattic_config.KJD = nh->declare_parameter<double>("KJD", 2.0);
    lattic_config.KD = nh->declare_parameter<double>("KD", 20.0);
    lattic_config.KLAT = nh->declare_parameter<double>("KLAT", 1.0);
    lattic_config.KLON = nh->declare_parameter<double>("KLON", 1.0);
    lattic_config.KCAC = nh->declare_parameter<double>("KCAC", 0.0);
    lattic_config.KEFF = nh->declare_parameter<double>("KEFF", 0.0);

    //ilqr
    cilqr_config.dt = nh->declare_parameter<double>("delta_t", 0.1);
    cilqr_config.N = nh->declare_parameter<int>("N", 30);
    cilqr_config.nx = nh->declare_parameter<int>("nx", 4);
    cilqr_config.nu = nh->declare_parameter<int>("nu", 2);
    cilqr_config.w_pos = nh->declare_parameter<double>("w_pos", 1.0);
    cilqr_config.w_vel = nh->declare_parameter<double>("w_vel", 1.0);
    cilqr_config.w_yaw = nh->declare_parameter<double>("w_yaw", 20.0);
    cilqr_config.w_acc = nh->declare_parameter<double>("w_acc", 0.5);
    cilqr_config.w_stl = nh->declare_parameter<double>("w_stl", 20.0);
    cilqr_config.use_last_solution = nh->declare_parameter<bool>("use_last_solution", false);
    cilqr_config.solve_type_str = nh->declare_parameter<std::string>("solve_type", "barrier");
    cilqr_config.obstacle_exp_q1 = nh->declare_parameter<double>("obstacle_exp_q1", 5.5);
    cilqr_config.obstacle_exp_q2 = nh->declare_parameter<double>("obstacle_exp_q2", 5.75);
    cilqr_config.state_exp_q1 = nh->declare_parameter<double>("state_exp_q1", 3.0);
    cilqr_config.state_exp_q2 = nh->declare_parameter<double>("state_exp_q2", 3.5);
    cilqr_config.alm_rho_init = nh->declare_parameter<double>("alm_rho_init", 20.0);
    cilqr_config.alm_gamma = nh->declare_parameter<double>("alm_gamma", 0.0);
    cilqr_config.max_rho = nh->declare_parameter<double>("max_rho", 20.0);
    cilqr_config.max_mu = nh->declare_parameter<double>("max_mu", 120.0);
    

    cilqr_config.max_iter = nh->declare_parameter<int>("max_iter", 100);
    cilqr_config.init_lamb = nh->declare_parameter<double>("init_lamb", 0.0);
    cilqr_config.lamb_decay = nh->declare_parameter<double>("lamb_decay", 0.5);
    cilqr_config.lamb_amplify = nh->declare_parameter<double>("lamb_amplify", 2.0);
    cilqr_config.max_lamb = nh->declare_parameter<double>("max_lamb", 1000.0);
    cilqr_config.convergence_threshold = nh->declare_parameter<double>("convergence_threshold", 0.01);
    cilqr_config.accept_step_threshold = nh->declare_parameter<double>("accept_step_threshold", 0.5);

    cilqr_config.wheelbase = nh->declare_parameter<double>("wheelbase", 1.198);
    cilqr_config.width = nh->declare_parameter<double>("width", 1.8);
    cilqr_config.length = nh->declare_parameter<double>("length", 2.5);
    cilqr_config.velo_max = nh->declare_parameter<double>("velo_max", 1.5);
    cilqr_config.velo_min = nh->declare_parameter<double>("velo_min", 0.0);
    cilqr_config.yaw_lim = nh->declare_parameter<double>("yaw_lim", 1.57);
    cilqr_config.acc_max = nh->declare_parameter<double>("acc_max", 3.0);
    cilqr_config.acc_min = nh->declare_parameter<double>("acc_min", -3.0);
    cilqr_config.stl_lim = nh->declare_parameter<double>("stl_lim", 0.4);
    cilqr_config.d_safe = nh->declare_parameter<double>("d_safe", 1.0);
    cilqr_config.reference_point = nh->declare_parameter<std::string>("reference_point", "rear_center");
    cilqr_config.target_velocity = nh->declare_parameter<double>("target_velocity", 1.5);

    run_dt = 0.1;
    currect_position.x = 0.0;
    currect_position.y = 0.0;   
    currect_position.z = 0.0;   
    currect_position.yaw = 0.0;
    currect_position.v = 0.0;    
    currect_position.dot_yaw = 0.0;  
    flag_is_new_task = true;   
    odom_sub = nh->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&DataPool::receive_odom_data, this, std::placeholders::_1));
    location_sub = nh->create_subscription<igv_msgs::msg::Location>("/localization", 10, std::bind(&DataPool::receive_location_data, this, std::placeholders::_1));
    occupancy_grid_sub = nh->create_subscription<nav_msgs::msg::OccupancyGrid>("/freespace/local_costmap", 10, std::bind(&DataPool::receive_occupancy_grid, this, std::placeholders::_1));
    ref_line_sub = nh->create_subscription<nav_msgs::msg::Path>("/planning_component/niit_routing_reference_line", 10, std::bind(&DataPool::receive_ref_line, this, std::placeholders::_1));
    left_bound_sub = nh->create_subscription<nav_msgs::msg::Path>("/planning_component/left_bound", 10, std::bind(&DataPool::receive_left_bound, this, std::placeholders::_1));
    right_bound_sub = nh->create_subscription<nav_msgs::msg::Path>("/planning_component/right_bound", 10, std::bind(&DataPool::receive_right_bound, this, std::placeholders::_1));
}
void DataPool::receive_odom_data(const nav_msgs::msg::Odometry::SharedPtr msg)
{    
    std::unique_lock<std::mutex> lock(mutexodomdata);  
    
    // tf::Matrix3x3 rotationMatrix;    
    // try
    // {
    //     listener_baselinktomap.lookupTransform("/map", "/base_link", ros::Time(0), transformStamped);
    //     tf::Quaternion tfQuaternion(transformStamped.getRotation().x(), transformStamped.getRotation().y(), transformStamped.getRotation().z(), transformStamped.getRotation().w());
    //     rotationMatrix.setRotation(tfQuaternion);
    // }
    // catch (tf::TransformException& ex) {
    // }
    
    double temp_v =  msg->twist.twist.linear.x;
    double temp_delta = msg->twist.twist.angular.z *3.14159/180;   
    double temp_dot_yaw = temp_v  * tan(temp_delta) / L ;
    double comp_angle = chassis_bias_angle * 3.14159 / 180;
    double dot_yaw_by_compangle = temp_v  * tan(comp_angle) / L ;
    currect_position.dot_yaw = temp_dot_yaw + dot_yaw_by_compangle;
    currect_position.v = temp_v;
}
void DataPool::receive_left_bound(const nav_msgs::msg::Path::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(mutexleftbound);    
    // left_bound.clear();
    if(!msg->poses.empty()&& left_bound.empty()){
        Position temp_point;
        for(auto point:msg->poses)
        {
            temp_point.x = point.pose.position.x;
            temp_point.y = point.pose.position.y;            
            left_bound.emplace_back(temp_point);
        }
        LOG_INFO("receive left bound!!!!");
    }

}
void DataPool::receive_right_bound(const nav_msgs::msg::Path::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(mutexrightbound);    
    // right_bound.clear();
    if(!msg->poses.empty() && right_bound.empty()){
        Position temp_point;
        for(auto point:msg->poses)
        {
            temp_point.x = point.pose.position.x;
            temp_point.y = point.pose.position.y;            
            right_bound.emplace_back(temp_point);
        }
        LOG_INFO("receive right bound!!!!");
    }
}


void DataPool::receive_ref_line(const nav_msgs::msg::Path::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(mutexrefline);      
    // ref_line.clear();
    // flag_is_new_task = true;  
    Position t_point;
    if(!msg->poses.empty() && ref_line.empty()){   
        flag_is_new_task = true;  
        auto fist_point = msg->poses.front();
        auto secend_point = msg->poses[1];
        double k_theta = atan2(secend_point.pose.position.y - fist_point.pose.position.y, secend_point.pose.position.x - fist_point.pose.position.x);
        double dd = std::hypot(secend_point.pose.position.x - fist_point.pose.position.x, secend_point.pose.position.y - fist_point.pose.position.y);
        t_point.x = fist_point.pose.position.x - dd * cos(k_theta);
        t_point.y = fist_point.pose.position.y - dd * sin(k_theta);
        ref_line.emplace_back(t_point);
        for(auto point:msg->poses)
        {            
            t_point.x = point.pose.position.x;
            t_point.y = point.pose.position.y;        
            
            ref_line.emplace_back(t_point);
            
              
        }
        LOG_INFO("receive ref line!!!!");  
    }    
}

void DataPool::receive_occupancy_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(mutexoccupancygrid);
    occupancy_grid.data.clear();
    occupancy_grid.header =  msg->header;
    occupancy_grid.info = msg->info;
    occupancy_grid.data = msg->data;    
}

void DataPool::receive_location_data(const igv_msgs::msg::Location::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(mutexlocationpoint);      
    if(check_location(msg))
    {        
        currect_position.x = msg->car_pose.position.x;
        currect_position.y = msg->car_pose.position.y;   
        currect_position.z = msg->car_pose.position.z;   
        currect_position.yaw = tf2::getYaw(msg->car_pose.orientation);
        currect_position.vx = msg->vel.x;
        currect_position.vy = msg->vel.y;
        currect_position.vz = msg->vel.z;
        // currect_position.dot_yaw = msg->gyr.z;     
        currect_position.a = 0.0;  
        location_receive_time = nh->now();
    }
    else
    {        
        LOG_ERROR("location fail!!!");
    }

}

bool DataPool::check_location(const igv_msgs::msg::Location::SharedPtr temp_location_point)
{    
    if(!check_location_status(temp_location_point))
    {
        prediction_status.location_warp = 3;
        
        return false;
    }
    if((nh->now().seconds() - temp_location_point->timestamp /1.0 / 1e6) > 0.2)
    {
        prediction_status.location_warp = 1;
        LOG_ERROR("location is failure!!!time gap: {}",
            (nh->now().seconds() - temp_location_point->timestamp /1.0 / 1e6));
        return false;
    } 
    prediction_status.location_warp = 0;
    return true;
}
bool DataPool::check_location_status(const igv_msgs::msg::Location::SharedPtr temp_location_point)
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
    }
    return true;
}
FrenetPathPoint DataPool::get_currect_position(){
    std::unique_lock<std::mutex> lock(mutexlocationpoint);
    return currect_position;
}
nav_msgs::msg::OccupancyGrid DataPool::get_occupancy_grid(){
    std::unique_lock<std::mutex> lock(mutexoccupancygrid);
    return occupancy_grid;
}

ObstacleGroup DataPool::get_obstacle_group(){
    std::unique_lock<std::mutex> lock(mutexobstacle); 
    return obstacle_group;
}
LocalTrajectory DataPool::get_ref_line()
{
    std::unique_lock<std::mutex> lock(mutexrefline);
    return ref_line;
}
std::vector<FrenetPathPoint> DataPool::get_left_bound(){
    std::unique_lock<std::mutex> lock(mutexleftbound);
    return left_bound;
}
std::vector<FrenetPathPoint> DataPool::get_right_bound(){
    std::unique_lock<std::mutex> lock(mutexrightbound);
    return right_bound;
}
LatticConfig DataPool::get_lattic_config(){
    std::unique_lock<std::mutex> lock(mutexlatticconfig);
    return lattic_config;
}
CIlqrConfig DataPool::get_ilqr_config(){
    std::unique_lock<std::mutex> lock(mutexilqrconfig);
    return cilqr_config;
;
}
