#include "control/loncontrol.h"
#include "control/torque_network.h"

#include <algorithm>

LonControl::LonControl(){
        
    auto cp = DataPool::getInstance();
    m_lon_param = cp->get_lon_param();
    dt = cp->dt;
}
// LonControl::~LonControl(){}

igv_msgs::msg::ControlInformation LonControl::lon_calculate(const Position &current_car_status, const Position &match_car_status)
{    
    error_calculate(current_car_status,match_car_status);    
    auto control_result = lon_contrl_calculate(current_car_status,match_car_status);
    control_result.torque_rpm_mode = m_lon_param.torque_rpm_mode;
    return control_result;    
}

void LonControl::error_calculate(Position current_point, Position match_point)
{
    double vx = current_point.v;
    double vy = 0;
    Eigen::Vector2d v_tou;
    v_tou << cos(match_point.phi),sin(match_point.phi);
    Eigen::Vector2d v_n;
    v_n << -sin(match_point.phi),cos(match_point.phi);
    Eigen::Vector2d temp_v;
    temp_v << current_point.x - match_point.x, current_point.y - match_point.y;    
    car_error_.lonerror.error_s = v_tou.dot(temp_v);
    car_error_.laterror.error_d = v_n.dot(temp_v);
    car_error_.kappar = match_point.kappa;
    double theta_r = match_point.phi + car_error_.lonerror.error_s * car_error_.kappar;
    car_error_.laterror.error_phi = norm_angle(current_point.phi - theta_r);    
    double dot_s = (vx * cos(car_error_.laterror.error_phi) - vy * sin(car_error_.laterror.error_phi))/(1 - car_error_.kappar * car_error_.laterror.error_d);  
    car_error_.lonerror.error_dot_s = match_point.v - dot_s;
    car_error_.lonerror.error_dot_dot_s = match_point.a - current_point.a;
    car_error_.lonerror.error_s = -1 * car_error_.lonerror.error_s;
    if(fabs(car_error_.lonerror.error_s > 0.3) || fabs(car_error_.lonerror.error_dot_s) > 0.3)
    {
        LOG_WARN("There is a risk of catapult starting!");
    }
    car_error_.lonerror.error_s_sum = car_error_.lonerror.error_s_sum + car_error_.lonerror.error_s;    
    car_error_.lonerror.error_s_sum =  (fabs(car_error_.lonerror.error_s_sum) > car_error_.lonerror.s_sum_limit) ? std::copysign(car_error_.lonerror.s_sum_limit,car_error_.lonerror.error_s_sum):car_error_.lonerror.error_s_sum;
    if(car_error_.lonerror.error_s_sum * car_error_.lonerror.error_s < 0)
        car_error_.lonerror.error_s_sum = 0;
    car_error_.lonerror.error_dot_s_sum = car_error_.lonerror.error_dot_s_sum + car_error_.lonerror.error_dot_s;
    car_error_.lonerror.error_dot_s_sum =  (fabs(car_error_.lonerror.error_dot_s_sum) > car_error_.lonerror.dot_s_sum_limit) ? std::copysign(car_error_.lonerror.dot_s_sum_limit,car_error_.lonerror.error_dot_s_sum):car_error_.lonerror.error_dot_s_sum;
    if(car_error_.lonerror.error_dot_s_sum * car_error_.lonerror.error_dot_s < 0)
        car_error_.lonerror.error_dot_s_sum = 0;
    //加入微分跟踪器降低纵向跟踪的阶跃 微分跟踪器暂定1秒的跟踪延时 速度柔化
    car_error_.lonerror.error_s_smooth = differential_tracker_es->work(car_error_.lonerror.error_s,dt);
    car_error_.lonerror.error_dot_s_smooth = differential_tracker_dot_es->work(car_error_.lonerror.error_dot_s,dt);
    car_error_.lonerror.error_dot_dot_s_smooth = differential_tracker_dot_dot_es->work(car_error_.lonerror.error_dot_dot_s,dt);
    car_error_.lonerror.error_s_sum_smooth = car_error_.lonerror.error_s_sum_smooth + car_error_.lonerror.error_s_smooth;    
    car_error_.lonerror.error_s_sum_smooth =  (fabs(car_error_.lonerror.error_s_sum_smooth) > car_error_.lonerror.s_sum_limit) ? std::copysign(car_error_.lonerror.s_sum_limit,car_error_.lonerror.error_s_sum_smooth):car_error_.lonerror.error_s_sum_smooth;
    if(car_error_.lonerror.error_s_sum_smooth * car_error_.lonerror.error_s_smooth < 0)
        car_error_.lonerror.error_s_sum_smooth = 0;
    LOG_INFO("current_point,x,y,v,phi,steer:{} ,{} ,{} ,{}, {}",
              current_point.x, current_point.y, current_point.v, current_point.phi, current_point.steer);
    LOG_INFO("match_point,x,y,v,phi,kappa: {} ,{} ,{} ,{}, {}",
              match_point.x, match_point.y, match_point.v, match_point.phi, match_point.kappa);
    LOG_INFO("lon_error:error_s,error_dot_s,error_dot_dot_s: {} ,{} ,{}",
              car_error_.lonerror.error_s, car_error_.lonerror.error_dot_s, car_error_.lonerror.error_dot_dot_s);

}

igv_msgs::msg::ControlInformation  LonControl::lon_contrl_calculate(Position current_car_status, Position match_point)
{
    igv_msgs::msg::ControlInformation res;
    res.brk_press = 0;
    double kp = m_lon_param.kp_lon,ki = m_lon_param.ki_lon,kd = m_lon_param.kd_lon; 
    double res_sp, forward_compensate;
    if(1 == m_lon_param.torque_rpm_mode)//选用底盘的速度控制模式。
    {
        res_sp = PID(car_error_.lonerror.error_s,kp,ki,kd,car_error_.lonerror.error_s_sum,car_error_.lonerror.error_s_last);

        // 是否选用自适应PID
        if(m_lon_param.is_use_adaptive_pid)
        {
            kp = adaptive_pid_.kp;
            ki = adaptive_pid_.ki;
            kd = adaptive_pid_.kd;
            res_sp = PID(car_error_.lonerror.error_s,kp,ki,kd,car_error_.lonerror.error_s_sum,car_error_.lonerror.error_s_last);
            adaptive_pid_.paramupdate(car_error_.lonerror.error_s,dt);
        }

        forward_compensate = match_point.v;
        
        if(match_point.v >= 0)//前进
        {
            res.speed = res_sp + forward_compensate;
        }else{
            //后退
            res.speed = res_sp + forward_compensate;
        }
        
        if(fabs(current_car_status.v) < 0.5 && match_point.v == 0.0)
        {
            res.speed = 0.0;
        }

        if((current_car_status.pitch < -3.5/180*3.14159) && (current_car_status.v >= 0.0 && current_car_status.v < 0.2) && (match_point.a > 0) && (match_point.v > 0))
        {   //坡道起步阶段，添加坡道补偿
            double slope_compensate = fabs(current_car_status.pitch * 180 / 3.14159 * m_lon_param.k_slope_compensate);
            res.speed = res.speed + slope_compensate;
            LOG_WARN("Slope start, slope compensation!!!");
        }

        res.speed = SpeedCheck(res.speed,m_lon_param);
        LOG_INFO("lon control mode: speed, commond, lon : {}", res.speed);
        //将速度四舍五入
        res.speed = round(res.speed * 10) / 10.0;

    }else if( 0 == m_lon_param.torque_rpm_mode)//选用底盘的扭矩模式
    {
        const double max_torque = 25.5;//数据来源与底盘产品说明
        const double max_torque_start = 18.0;
        double feedforward_torque = 0.0, t_p_r = 0.0, t_s_r = 0.0, a_refer = 0.0, origin_torque = 0.0;

        if(0 == feedforward_mode)//线性手动拟合稳态扭矩模式
        {
            double torque_base = 8.2625, k_torque_speed = 0.1, k_torque_slope = -34.9908 *2.0;
            double kp_t_p = 1.0, ki_t_p = 0.0, kd_t_p = 0.0;
            double kp_t_s = 60.0, ki_t_s = 1.0, kd_t_s = 0.0;
            double kp_a = 0.8;
            double kp_max_v = 1.0;
            feedforward_torque = torque_base + std::fabs(match_point.v) * k_torque_speed + current_car_status.pitch * k_torque_slope * std::copysign(1.0, match_point.v);
            t_p_r = PID(car_error_.lonerror.error_s, kp_t_p, ki_t_p, kd_t_p, car_error_.lonerror.error_s_sum, car_error_.lonerror.error_s_last);
            t_s_r = PID(car_error_.lonerror.error_dot_s + t_p_r, kp_t_s, ki_t_s, kd_t_s, car_error_.lonerror.error_dot_s_sum, car_error_.lonerror.error_dot_s_last);
            a_refer = kp_a * car_error_.lonerror.error_dot_dot_s + t_s_r;
            auto max_speed_com = (match_point.v > 0) ? ((current_car_status.v > m_lon_param.max_v)? kp_max_v * (m_lon_param.max_v - current_car_status.v): 0.0): ((current_car_status.v < m_lon_param.max_v_back)? kp_max_v * (m_lon_param.max_v_back - current_car_status.v): 0.0);
            LOG_IF(spdlog::level::info, std::fabs(max_speed_com) > 0, "max_speed_com: {}",max_speed_com);
            origin_torque = std::copysign(feedforward_torque, match_point.v) + a_refer + max_speed_com;//根据匹配点的速度方向，给前馈扭矩加上符号

            if(std::fabs(match_point.v) <= 0.21 && std::fabs(current_car_status.v) < 0.2)
                res.torque = match_point.v > 0 ? std::clamp(origin_torque, 0.0, max_torque_start) : std::clamp(origin_torque, -max_torque_start, 0.0);
            else
                res.torque = match_point.v > 0 ? std::clamp(origin_torque, 0.0, max_torque) : std::clamp(origin_torque, -max_torque, 0.0);

        }else if(1 == feedforward_mode)//神经网络拟合扭矩表模式
        {
            double kp_t_p = 0.1, ki_t_p = 0.0, kd_t_p = 0.0;
            double kp_t_s = 0.4, ki_t_s = 0.0, kd_t_s = 0.0;
            t_p_r = PID(car_error_.lonerror.error_s, kp_t_p, ki_t_p, kd_t_p, car_error_.lonerror.error_s_sum, car_error_.lonerror.error_s_last);
            t_s_r = PID(car_error_.lonerror.error_dot_s + t_p_r, kp_t_s, ki_t_s, kd_t_s, car_error_.lonerror.error_dot_s_sum, car_error_.lonerror.error_dot_s_last);

            a_refer = match_point.a + t_s_r;
            if(match_point.v >= 0.0)
            {
                origin_torque = FitTorqueTable(current_car_status.v, a_refer, current_car_status.pitch);
            }else{
                //为了匹配后退的工况。神经网络只拟合前进工况。
                origin_torque = FitTorqueTable(-current_car_status.v, -a_refer, -current_car_status.pitch);
                //为了匹配速度方向。
                origin_torque = -origin_torque;
            }
            res.torque = match_point.v > 0 ? std::clamp(origin_torque, 0.0, max_torque) : std::clamp(origin_torque, -max_torque, 0.0);
        }

        LOG_INFO("lon control mode: torque, feedforward mode: {}, feedforward_torque: {}, t_p_r: {}, t_s_r: {}, a_refer: {}, origin torque: {}, current speed: {}, acc: {}, pitch: {}.",
            1 == feedforward_mode ? "Neural Network" : "Linear Fit", feedforward_torque, t_p_r, t_s_r, a_refer, origin_torque, current_car_status.v, current_car_status.a, current_car_status.pitch);

        res.torque = TorqueCheck(res.torque);

        if(current_car_status.v > (m_lon_param.max_v + 0.5) || current_car_status.v < (m_lon_param.max_v_back - 0.1))
        {
            res.brk_press = 0.1 + (current_car_status.v > m_lon_param.max_v ? (current_car_status.v - m_lon_param.max_v) : (m_lon_param.max_v_back - current_car_status.v)) * 1.0;
            LOG_INFO("扭矩模式下，触发制动! 速度超过最大值: {}", current_car_status.v);
            res.speed = 0.0;
            res.torque = 0.0;
        }
    }
    
    //当出现冲坡现象时，采取制动
    if((((current_car_status.v - match_point.v) > 0.3 && match_point.v > 0.01) ||
        ((current_car_status.v - match_point.v) < -0.3 || match_point.v < -0.01)) &&
        fabs(current_car_status.pitch) > 2.0*3.14159/180)
    {
        res.brk_press = 0.1 + (current_car_status.v - match_point.v) * 1.0;

        LOG_INFO("冲坡！触发制动! pitch: {}", current_car_status.pitch);
    }

    //slope mode ~downhill
    if(m_lon_param.slope_mode )
    {
        double fixed_pressure = current_car_status.pitch / match_point.v * m_lon_param.k_pressure;
        res.brk_press =  fixed_pressure - 1 * car_error_.lonerror.error_dot_s * m_lon_param.kp_press;
        if(res.brk_press < 0)
            res.brk_press = 0;
    }

    return res;
}

double LonControl::FitTorqueTable(double speed, double acc, double pitch)
{
    return control::torque_network::Predict(speed, acc, pitch);
}

double LonControl::PID(double error, double kp, double ki, double kd, double sum_error, double &last_error)
{
    double res = kp * error + ki * sum_error + kd * (error - last_error);
    last_error = error;
    return res;
}

void PidController::init(double pp,double ii,double dd)
{
    kp = pp;
    ki = ii;
    kd = dd;
    last_error = 0;
    integral = 0;

}
void PidController::paramupdate(double err,double dt){
     // 示例调整规则，可以根据具体需求修改
    double abs_error = fabs(err);
 
    // 增强比例增益
    kp = kp + 0.1 * exp(-0.1 * abs_error);
 
    // 增强积分增益（避免积分饱和）
    ki = ki + 0.01 * exp(-0.05 * abs_error);
 
    // 增强微分增益（根据误差变化率调整）
    kd = kd + 0.01 * exp(-0.2 * fabs((err - last_error) / dt));

    last_error = err;

    integral = integral + err * dt ;

}

double LonControl::SpeedCheck(double speed,LongitudinalParam lonp)
{   
    if (isnan(speed)||isinf(speed)){   
        DataPool::getInstance()->get_control_status()->solution = 1;     
        LOG_ERROR("The speed is a NaN or a inf. ");
        return 0.0;    
    }else{
        DataPool::getInstance()->get_control_status()->solution = 0;
    }
    if(speed > 0.0 && speed < 0.1){
        speed = 0.1;
    }else if(speed > lonp.max_v)
    {
        LOG_WARN("lon calculate result greater than max_v, max_v: {}, origin speed: {}", lonp.max_v, speed);
        speed = lonp.max_v;        
    }else if(speed < lonp.max_v_back)
    {
        LOG_WARN("lon calculate result greater than max_v_back, max_v_back: {}, origin speed: {}",
                  lonp.max_v_back, speed);
        speed = lonp.max_v_back;
        
    }else if(speed < 0.0 && speed > -0.1)
    {
        speed = -0.1;
    }
    
    return speed;
}

double LonControl::TorqueCheck(double torque)
{
    if (!std::isfinite(torque)){
        DataPool::getInstance()->get_control_status()->solution = 1;
        LOG_ERROR("The torque is a NaN or a inf. ");
        return 0.0;
    }else{
        DataPool::getInstance()->get_control_status()->solution = 0;
    }
    return torque;
}

double LonControl::norm_angle(double angle)
{
    angle = std::fmod (angle,2*M_PI);
    angle = angle > M_PI ? (angle - 2*M_PI):angle;
    angle = angle < -M_PI ? (angle + 2*M_PI):angle;
    return angle;
}
void LonControl::clear_error()
{
    car_error_.laterror.error_d = 0;
    car_error_.laterror.error_dot_d = 0;
    car_error_.laterror.error_phi = 0;
    car_error_.laterror.error_dot_phi = 0;
    car_error_.laterror.error_d_sum = 0;   
    car_error_.laterror.last_error_d = 0;
    car_error_.laterror.last_error_phi = 0;
    car_error_.laterror.error_phi_sum = 0;
    car_error_.lonerror.error_s = 0;
    car_error_.lonerror.error_dot_s = 0;
    car_error_.lonerror.error_dot_dot_s = 0;
    car_error_.lonerror.error_s_sum = 0;
    car_error_.lonerror.error_s_last = 0;
    car_error_.lonerror.error_dot_s_sum = 0;
    car_error_.lonerror.error_dot_s_last = 0;
    differential_tracker_es->clear();
    differential_tracker_dot_es->clear();
    differential_tracker_dot_dot_es->clear();
    differential_tracker_refer_v->clear();

}
