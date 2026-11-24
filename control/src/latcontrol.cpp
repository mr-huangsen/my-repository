#include "control/latcontrol.h"
#include <sstream>

LatControl::LatControl(const int &model_type,const int &control_type){
    if(model_type != 1 && model_type != 2)
        {model_select_ = Model::KinematicsModel;}
    else
        {model_select_ = (Model)model_type;}
    if(control_type != 0 && control_type != 1 && control_type != 2)
        {control_type_ = ControlType::LQR;}
    else
        {control_type_ = (ControlType)control_type;}
    
    auto cp = DataPool::getInstance();
    m_lat_param = cp->get_lat_param();
    m_vehicle_param = cp->get_vehicle_param();
    
    if((model_select_ == Model::KinematicsModel))
    {
        A_.resize(2,2);
        B_.resize(2,1);
        x_.resize(2);
        u_lat_.resize(1);
        C_.resize(2,2);
        D_.resize(2,1);
        Q_.resize(2,2);
        R_.resize(1,1);
        if(control_type_ == ControlType::LQR)
        {
            Q_ << m_lat_param.Q_matrix_kinematics[0],0,
                  0,m_lat_param.Q_matrix_kinematics[1];            
            
            R_ << m_lat_param.R_matrix_kinematics_lat[0];
        }
        else if (control_type_ == ControlType::MPC)
        {
            Q_ << m_lat_param.Q_matrix_kinematics_mpc[0],0,
                  0,m_lat_param.Q_matrix_kinematics_mpc[1];            
            
            R_ << m_lat_param.R_matrix_kinematics_lat_mpc[0];
        }
    }
    else if((model_select_ == Model::DynamicalModel) )
    {
        A_.resize(4,4);
        B_.resize(4,1);
        x_.resize(4);
        u_lat_.resize(1);
        Q_.resize(4,4); 
        C_.resize(4,4);
        D_.resize(4,1);            
        R_.resize(1,1);
        if(control_type_ == ControlType::LQR)
        {
            Q_ << m_lat_param.Q_matrix_dynamical[0],0,0,0,
                  0,m_lat_param.Q_matrix_dynamical[0],0,0,
                  0,0,m_lat_param.Q_matrix_dynamical[0],0,
                  0,0,0,m_lat_param.Q_matrix_dynamical[0];  
            
            R_ << m_lat_param.R_matrix_dynamical_lat[0];
        }
        else if(control_type_ == ControlType::MPC)
        {
            Q_ << m_lat_param.Q_matrix_dynamical_mpc[0],0,0,0,
                  0,m_lat_param.Q_matrix_dynamical_mpc[0],0,0,
                  0,0,m_lat_param.Q_matrix_dynamical_mpc[0],0,
                  0,0,0,m_lat_param.Q_matrix_dynamical_mpc[0];  
            
            R_ << m_lat_param.R_matrix_dynamical_lat_mpc[0];
        }
        
    }    
    dt = cp->dt;
}
// LatControl::~LatControl(){}

double LatControl::lat_calculate(const Position &current_car_status, const Position &match_car_status, const Position &look_ahead_car_status)
{    
    status_update(current_car_status,match_car_status);    
    double res_angle = lat_control_calculate(current_car_status,match_car_status,look_ahead_car_status);    
    res_angle = res_angle - m_lat_param.manufacturing_accuracy_compensation/180*3.14159; 
    res_angle = LimitAngle(res_angle,m_lat_param);
    LOG_INFO("commond,lat : {}", res_angle);
    return res_angle;    
}
void LatControl::status_update(Position current_car_status, Position match_car_status)
{
    error_calculate(current_car_status,match_car_status);
    
    double vx = current_car_status.v;
    if(fabs(vx) < 0.075)
        {vx = std::copysign(0.075,vx);}  

    if(model_select_ == Model::KinematicsModel)
    {       
        A_ << 1 , vx * dt,
              0 ,  1;
        B_ << 0, vx * 1 / m_vehicle_param.L * dt;
        C_ << 1,0,
              0,1;
        D_ << 0,- vx * car_error_.kappar * dt;
        x_ << car_error_.laterror.error_d,car_error_.laterror.error_phi;
        
    } else if(model_select_ == Model::DynamicalModel)
    {  
        A_ << 0 , 1 , 0 , 0,
              0 , 2 * (m_vehicle_param.Cf+m_vehicle_param.Cr) / m_vehicle_param.m / vx, -2 * (m_vehicle_param.Cf+m_vehicle_param.Cr) / m_vehicle_param.m, 2 * (m_vehicle_param.Cf*m_vehicle_param.lf - m_vehicle_param.Cr*m_vehicle_param.lr) / m_vehicle_param.m / vx,
              0 , 0 , 0 , 1,
              0 , 2*(m_vehicle_param.Cf*m_vehicle_param.lf-m_vehicle_param.Cr*m_vehicle_param.lr) / m_vehicle_param.I / vx, -2 * (m_vehicle_param.Cf*m_vehicle_param.lf-m_vehicle_param.Cr*m_vehicle_param.lr) / m_vehicle_param.I, 2 * (m_vehicle_param.Cf*m_vehicle_param.lf*m_vehicle_param.lf+m_vehicle_param.Cr*m_vehicle_param.lr*m_vehicle_param.lr) / m_vehicle_param.I / vx;
        A_ = A_ * dt + Eigen::MatrixXd::Identity(4, 4);
        B_ << 0 , -2*m_vehicle_param.Cf/m_vehicle_param.m, 0, -2*m_vehicle_param.Cf*m_vehicle_param.lf/m_vehicle_param.I;
        B_ = B_ * dt;
        C_ << 1,0,0,0,
              0,1,0,0,
              0,0,1,0,
              0,0,0,1;
        D_ << 0,(2 * m_vehicle_param.Cf * m_vehicle_param.lf - 2 * m_vehicle_param.Cr * m_vehicle_param.lr) / m_vehicle_param.m / vx - vx,0,(2 * m_vehicle_param.Cf * m_vehicle_param.lf * m_vehicle_param.lf + 2 * m_vehicle_param.Cr * m_vehicle_param.lr * m_vehicle_param.lr) / m_vehicle_param.I / vx ;
        D_ = D_ * match_car_status.dot_phi * dt;
        x_ << car_error_.laterror.error_d,car_error_.laterror.error_dot_d,car_error_.laterror.error_phi,car_error_.laterror.error_dot_phi;
        
    }  
    LOG_INFO("model_select_: {}", ((model_select_ == Model::KinematicsModel)?"KinematicsModel":"DynamicalModel"));

}
void LatControl::error_calculate(Position current_point, Position match_point)
{
    double vx = current_point.v;
    double vy = 0;
    Eigen::Vector2d v_tou;
    v_tou << cos(match_point.phi),sin(match_point.phi);
    Eigen::Vector2d v_n;
    v_n << -sin(match_point.phi),cos(match_point.phi);
    Eigen::Vector2d temp_v;
    temp_v << current_point.x - match_point.x, current_point.y - match_point.y;
    car_error_.kappar = match_point.kappa; 
    car_error_.laterror.error_d = v_n.dot(temp_v);
    car_error_.lonerror.error_s = v_tou.dot(temp_v);
    double theta_r = match_point.phi + car_error_.lonerror.error_s * car_error_.kappar;
    car_error_.laterror.error_phi = norm_angle(current_point.phi - theta_r);    
    car_error_.laterror.error_dot_d = vy * cos(car_error_.laterror.error_phi) + vx * sin(car_error_.laterror.error_phi);    
    double dot_s = (vx * cos(car_error_.laterror.error_phi) - vy * sin(car_error_.laterror.error_phi))/(1 - car_error_.kappar * car_error_.laterror.error_d);  
    car_error_.laterror.error_dot_phi = current_point.dot_phi - car_error_.kappar * dot_s;    
    
    car_error_.laterror.error_d_sum = car_error_.laterror.error_d_sum + car_error_.laterror.error_d;
    car_error_.laterror.error_d_sum = (fabs(car_error_.laterror.error_d_sum) > car_error_.laterror.d_sum_limit) ? std::copysign(car_error_.laterror.d_sum_limit,car_error_.laterror.error_d_sum):car_error_.laterror.error_d_sum;
    // if(car_error_.laterror.error_d_sum * car_error_.laterror.error_d < 0)    
    //     car_error_.laterror.error_d_sum = 0;
    
    car_error_.laterror.error_phi_sum += car_error_.laterror.error_phi;
    car_error_.laterror.error_phi_sum = (fabs(car_error_.laterror.error_phi_sum) > car_error_.laterror.phi_sum_limit) ? std::copysign(car_error_.laterror.phi_sum_limit,car_error_.laterror.error_phi_sum):car_error_.laterror.error_phi_sum;
    // if(car_error_.laterror.error_phi_sum * car_error_.laterror.error_phi < 0)    
    //     car_error_.laterror.error_phi_sum = 0;
    
    if(car_error_.laterror.error_d > 0.75)
        {DataPool::getInstance()->get_control_status()->deviation = 1; LOG_ERROR("deviation path !!");}
    else
        {DataPool::getInstance()->get_control_status()->deviation = 0; }
    LOG_INFO("lat_error:error_d,error_dot_d,error_phi,error_dot_phi: {} ,{} ,{} ,{}",
              car_error_.laterror.error_d, car_error_.laterror.error_dot_d, car_error_.laterror.error_phi,
              car_error_.laterror.error_dot_phi);
    LOG_INFO("kappar: {}", car_error_.kappar);

}
double LatControl::lat_control_calculate(Position current_point, Position match_point, Position look_ahead_point)
{    
    double res = 0.0;
    double commond_lat = 0.0;
    // RCLCPP_INFO(rclcpp::get_logger("control"), "control_type_: %d",control_type_);
    if(control_type_ == ControlType::LQR)
        {res = LQR(A_,B_,x_);}
    else if(control_type_ == ControlType::MPC)
        {res = MPC();}
    else if(control_type_ == ControlType::PP)
        {res = PP(current_point,look_ahead_point);}    

    double forward_compensate = atan(car_error_.kappar * m_vehicle_param.L);
    if(model_select_ == Model::DynamicalModel)
    {
        forward_compensate = car_error_.kappar * (m_vehicle_param.L-m_vehicle_param.lr*k03_-(m_vehicle_param.m*current_point.v*current_point.v/m_vehicle_param.L)*(m_vehicle_param.lr/m_vehicle_param.Cf+m_vehicle_param.lf/m_vehicle_param.Cr*k03_-m_vehicle_param.lf/m_vehicle_param.Cr));
    }
    if(control_type_ == ControlType::PP || control_type_ == ControlType::MPC)
       {forward_compensate = 0;}
        
    double model_compensate = m_lat_param.model_compensate*std::copysign(car_error_.laterror.error_d * car_error_.kappar,car_error_.laterror.error_d);
    if(control_type_ == ControlType::PP)
        model_compensate = 0;
    double steady_state_compensation = car_error_.laterror.error_d_sum * m_lat_param.ki_lat;
    double phi_ki_compensation = car_error_.laterror.error_phi_sum * m_lat_param.ki_phi;
    double vibration_compensate = m_lat_param.kd_lat * (car_error_.laterror.error_d - car_error_.laterror.last_error_d);
    double phi_compensate = m_lat_param.kd_phi * (car_error_.laterror.last_error_phi - car_error_.laterror.last_error_phi);
    
    if(match_point.v >= 0)//前进
    {
        commond_lat = res + forward_compensate - steady_state_compensation  - phi_ki_compensation - model_compensate - vibration_compensate - phi_compensate; 
        
        if(model_select_ == Model::DynamicalModel)
        {
            commond_lat = res + forward_compensate - steady_state_compensation  - phi_ki_compensation - model_compensate; 
        }
    }else{
        //后退
        commond_lat = res - forward_compensate - steady_state_compensation  - phi_ki_compensation - model_compensate - vibration_compensate - phi_compensate;
        if(model_select_ == Model::DynamicalModel)
        {
            commond_lat = res - forward_compensate - steady_state_compensation  - phi_ki_compensation - model_compensate; 
        }
    }
    if(isnan(commond_lat)||isinf(commond_lat))
    {
        if(isnan(forward_compensate)||isinf(forward_compensate))
            LOG_ERROR("{}{}", car_error_.kappar, m_vehicle_param.L);
        if(isnan(steady_state_compensation)||isinf(steady_state_compensation))
            LOG_ERROR("{}{}", car_error_.laterror.error_d_sum, m_lat_param.ki_lat);
        if(isnan(phi_ki_compensation)||isinf(phi_ki_compensation))
            LOG_ERROR("{}{}", car_error_.laterror.error_phi_sum, m_lat_param.ki_phi);
        if(isnan(model_compensate)||isinf(model_compensate))
            LOG_ERROR("{}", m_lat_param.model_compensate);
        if(isnan(vibration_compensate)||isinf(vibration_compensate)) 
            LOG_ERROR("{}{}{}", m_lat_param.kd_lat, car_error_.laterror.error_d, car_error_.laterror.last_error_d);
        if(isnan(phi_compensate)||isinf(phi_compensate))
            LOG_ERROR("{}{}", m_lat_param.kd_phi, car_error_.laterror.last_error_phi);
    }
    car_error_.laterror.last_error_d = car_error_.laterror.error_d;
    car_error_.laterror.last_error_phi = car_error_.laterror.error_phi;
    return commond_lat;

}

double LatControl::LQR(Eigen::MatrixXd A, Eigen::VectorXd B, Eigen::VectorXd x)
{     
    Eigen::MatrixXd P,P_;
    P = Q_;
    Eigen::MatrixXd AT = A.transpose();
    Eigen::RowVectorXd BT = B.transpose();
    Eigen::MatrixXd RI = Eigen::MatrixXd::Identity(R_.rows(), R_.cols());
    for(int i=0;i<m_lat_param.max_iter;i++){
        // 使用Cholesky分解替代直接求逆
        Eigen::MatrixXd RR = R_ + BT*P*B;
        Eigen::LLT<Eigen::MatrixXd> lltOfR(RR);
        Eigen::MatrixXd RR_inv = lltOfR.solve(RI);        
        P_ = Q_ + AT*P*A-AT*P*B*RR_inv*BT*P*A;
        // P_ = Q_ + A.transpose()*P*A-A.transpose()*P*B*(R_ + B.transpose()*P*B).inverse()*B.transpose()*P*A;
        if((P-P_).array().abs().maxCoeff()<m_lat_param.eps_iter){            
            break;
        }
        P = P_;
    }
    Eigen::MatrixXd K = (R_+B.transpose()*P_*B).inverse()*B.transpose()*P_*A;
    Eigen::MatrixXd res = -K*x;
    if(model_select_ == Model::DynamicalModel)
        k03_ = K(0,2);    
    
    std::ostringstream result_log;
    result_log << res;
    LOG_INFO("LQR RESULT: {}", result_log.str());
    return res(0,0);
}

double LatControl::PP(Position current_car_status, Position lookahead_point){
    
    double total_ld = lookahead_dist;
    double alpha = atan2((lookahead_point.y-current_car_status.y),(lookahead_point.x-current_car_status.x)) - current_car_status.phi;
    alpha = norm_angle(alpha);
    double delta = atan(2 * m_vehicle_param.L * sin(alpha)/total_ld);
    return delta;
}
double LatControl::MPC(){
    //Construct a prediction matrix
    make_prediction_matrix(x_,u_lat_,A_,B_,C_,D_,Q_,R_);
    //Construct constraint conditions
    make_constraint_conditions(x_,u_lat_);
    //qp solve
    if(qp_solve_mpc->run(H_,g_,A_con_,l_inf_,u_sup_))
    {
        DataPool::getInstance()->get_control_status()->solution = 0;
        return (qp_solve_mpc->osqp_solution(0) + u_lat_(0));
    }
    else
    {
        DataPool::getInstance()->get_control_status()->solution = 1; 
        LOG_ERROR("mpc solve failed!!! ");
        return 0.0;
    } 
}
void LatControl::make_prediction_matrix(Eigen::VectorXd x,Eigen::VectorXd u,Eigen::MatrixXd A,Eigen::MatrixXd B,Eigen::MatrixXd C,Eigen::MatrixXd D,Eigen::MatrixXd Q,Eigen::MatrixXd R)
{
    int a = A.rows();
    int b = B.cols();
    int d = D.cols();
    Eigen::MatrixXd Sx = Eigen::MatrixXd::Zero(a*m_lat_param.Hp, a);
    Eigen::MatrixXd sum = Eigen::MatrixXd::Zero(a, a);
    Eigen::MatrixXd Su = Eigen::MatrixXd::Zero(a* m_lat_param.Hp, b);
    Eigen::MatrixXd S_u = Eigen::MatrixXd::Zero(a* m_lat_param.Hp, b*m_lat_param.Hu);
    Eigen::MatrixXd Sd = Eigen::MatrixXd::Zero(a* m_lat_param.Hp, d);
    Eigen::MatrixXd S_u_ = Eigen::MatrixXd::Zero(a* m_lat_param.Hp, b*m_lat_param.Hp);
    Eigen::MatrixXd Refer = Eigen::MatrixXd::Zero(a* m_lat_param.Hp, 1);
    Eigen::MatrixXd refer = Eigen::MatrixXd::Zero(a,1);
    for(int i = 0;i<m_lat_param.Hp;i++)
    {
        Sx.block(i*a,0,a,a) = calc_power(A,i+1,a);
        sum = sum + calc_power(A,i,a);
        Su.block(i*a,0,a,b) = sum * B;
        Sd.block(i*a,0,a,d) = sum * D;
        S_u_.block((m_lat_param.Hp-i-1)*a,(m_lat_param.Hp-i-1)*b,(i+1)*a,b) = Su.block(0,0,(i+1)*a,b);
        Refer.block(i*a,0,a,1) = refer;
    }
    S_u = S_u_.block(0,0,a*m_lat_param.Hp,b*m_lat_param.Hu);

    Eigen::MatrixXd Cz = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(m_lat_param.Hp, m_lat_param.Hp),C);
    Eigen::MatrixXd C_q = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(m_lat_param.Hp, m_lat_param.Hp),Q);
    Eigen::MatrixXd C_r = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(m_lat_param.Hu, m_lat_param.Hu),R);
    
    //误差等于Refer - Cz * Sx * x - Cz * Su * u - Cz * Sd
    Eigen::MatrixXd m_Error  = Refer - (Cz * Sx * x + Cz * Su * u + Cz * Sd);//
    X_ = Cz * Sx * x + Cz * Su * u + Cz * Sd; 
    Eigen::MatrixXd Y = Cz * S_u;
    Su_b_ = Y;
    H_ = 2 * Y.transpose() * C_q * Y + 2 * C_r;
    g_ = -2 * Y.transpose() * C_q * m_Error;
}
void LatControl::make_constraint_conditions(Eigen::VectorXd x,Eigen::VectorXd u){
    // l_inf < Ax < u_sup
    // 控制量的最大值 控制量变化量的最大值 
    double du_min = -m_lat_param.max_w_delta / 1;
    double du_max = m_lat_param.max_w_delta / 1;
    double u_min = -m_lat_param.max_w;
    double u_max = m_lat_param.max_w;
    double x_max = std::numeric_limits<double>::infinity();
    double x_min = -std::numeric_limits<double>::infinity();

    int b = u.rows();
    int a = x.rows();
    Eigen::MatrixXd con_du = Eigen::MatrixXd::Zero(2,b);
    Eigen::MatrixXd con_u = Eigen::MatrixXd::Zero(2,b);
    Eigen::MatrixXd con_x = Eigen::MatrixXd::Zero(2,a);
    for(int i = 0; i < b;i++)
    {
        con_du(0,i) = du_min;
        con_du(1,i) = du_max;
        con_u(0,i) = u_min;
        con_u(1,i) = u_max;        
    }
    //状态量的最大值
    for(int i = 0; i < a;i++)
    {
        con_x(0,i) = x_min;
        con_x(1,i) = x_max;
    }
    Eigen::MatrixXd tempo = Eigen::MatrixXd::Zero(b,b * m_lat_param.Hu);
    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(b * m_lat_param.Hu, b* m_lat_param.Hu);
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(b * m_lat_param.Hu, b* m_lat_param.Hu);
    Eigen::MatrixXd con_Du = Eigen::MatrixXd::Zero(con_du.cols() * m_lat_param.Hu,con_du.rows());
    Eigen::MatrixXd con_U = Eigen::MatrixXd::Zero(con_u.cols() * m_lat_param.Hu,con_u.rows());

    for(int i = 0;i<m_lat_param.Hu;i++)
    {
        T.block(i*b,i*b,b,b) = Eigen::MatrixXd::Identity(b,b);
        tempo.block(0,i*b,b,b) = Eigen::MatrixXd::Identity(b,b);
        L.block(i*b,0,b,b*m_lat_param.Hu) = tempo;
        con_Du.block(con_du.cols() * i,0,con_du.cols(),con_du.rows()) = con_du.transpose();
        con_U.block(con_u.cols() * i,0,con_u.cols(),con_u.rows()) = con_u.transpose();
    }

    // Eigen::MatrixXd Su_b_ = Cz * S_u;
    
    Eigen::MatrixXd con_X_k = Eigen::MatrixXd::Zero(con_x.cols() * m_lat_param.Hp,con_x.rows());
    for(int j = 0; j < m_lat_param.Hp;j++)
        {con_X_k.block(con_x.cols() * j,0,con_x.cols(),con_x.rows()) = con_x.transpose();}
    // Eigen::MatrixXd X_ = Cz * Sx * x + Cz * Su * u + Cz * Sd ;
    Eigen::MatrixXd XX(X_.rows(),X_.cols() * 2);
    XX << X_, X_;
    Eigen::MatrixXd con_X =  con_X_k - XX;
    A_con_.resize(T.rows() + L.rows() + Su_b_.rows(),T.cols());
    A_con_ << T, L, Su_b_;
    
    l_inf_.resize(con_Du.rows() + con_U.rows() + con_X.rows(),1);
    l_inf_ << con_Du.block(0,0,con_Du.rows(),1),con_U.block(0,0,con_U.rows(),1),con_X.block(0,0,con_X.rows(),1);
    
    u_sup_.resize(con_Du.rows() + con_U.rows() + con_X.rows(),1);
    u_sup_ << con_Du.block(0,1,con_Du.rows(),1),con_U.block(0,1,con_U.rows(),1),con_X.block(0,1,con_X.rows(),1);
    
}
Eigen::MatrixXd LatControl::multi_matrix(Eigen::MatrixXd Matrix_1, Eigen::MatrixXd Matrix_2, const int n)
//两个方阵相乘，n为方阵维度
{
    Eigen::MatrixXd Matrix;
    Matrix.resize(Matrix_1.rows(), Matrix_1.cols());
    Matrix = Eigen::MatrixXd::Zero(Matrix_1.rows(), Matrix_1.cols());
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            //Matrix(i,j) = 0;
            for (int k = 0; k < n; k++) {
                Matrix(i, j) += Matrix_1(i, k) * Matrix_2(k, j);
            }
        }
    }

    return Matrix;
}

Eigen::MatrixXd LatControl::calc_power(Eigen::MatrixXd Matrix, const int m, const int n)
//计算次幂 ，m> 0;
//m 为次方数  n为方阵维度
{
    if (m == 1)
        return Matrix;
    else
    {
        Eigen::MatrixXd Matrix_copy;   //复制当前矩阵
        Matrix_copy.resize(Matrix.rows(), Matrix.cols());
        Matrix_copy = Matrix;
        // Matrix_copy = Eigen::MatrixXd::Zero(Matrix.rows(), Matrix.cols());
        for (int i = 0; i < m - 1; i++)
        {
            Matrix = multi_matrix(Matrix, Matrix_copy, n);
        }
        return Matrix;
    }
}

double LatControl::norm_angle(double angle)
{
    angle = std::fmod (angle,2*M_PI);
    angle = angle > M_PI ? (angle - 2*M_PI):angle;
    angle = angle < -M_PI ? (angle + 2*M_PI):angle;
    return angle;
}
double LatControl::LimitAngle(double angle,LateralParam latp)
{
    if (isnan(angle)||isinf(angle)){     
        DataPool::getInstance()->get_control_status()->solution = 1;    
        LOG_ERROR("The angle is a NaN or a inf. ");
        return 0.0;    
    }else
    {
        DataPool::getInstance()->get_control_status()->solution = 0;
    }
    angle = (angle > latp.max_w) ? latp.max_w : angle;
    angle = (angle < latp.min_w) ? latp.min_w : angle;
    return angle;
}

void LatControl::clear_error()
{
    car_error_.laterror.error_d = 0;
    car_error_.laterror.error_dot_d = 0;
    car_error_.laterror.error_phi = 0;
    car_error_.laterror.error_dot_phi = 0;
    car_error_.laterror.error_d_sum = 0;   
    car_error_.laterror.last_error_d = 0;
    car_error_.laterror.last_error_phi = 0;
    car_error_.lonerror.error_s = 0;
    car_error_.lonerror.error_dot_s = 0;
    car_error_.lonerror.error_dot_dot_s = 0;
    car_error_.lonerror.error_s_sum = 0;
    car_error_.lonerror.error_s_last = 0;   
    car_error_.laterror.error_phi_sum = 0;  

}
