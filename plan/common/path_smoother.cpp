#include "common/path_smoother.h"

PathSmoother::PathSmoother(int pn):point_number(pn){
     w_s = 0.3;
     w_d = 0.3;
     w_r = 0.3;
     buffer_x = 0.2;
     buffer_y = 0.2;
     stack = 0.0;
     max_kappa = 0.4;

    A_s.resize(2,6);
    A_d.resize(2,4);
    A_r.resize(2,2);
    A_s << 1,0,-2,0,1,0,
           0,1,0,-2,0,1;
    A_d << 1,0,-1,0,
           0,1,0,-1;
    A_r << 1,0,
           0,1;

    temp_A_s.resize(2*(point_number-2),point_number*2);
    temp_A_d.resize(2*(point_number-1),point_number*2);
    temp_A_r.resize(point_number*2, point_number*2);
    temp_A_s.setZero();
    temp_A_d.setZero();    
    temp_A_r.setZero();        
       
    for(int i=0;i<point_number-2;i++)
    {
        temp_A_s.block(i*2,i*2,2,6) = A_s;
    }
    for(int i=0;i<point_number-1;i++)
    {
        temp_A_d.block(i*2,i*2,2,4) = A_d;
    }
    temp_A_r = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(point_number, point_number),A_r);
    
    H = 2 * (w_s*temp_A_s.transpose() * temp_A_s + w_d*temp_A_d.transpose() * temp_A_d + w_r*temp_A_r.transpose() * temp_A_r);
    g.resize(2 * point_number); 
    //约束
    A_con_1 = Eigen::MatrixXd::Identity(2*point_number, 2*point_number);
    l_inf_1.resize(2*point_number);
    u_sup_1.resize(2*point_number);    
    //曲律约束
    A_f.resize(6,6);
    A_f << 2,0,-4,0,2,0,
           0,2,0,-4,0,2,
           -4,0,8,0,-4,0,
           0,-4,0,8,0,-4,
           2,0,-4,0,2,0,
           0,2,0,-4,0,2;
    A_con_k.resize(point_number-2,2*point_number);
    l_inf_k.resize(point_number-2);
    u_sup_k.resize(point_number-2);
    temp_xy.resize(6);
    //总约束
    A_con.resize(3*point_number-2,2*point_number);
    l_inf.resize(3*point_number-2);
    u_sup.resize(3*point_number-2);
}
PathSmoother::PathSmoother(double w_s,double w_d,double w_r,double buffer_x,double buffer_y,double max_kappa,int pn): w_s(w_s),w_d(w_d),w_r(w_r),
            buffer_x(buffer_x),buffer_y(buffer_y),max_kappa(max_kappa),point_number(pn){
    stack = 0.0;
}

PathSmoother::~PathSmoother(){}

bool PathSmoother::run(LocalTrajectory local_trajectory)
{	     
    for(int i = 0;i < point_number;i++)
    {
        g(2*i) =  local_trajectory[i].x;
        g(2*i + 1) =  local_trajectory[i].y;
    }
    g = -2 * w_r * g;
    
    //约束
    for(int i = 0;i<point_number;i++)
    {
        l_inf_1(2*i) = local_trajectory[i].x - buffer_x;
        l_inf_1(2*i + 1) = local_trajectory[i].y - buffer_y;
        u_sup_1(2*i) = local_trajectory[i].x + buffer_x;
        u_sup_1(2*i + 1) = local_trajectory[i].y + buffer_y;
    }
     //曲率约束
    double sum_s = 0;
    for(int i = 1;i<point_number-1;i++)
    {
        sum_s = sum_s + std::hypot(local_trajectory[i+1].x-local_trajectory[i].x,local_trajectory[i+1].y-local_trajectory[i].y);
    }
    double temp_ds = sum_s / (point_number-1);
    
    for(int i = 1;i< point_number-1;i++)
    {
        double F_ref = pow((local_trajectory[i-1].x+local_trajectory[i+1].x-2*local_trajectory[i].x),2) + 
                       pow((local_trajectory[i-1].y+local_trajectory[i+1].y-2*local_trajectory[i].y),2) - stack;

        temp_xy << local_trajectory[i-1].x,local_trajectory[i-1].y,
                   local_trajectory[i].x,local_trajectory[i].y,
                   local_trajectory[i+1].x,local_trajectory[i+1].y;
   
        Eigen::VectorXd diff_ref = A_f * temp_xy;

        A_con_k.block(i-1,(i-1)*2,1,6) = diff_ref.transpose();

        l_inf_k(i-1) = -std::numeric_limits<double>::infinity();
        u_sup_k(i-1) = (temp_ds * temp_ds + max_kappa) * (temp_ds * temp_ds + max_kappa) + diff_ref.transpose() * temp_xy - F_ref;
    }   
    //总约束
    A_con << A_con_1,A_con_k;
    l_inf << l_inf_1,l_inf_k;
    u_sup << u_sup_1,u_sup_k;
    // if(qp_solve_path_smoother->run(H,g,A_con,l_inf,u_sup))
    if(qp_solve_path_smoother->run(H,g,A_con_1,l_inf_1,u_sup_1))
    {
        result.clear();
        Position temp_position;
        for(int i = 0; i < point_number; i++)
        {
            temp_position.x = qp_solve_path_smoother->osqp_solution(2*i);
            temp_position.y = qp_solve_path_smoother->osqp_solution(2*i+1);
            result.push_back(temp_position);
        }
        LOG_INFO("smoother path success !!");
        return true;
    }
    else
    {
        result = local_trajectory;
        LOG_ERROR("smoother path failure !!, origin path!!");
        return false;
    }
}
// bool PathSmoother::run(LocalTrajectory local_trajectory)
// {	
//     int point_number = local_trajectory.size(); 
//     Eigen::MatrixXd A_s;
//     Eigen::MatrixXd A_d;
//     Eigen::MatrixXd A_r;
//     A_s.resize(2,6);
//     A_d.resize(2,4);
//     A_r.resize(2,2);
//     A_s << 1,0,-2,0,1,0,
//            0,1,0,-2,0,1;
//     A_d << 1,0,-1,0,
//            0,1,0,-1;
//     A_r << 1,0,
//            0,1;
//     // Eigen::MatrixXd temp_A_s = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(point_number-2, point_number),A_s);
//     // Eigen::MatrixXd temp_A_d = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(point_number-1, point_number),A_d);
//     Eigen::MatrixXd temp_A_r = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(point_number, point_number),A_r);
//     Eigen::MatrixXd temp_A_s;
//     Eigen::MatrixXd temp_A_d;
//     temp_A_s.resize(2*(point_number-2),point_number*2);
//     temp_A_d.resize(2*(point_number-1),point_number*2);
//     temp_A_s.setZero();
//     temp_A_d.setZero();    
//     for(int i=0;i<point_number-2;i++)
//     {
//         temp_A_s.block(i*2,i*2,2,6) = A_s;
//     }
//     for(int i=0;i<point_number-1;i++)
//     {
//         temp_A_d.block(i*2,i*2,2,4) = A_d;
//     }
//     Eigen::MatrixXd AA_s = temp_A_s.transpose() * temp_A_s;
//     Eigen::MatrixXd AA_d = temp_A_d.transpose() * temp_A_d;
//     Eigen::MatrixXd AA_r = temp_A_r.transpose() * temp_A_r;
//     Eigen::MatrixXd H = w_s*AA_s + w_d*AA_d + w_r*AA_r;
//     Eigen::VectorXd g;
//     g.resize(2 * point_number);    
//     for(int i = 0;i < point_number;i++)
//     {
//         g(2*i) =  local_trajectory[i].x;
//         g(2*i + 1) =  local_trajectory[i].y;
//     }
//     g = -2 * w_r * g;
//     //约束    
//     Eigen::MatrixXd A_con_1 = Eigen::MatrixXd::Identity(2*point_number, 2*point_number);
//     Eigen::VectorXd l_inf_1, u_sup_1;
//     l_inf_1.resize(2*point_number);
//     u_sup_1.resize(2*point_number);
//     for(int i = 0;i<point_number;i++)
//     {
//         l_inf_1(2*i) = local_trajectory[i].x - buffer_x;
//         l_inf_1(2*i + 1) = local_trajectory[i].y - buffer_y;
//         u_sup_1(2*i) = local_trajectory[i].x + buffer_x;
//         u_sup_1(2*i + 1) = local_trajectory[i].y + buffer_y;
//     }
//      //曲率约束
//     double sum_s = 0;
//     for(int i = 1;i<point_number-1;i++)
//     {
//         sum_s = sum_s + std::hypot(local_trajectory[i+1].x-local_trajectory[i].x,local_trajectory[i+1].y-local_trajectory[i].y);
//     }
//     double temp_ds = sum_s / (point_number-1);

//     Eigen::MatrixXd A_f;
//     A_f.resize(6,6);
//     A_f << 2,0,-4,0,2,0,
//            0,2,0,-4,0,2,
//            -4,0,8,0,-4,0,
//            0,-4,0,8,0,-4,
//            2,0,-4,0,2,0,
//            0,2,0,-4,0,2;
//     Eigen::MatrixXd A_con_k;    
//     Eigen::VectorXd l_inf_k, u_sup_k;
//     A_con_k.resize(point_number-2,2*point_number);
//     l_inf_k.resize(point_number-2);
//     u_sup_k.resize(point_number-2);
//     Eigen::VectorXd temp_xy;
//     temp_xy.resize(6);
//     for(int i = 1;i< point_number-1;i++)
//     {
//         double F_ref = pow((local_trajectory[i-1].x+local_trajectory[i+1].x-2*local_trajectory[i].x),2) + 
//                        pow((local_trajectory[i-1].y+local_trajectory[i+1].y-2*local_trajectory[i].y),2) - stack;

//         temp_xy << local_trajectory[i-1].x,local_trajectory[i-1].y,
//                    local_trajectory[i].x,local_trajectory[i].y,
//                    local_trajectory[i+1].x,local_trajectory[i+1].y;
   
//         Eigen::VectorXd diff_ref = A_f * temp_xy;

//         A_con_k.block(i-1,(i-1)*2,1,6) = diff_ref.transpose();

//         l_inf_k(i-1) = -std::numeric_limits<double>::infinity();
//         u_sup_k(i-1) = (temp_ds * temp_ds + max_kappa) * (temp_ds * temp_ds + max_kappa) + diff_ref.transpose() * temp_xy - F_ref;
//     }
//     Eigen::MatrixXd A_con;    
//     Eigen::VectorXd l_inf, u_sup;
//     A_con.resize(3*point_number-2,2*point_number);
//     l_inf.resize(3*point_number-2);
//     u_sup.resize(3*point_number-2);

//     A_con << A_con_1,A_con_k;
//     l_inf << l_inf_1,l_inf_k;
//     u_sup << u_sup_1,u_sup_k;
//     if(qp_solve_path_smoother->run(H,g,A_con,l_inf,u_sup))
//     {
//         result.resize(point_number);
//         Position temp_position;
//         for(int i = 0;i<point_number;i++)
//         {
//             temp_position.x = qp_solve_path_smoother->osqp_solution(2*i);
//             temp_position.y = qp_solve_path_smoother->osqp_solution(2*i+1);
//             result.push_back(temp_position);
//         }
//         return true;
//     }
//     else
//     {
//         result = local_trajectory;
//         return false;
//     }
    


    
// }
