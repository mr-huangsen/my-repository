#include "lattic_plan/Frenet_path.hpp"
#define M_PI 3.14159265358979323846

namespace
{
double nowSeconds()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

// const double MAX_SPEED = 12.5; // maximum speed [m/s]
// const double MAX_SPEED_LAT = 100.0; //maximum angluar 
// const double MAX_ACCEL = 10.0;  // maximum acceleration [m/ss]
// const double MAX_CURVATURE = 0.4;  // maximum curvature [1/m]
// const double MAX_ROAD_WIDTH = 5.0;  // maximum road width [m]
// const double D_ROAD_W = 1.0;  // road width sampling length [m]
// const double DT = 0.1;  // time tick [s]
// const double MAXT = 15.0;  // max prediction time [m]
// const double MINT = 1.0;  // min prediction time [m]
// const double TARGET_SPEED = 1.5;  // target speed [m/s]
// const double D_T_S = 0.3;  // target speed sampling length [m/s]
// const double SMALL_SPEED = 0.3;
// const double DS = 5;  // distance tick [s]
// const double MAXS = 15.0;  // max prediction time [m]
// const double MINS = 5.0;  // min prediction time [m]
// // cost weights
// const double KJ = 0.1;
// const double KT = 0.1;
// const double KJD = 2.0;
// const double KD = 20.0;
// const double KLAT = 1.0;
// const double KLON = 1.0;
// const double KCAC = 0.0;
// const double KEFF = 0.0;

double norm_angle(double angle)
{
    angle = std::fmod (angle,2*M_PI);
    angle = angle > M_PI ? (angle - 2*M_PI):angle;
    angle = angle < -M_PI ? (angle + 2*M_PI):angle;
    return angle;
}

Frenet_plan::Frenet_plan(LatticConfig l_c, double con_cyc)
{
    MAX_SPEED = l_c.MAX_SPEED; 
    MAX_SPEED_LAT = l_c.MAX_SPEED_LAT; 
    MAX_ACCEL = l_c.MAX_ACCEL;  
    MAX_CURVATURE = l_c.MAX_CURVATURE;  
    MAX_ROAD_WIDTH = l_c.MAX_ROAD_WIDTH; 
    D_ROAD_W = l_c.D_ROAD_W;  
    DT = l_c.DT;  
    MAXT = l_c.MAXT;  
    MINT = l_c.MINT;  
    TARGET_SPEED = l_c.TARGET_SPEED; 
    D_T_S = l_c.D_T_S; 
    SMALL_SPEED = l_c.SMALL_SPEED;
    DS = l_c.DS;  
    MAXS = l_c.MAXS;  
    MINS = l_c.MINS;  
    DTIME = l_c.DTIME;
    // cost weights
    KJ = l_c.KJ;
    KT = l_c.KT;
    KJD = l_c.KJD;
    KD = l_c.KD;
    KLAT = l_c.KLAT;
    KLON = l_c.KLON;
    KCAC = l_c.KCAC;
    KEFF = l_c.KEFF;
    sample_type = l_c.sample_type;
    dt = con_cyc;
}
std::priority_queue<Frenet_path, std::vector<Frenet_path>, CompareCost> Frenet_plan::calc_frenet_paths(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0){
    std::priority_queue<Frenet_path, std::vector<Frenet_path>, CompareCost> frenet_paths;
    //  generate path to each offset goal
    for(double di = 0.0 - MAX_ROAD_WIDTH; di < MAX_ROAD_WIDTH; di += D_ROAD_W){
//        std::cout << "start path\n";
        //  Lateral motion planning
        for(double Ti = MINT; Ti < MAXT; Ti += 2){
            Frenet_path fp = Frenet_path();
            fp.t.resize(static_cast<int> (ceil(Ti/DT)));

            fp.d.resize(static_cast<int> (ceil(Ti/DT)));
            fp.d_d.resize(static_cast<int> (ceil(Ti/DT)));
            fp.d_dd.resize(static_cast<int> (ceil(Ti/DT)));
            fp.d_ddd.resize(static_cast<int> (ceil(Ti/DT)));
            fp.s.resize(static_cast<int> (ceil(Ti/DT)));
            fp.s_d.resize(static_cast<int> (ceil(Ti/DT)));
            fp.s_dd.resize(static_cast<int> (ceil(Ti/DT)));
            fp.s_ddd.resize(static_cast<int> (ceil(Ti/DT)));

            auto lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti);
            for(auto i_t = std::make_pair(0, 0.0); i_t.first < static_cast<int>(ceil(Ti/DT)) && i_t.second < Ti; ++i_t.first, i_t.second += DT) {
                fp.t(i_t.first) = i_t.second;
            }
			///
			//std::cout << "Ti / DT: " << Ti / DT << std::endl;
			//std::cout << "ceil of Ti / DT:" << ceil(Ti / DT) << std::endl;
			//std::cout << "fp.t.size: " << fp.t.size() << std::endl;
			//std::cout << std::endl;
			///
            for(int i = 0; i < fp.t.size(); ++i)
                fp.d(i) = lat_qp.calc_point(fp.t(i));
            for(int i = 0; i < fp.t.size(); ++i)
                fp.d_d(i) = lat_qp.calc_first_derivative(fp.t(i));
            for(int i = 0; i < fp.t.size(); ++i)
                fp.d_dd(i) = lat_qp.calc_second_derivative(fp.t(i));
            for(int i = 0; i < fp.t.size(); ++i)
                fp.d_ddd(i) = lat_qp.calc_third_derivative(fp.t(i));

            // Longitudinal motion planning (Velocity keeping)
            for(double tv = SMALL_SPEED; tv <= TARGET_SPEED ; tv += D_T_S){
                auto tfp = fp;
                auto lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti);
                //std::cout << "fp.t.size(): " << fp.t.size() << std::endl;
                for(int i = 0; i < fp.t.size(); ++i)
                    tfp.s(i) = lon_qp.calc_point(fp.t(i));
                for(int i = 0; i < fp.t.size(); ++i)
                    tfp.s_d(i) = lon_qp.calc_first_derivative(fp.t(i));
                for(int i = 0; i < fp.t.size(); ++i)
                    tfp.s_dd(i) = lon_qp.calc_second_derivative(fp.t(i));
                for(int i = 0; i < fp.t.size(); ++i)
                    tfp.s_ddd(i) = lon_qp.calc_third_derivative(fp.t(i));

                auto Jp = (tfp.d_ddd.array() * tfp.d_ddd.array()).matrix().sum();
                auto Js = (tfp.s_ddd.array() * tfp.s_ddd.array()).matrix().sum();
                // square of diff from target speed
                auto ds = pow((TARGET_SPEED - tfp.s_d[tfp.s_d.size() - 1]),2);
                
                tfp.cd = KJ * Jp + KT * Ti + KD * pow(tfp.d[tfp.d.size() -1], 2);
                tfp.cv = KJ * Js + KT * Ti + KD * ds;
                tfp.cf = KLAT * tfp.cd + KLON * tfp.cv;

                fp.lat_line.set_param(lat_qp.a0, lat_qp.a1, lat_qp.a2, lat_qp.a3, lat_qp.a4, lat_qp.a5);
                fp.lon_line_4.set_param(lon_qp.a0, lon_qp.a1, lon_qp.a2, lon_qp.a3, lon_qp.a4);

                frenet_paths.push(tfp);
            }
        }
    }
    return frenet_paths;
}
std::priority_queue<Frenet_path, std::vector<Frenet_path>, CompareCost> Frenet_plan::generate_frenet_paths_2(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0){
    std::priority_queue<Frenet_path, std::vector<Frenet_path>, CompareCost> frenet_paths;
    //  generate path to each offset goal
    std::vector<quintic_polynomial> lat_qp_path;
    for(double di = 0.0 - MAX_ROAD_WIDTH; di < MAX_ROAD_WIDTH; di += D_ROAD_W){
        for(int Si = MINS; Si <=  MAXS; Si +=  DS)
        {
            auto lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, s0, s0+Si);
            lat_qp_path.emplace_back(lat_qp);
        }
    }
    for(double tv = SMALL_SPEED; tv <= TARGET_SPEED ; tv += D_T_S){
        // (Velocity keeping)
        for(double Ti = MINT; Ti <= MAXT; Ti += 2){
            
            Frenet_path fp = Frenet_path();
            fp.sample_type = SampleType::Cruise;  
            fp.t.resize(static_cast<int> (ceil(Ti/DT)));

            fp.d.resize(static_cast<int> (ceil(Ti/DT)));
            fp.d_d.resize(static_cast<int> (ceil(Ti/DT)));
            fp.d_dd.resize(static_cast<int> (ceil(Ti/DT)));
            fp.d_ddd.resize(static_cast<int> (ceil(Ti/DT)));
            fp.s.resize(static_cast<int> (ceil(Ti/DT)));
            fp.s_d.resize(static_cast<int> (ceil(Ti/DT)));
            fp.s_dd.resize(static_cast<int> (ceil(Ti/DT)));
            fp.s_ddd.resize(static_cast<int> (ceil(Ti/DT)));

            for(auto i_t = std::make_pair(0, 0.0); i_t.first < static_cast<int>(ceil(Ti/DT)) && i_t.second < Ti; ++i_t.first, i_t.second += DT) {
                fp.t(i_t.first) = i_t.second;
            }
            auto lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti);

            for(int j = 0; j < lat_qp_path.size(); j++)
            {
                for(int i = 0; i < fp.t.size(); ++i){
                    fp.s(i) = lon_qp.calc_point(fp.t(i));
                    fp.d(i) = lat_qp_path[j].calc_point(fp.s(i));
                
                    fp.s_d(i) = lon_qp.calc_first_derivative(fp.t(i));
                    fp.d_d(i) = lat_qp_path[j].calc_first_derivative(fp.s(i));
                
                    fp.s_dd(i) = lon_qp.calc_second_derivative(fp.t(i));
                    fp.d_dd(i) = lat_qp_path[j].calc_second_derivative(fp.s(i));
                
                    fp.s_ddd(i) = lon_qp.calc_third_derivative(fp.t(i));   
                    fp.d_ddd(i) = lat_qp_path[j].calc_third_derivative(fp.s(i)); 


                    fp.lat_line.set_param(lat_qp_path[j].a0, lat_qp_path[j].a1, lat_qp_path[j].a2, lat_qp_path[j].a3, lat_qp_path[j].a4, lat_qp_path[j].a5);
                    fp.lon_line_4.set_param(lon_qp.a0, lon_qp.a1, lon_qp.a2, lon_qp.a3, lon_qp.a4);
                }    

                    auto Jp = (fp.d_ddd.array() * fp.d_ddd.array()).matrix().sum();
                    auto Js = (fp.s_ddd.array() * fp.s_ddd.array()).matrix().sum();
                    // square of diff from target speed
                    auto ds = pow((TARGET_SPEED - fp.s_d[fp.s_d.size() - 1]),2);

                    fp.cd = KJ * Jp + KT * Ti + KD * pow(fp.d[fp.d.size() -1], 2);
                    fp.cv = KJ * Js + KT * Ti + KD * ds;
                    fp.cf = KLAT * fp.cd + KLON * fp.cv;
                    
                    
              
                    frenet_paths.push(fp);
            }
        }
    }
    return frenet_paths;
}
std::priority_queue<Frenet_path, std::vector<Frenet_path>, CompareCost> Frenet_plan::generate_frenet_paths(double c_speed, double c_d, double c_d_d, double c_d_dd, double s0){
    std::priority_queue<Frenet_path, std::vector<Frenet_path>, CompareCost> frenet_paths;
    //  generate path to each offset goal
    
    for(double di = 0.0 - MAX_ROAD_WIDTH; di <= MAX_ROAD_WIDTH; di += D_ROAD_W){

        for(double tv = SMALL_SPEED; tv <= TARGET_SPEED ; tv += D_T_S){           

            // (Velocity keeping)
            for(double Ti = MINT; Ti <= MAXT; Ti += 1){
                
                Frenet_path fp = Frenet_path();

                fp.sample_type = SampleType::Cruise;  

                fp.t.resize(static_cast<int> (ceil(Ti/DT)));

                fp.d.resize(static_cast<int> (ceil(Ti/DT)));
                fp.d_d.resize(static_cast<int> (ceil(Ti/DT)));
                fp.d_dd.resize(static_cast<int> (ceil(Ti/DT)));
                fp.d_ddd.resize(static_cast<int> (ceil(Ti/DT)));
                fp.s.resize(static_cast<int> (ceil(Ti/DT)));
                fp.s_d.resize(static_cast<int> (ceil(Ti/DT)));
                fp.s_dd.resize(static_cast<int> (ceil(Ti/DT)));
                fp.s_ddd.resize(static_cast<int> (ceil(Ti/DT)));
                              
                for(auto i_t = std::make_pair(0, 0.0); i_t.first < static_cast<int>(ceil(Ti/DT)) && i_t.second < Ti; ++i_t.first, i_t.second += DT) {
                    fp.t(i_t.first) = i_t.second;
                }
                auto lon_qp = quartic_polynomial(s0, c_speed, 0.0, tv, 0.0, Ti);
                // std::cout << "fp.t.size(): " << fp.t.size() << std::endl;
                // double s_end = fp.s(fp.t.size()-1);
                // double s_end = lon_qp.calc_point(fp.t(fp.t.size()-1));
                double s_end = lon_qp.calc_point(Ti);

                auto lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, s0, s_end);

                for(int i = 0; i < fp.t.size(); ++i)
                {
                    double ss = lon_qp.calc_point(fp.t(i));
                    fp.s(i) = ss;
                    fp.s_d(i) = lon_qp.calc_first_derivative(fp.t(i));
                    fp.s_dd(i) = lon_qp.calc_second_derivative(fp.t(i));
                    fp.s_ddd(i) = lon_qp.calc_third_derivative(fp.t(i));

                    fp.d(i) = lat_qp.calc_point(ss);
                    fp.d_d(i) = lat_qp.calc_first_derivative(ss);
                    fp.d_dd(i) = lat_qp.calc_second_derivative(ss);
                    fp.d_ddd(i) = lat_qp.calc_third_derivative(ss);
                }
                
                auto Jp = (fp.d_ddd.array() * fp.d_ddd.array()).matrix().sum();
                auto Js = (fp.s_ddd.array() * fp.s_ddd.array()).matrix().sum();
                auto Jd = (fp.d.array() * fp.d.array()).matrix().sum();

                // auto c_centripetal_acc = KCAC * (fp.d_d.array().abs() * fp.s_d.array().abs()).matrix().sum();
                auto c_eff = KEFF * pow((MAX_ACCEL - fp.s_dd.array().abs().maxCoeff()),2);
                // square of diff from target speed
                auto ds = pow((TARGET_SPEED - fp.s_d[fp.s_d.size() - 1]),2);
                auto dd = pow(fp.d[fp.d.size() -1], 2);

                fp.cd = KJ * Jp + KT * Ti + KJD * Jd + KD * dd;
                fp.cv = KJ * Js + KT * Ti + KD * ds;
                fp.cf = KLAT * fp.cd + KLON * fp.cv;
                fp.cf = KLAT * fp.cd + KLON * fp.cv + c_eff;

                fp.lat_line.set_param(lat_qp.a0, lat_qp.a1, lat_qp.a2, lat_qp.a3, lat_qp.a4, lat_qp.a5);
                fp.lon_line_4.set_param(lon_qp.a0, lon_qp.a1, lon_qp.a2, lon_qp.a3, lon_qp.a4);
                
                frenet_paths.push(fp);
            }
        }
    }
    return frenet_paths;
}
std::priority_queue<Frenet_path, std::vector<Frenet_path>, CompareCost> Frenet_plan::path_sample(Position start_point)
{
    if(sample_type == 0)
    {
        return this->generate_frenet_paths(start_point.s_d, start_point.l, start_point.l_ds, start_point.l_dds, start_point.s);
        // return this->generate_frenet_paths_2(start_point.s_d, start_point.l, start_point.l_d, start_point.l_dd, start_point.s);

    }else if(sample_type == 1)
    {
        return this->calc_frenet_paths(start_point.s_d, start_point.l, start_point.l_d, start_point.l_dd, start_point.s);
    }
    return this->generate_frenet_paths(start_point.s_d, start_point.l, start_point.l_ds, start_point.l_dds, start_point.s);
}
bool Frenet_plan::calc_global_paths(Frenet_path &fp, Spline2D &csp){
        std::vector<double> fpx;
        std::vector<double> fpy;
        std::vector<double> fpyaw;
        std::vector<double> fpc;
        std::vector<double> fpv;
        std::vector<double> fpa;
        for(int i = 0; i < fp.s.size(); ++i){   
            if(fp.s(i) < fp.s[0] || fp.s(i) > fp.s[fp.s.size() - 1])  
                 LOG_INFO("fp.s(i): {}", fp.s(i));

            auto ixy_pair = csp.CalcPosition(fp.s(i));
            if(std::isnan(ixy_pair.first)){
				return false;
            }
            auto iyaw = csp.CalcYaw(fp.s(i));   
                     
            auto fx = ixy_pair.first + fp.d(i) * cos(iyaw + M_PI / 2.0);
            auto fy = ixy_pair.second + fp.d(i) * sin(iyaw + M_PI / 2.0);
            fpx.push_back(fx);
            fpy.push_back(fy);
            if(this->sample_type == 0)
            {
                auto icur = csp.CalcCurvature(fp.s(i));
                auto dyaw = atan2(fp.d_d(i), 1 - icur * fp.d(i));            
                auto fyaw = norm_angle(dyaw + iyaw);
                double dkappa = 0.0;
                auto Krd = 1 - icur * fp.d(i);
                auto fc = ((fp.d_dd(i) + (icur * fp.d_d(i) + dkappa * fp.d(i)) * tan(dyaw)) * cos(dyaw) * cos(dyaw) / Krd + icur) * cos(dyaw) / Krd;
                auto fv = std::hypot(fp.s_d(i) * Krd , fp.s_d(i) * fp.d_d(i));
                auto fa = fp.s_dd(i) * Krd / cos(dyaw) + 
                            fp.s_d(i) * fp.s_d(i) / cos(dyaw) * (fp.d_d(i) * (fc * Krd / cos(dyaw) - icur) - 
                        icur * fp.d_d(i) - dkappa * fp.d(i));            
                fpyaw.push_back(fyaw);
                fpc.push_back(fc);
                fpv.push_back(fv);
                fpa.push_back(fa);
            }else{

                auto icur = csp.CalcCurvature(fp.s(i));
                double dl_p;
                if(fp.s_d(i) < 1e-5)
                {
                    dl_p = 0;
                }else{
                    dl_p = fp.d_d(i) / fp.s_d(i);
                }
                
                auto dyaw = atan2(dl_p, 1 - icur * fp.d(i));     

                auto fyaw = norm_angle(dyaw + iyaw);
                double dkappa = 0.0;
                auto Krd = 1 - icur * fp.d(i);
                auto fc = ((fp.d_dd(i) + (icur * dl_p + dkappa * fp.d(i)) * tan(dyaw)) * cos(dyaw) * cos(dyaw) / Krd + icur) * cos(dyaw) / Krd;
                auto fv = std::hypot(fp.s_d(i) * Krd , fp.d_d(i));
                auto fa = fp.s_dd(i) * Krd / cos(dyaw) + 
                            fp.s_d(i) * fp.s_d(i) / cos(dyaw) * (dl_p * (fc * Krd / cos(dyaw) - icur) - 
                        icur * dl_p - dkappa * fp.d(i));            
                fpyaw.push_back(fyaw);
                fpc.push_back(fc);
                fpv.push_back(fv);
                fpa.push_back(fa);
            }
            

        }
        fp.x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(fpx.data(), fpx.size());
        fp.y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(fpy.data(), fpy.size());
        
        fp.yaw = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(fpyaw.data(), fpyaw.size());
        fp.c = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(fpc.data(), fpc.size());
        fp.v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(fpv.data(), fpv.size());
        fp.a = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(fpa.data(), fpa.size());
        return true;
        
        // calc yaw and ds
    //     fp.v.resize(fp.x.size());
    //     fp.a.resize(fp.x.size());
    //     for(int i = 0;i<fp.x.size();i++)
    //     {
    //         fp.v(i) = std::hypot(fp.s_d(i),fp.d_d(i));
    //         fp.a(i) = std::hypot(fp.s_dd(i),fp.d_dd(i));
    //     }
    //     fp.yaw.resize(fp.x.size());
    //     fp.ds.resize(fp.x.size());
    //     fp.c.resize(fp.x.size()); 
    //     std::vector<double> dx;
    //     std::vector<double> dy;
    //     dx.resize(fp.x.size()-1,0);
    //     dy.resize(fp.x.size()-1,0);
    //     for(int i = 0;i<fp.x.size()-1;i++)
    //     {
    //         dx[i] = fp.x(i + 1) - fp.x(i);
    //         dy[i] = fp.y(i + 1) - fp.y(i);
    //     }
    //     std::vector<double> dx_1 = dx;
    //     std::vector<double> dx_2 = dx;
    //     std::vector<double> dy_1 = dy;
    //     std::vector<double> dy_2 = dy;
    //     dx_1.insert(dx_1.begin(), dx.front());
    //     dx_2.push_back(dx.back());
    //     dy_1.insert(dy_1.begin(), dy.front());
    //     dy_2.push_back(dy.back()); 
    //     double dx_f,dy_f;
    //     for(int i = 0;i<fp.x.size();i++)
    //     {
    //         dx_f = (dx_1[i] + dx_2[i])/2;
    //         dy_f = (dy_1[i] + dy_2[i])/2;       
    //         fp.yaw(i) = atan2(dy_f,dx_f);
    //         fp.ds(i) = std::hypot(dx_f,dy_f);
    //     }
    //     std::vector<double> dphi;
    //     for(int i = 0;i<fp.x.size()-1;i++)
    //     {
    //         dphi.push_back(fp.yaw(i+1) - fp.yaw(i));
    //     }
    //     std::vector<double> dphi_1 = dphi;
    //     std::vector<double> dphi_2 = dphi;
    //     dphi_1.insert(dphi_1.begin(),dphi.front());
    //     dphi_2.push_back(dphi.back());
    //     double dphi_f;
    //     for(int i = 0;i<fp.x.size();i++)
    //     {
    //         dphi_f = (dphi_1[i] + dphi_2[i]) / 2;
    //         fp.c(i) = norm_angle(dphi_f) / fp.ds(i);            
    //     }      
    // return true;
}

bool Frenet_plan::check_collision(Frenet_path& fp, ObstacleGroup obstacle_group){
    
    //找到路径上距离障碍物最近的点，判断这个点有没有和障碍物发生碰撞
    std::vector<int> closest_point_index(obstacle_group.size(),0);
    std::vector<float> closest_distance(obstacle_group.size(),1000000.0);
    float temp_d;
    for(int i = 0;i < fp.x.size();i++)
    {
        for(int j = 0; j < obstacle_group.size();j++)
        {
            temp_d = std::hypot(fp.x(i) - obstacle_group[j].center.x, fp.y(i) - obstacle_group[j].center.y);
            if(temp_d < closest_distance[j])
            {
                closest_distance[j] = temp_d;
                closest_point_index[j] = i;
            }
        }
    }
    OBB car;
    for(int k = 0;k<obstacle_group.size();k++)
    {
        car.center.x = fp.x(closest_point_index[k]);
        car.center.y = fp.y(closest_point_index[k]);
        car.rotation = fp.yaw(closest_point_index[k]);
        car.halfExtents.x = 1.1;
        car.halfExtents.y = 0.9;
        if(obb_check->isOBBColliding(car,obstacle_group[k]))
        {
            LOG_INFO("OBB Collision Detected!");
            return false;//发生碰撞,检测不通过
        }
    }
    
    return true;//没有发生碰撞，检测通过

}

bool Frenet_plan::check_paths(Frenet_path &fp, LocalTrajectory left_bound, 
    LocalTrajectory right_bound, ObstacleGroup obstacle_group){    
    
    if(fp.s_d.array().abs().maxCoeff()> MAX_SPEED)
        return false;
    // if(fp.d_d.maxCoeff()> MAX_SPEED_LAT)
    //     return false;
    if(fp.s_dd.array().abs().maxCoeff() > MAX_ACCEL)
        return false;    
    if(fp.c.array().abs().maxCoeff() > MAX_CURVATURE)
        return false;
    if(!check_collision(fp, obstacle_group))
        return false;
    if(!check_over_bound(fp, left_bound, right_bound))
        return false;
    return true;
}
bool Frenet_plan::check_over_bound(Frenet_path &fp, LocalTrajectory left_bound, LocalTrajectory right_bound)
{
    OBB temp_point;
    for(int i = 0;i<fp.x.size();i++)
    {
        temp_point.center.x = fp.x(i);
        temp_point.center.y = fp.y(i);
        temp_point.rotation = fp.yaw(i);
        temp_point.halfExtents.x = 1.1;//车半长
        temp_point.halfExtents.y = 0.9;//车半宽
        if(obb_check->isVehicleCrossingLaneBoundary(temp_point,left_bound))
        {
            return false;//越界，检查没有通过
        }
        if(obb_check->isVehicleCrossingLaneBoundary(temp_point,right_bound))
        {
            return false;//越界，检查没有通过
        }
    }
    
    return true;
}

LocalTrajectory Frenet_plan::run(FrenetPathPoint s_p,LocalTrajectory ref_line,LocalTrajectory left_bound, 
    LocalTrajectory right_bound, ObstacleGroup obstacle_group)
{
    std::vector<double> x, y;
    x.resize(ref_line.size());
    y.resize(ref_line.size());
    for (size_t i = 0; i < ref_line.size(); i++)
    {
        x.at(i) = ref_line[i].x;
        y.at(i) = ref_line[i].y;
    }   
    double t1 = nowSeconds();
    auto path = generate_target_course(x,y);
    auto rx = std::get<0>(path);
    auto ry = std::get<1>(path);
    smooth_path.resize(rx.size());
    for(int i = 0;i<rx.size();i++)
    {
       smooth_path.at(i).x =  rx[i];
       smooth_path.at(i).y =  ry[i];
    }
    LOG_INFO("generate_target_course time : {}", (nowSeconds() - t1));
    auto start_point = frenet_change::CalcFrenet(s_p,path);
    if(std::isnan(start_point.s_d) || std::isnan(start_point.l) || std::isnan(start_point.l_ds) ||
        std::isnan(start_point.l_dds) || std::isnan(start_point.s))
        {
            LocalTrajectory t_path;
            t_path.clear();
            LOG_ERROR("start point is nan!");
            return t_path;
        }
    LOG_INFO("start_point.s_d: {}, start_point.l: {}, start_point.l_ds: {}, start_point.l_dds: {}, start_point.s: {}",
        start_point.s_d,
        start_point.l,
        start_point.l_ds,
        start_point.l_dds,
        start_point.s);
    LOG_INFO("frenet_change time : {}", (nowSeconds() - t1));
    auto csp = std::get<4>(path);

    return frenet_optimal_planning(csp, start_point,left_bound,right_bound,obstacle_group);
}
LocalTrajectory Frenet_plan::frenet_optimal_planning(Spline2D& csp,Position start_point,
    LocalTrajectory left_bound, LocalTrajectory right_bound, ObstacleGroup obstacle_group){
    double t1 = nowSeconds();
    // auto fplist = this->generate_frenet_paths_2(start_point.s_d, start_point.l, start_point.l_ds, start_point.l_dds, start_point.s);   
    auto fplist = this->path_sample(start_point);
    double generate_frenet_paths_time = nowSeconds();
    LOG_INFO("generate_frenet_paths time : {}", (nowSeconds() - t1));

    auto op_path = get_optimal_path(fplist, csp, left_bound, right_bound, obstacle_group);
    double get_optimal_path_time = nowSeconds();
    LOG_INFO("get_optimal_path time : {}", (nowSeconds() - generate_frenet_paths_time));

    LocalTrajectory result_path;
    Position temp_point;
    if(op_path.valid == true)
    {
        for(int i = 0 ;i < op_path.x.size();i++)
        {
            temp_point.time = start_point.time + i * 0.02 * 1e6;
            temp_point.x = op_path.x(i);
            temp_point.y = op_path.y(i);
            temp_point.yaw = op_path.yaw(i);
            temp_point.cur = op_path.c(i);
            temp_point.v = op_path.v(i);
            temp_point.a = op_path.a(i);
            result_path.emplace_back(temp_point);
        }
    }else
    {
        LOG_ERROR("NO valid path!");
        result_path.clear();
    }
    LOG_INFO("result_path time : {}", (nowSeconds() - get_optimal_path_time));
    return result_path;
}
Frenet_path Frenet_plan::trajectory_interp(Frenet_path &path)
{
    int interp_point_number = DT/dt;
    Frenet_path temp_path;
    int path_length = (path.t.size()-1) * (DT / dt);
    std::vector<double> ps;
    std::vector<double> ps_d;
    std::vector<double> ps_dd;
    std::vector<double> pd;
    std::vector<double> pd_d;
    std::vector<double> pd_dd;    
    for(int i = 0; i < path.t.size()-1; i++)
    {

        for(int j = 0; j < interp_point_number; j++)
        {
            double t = path.t(i) + j * dt;   
            if(this->sample_type == 0)
            {                        
                if(path.sample_type == SampleType::Cruise)
                {
                    double temp_s = path.lon_line_4.calc_point(t);
                    ps.emplace_back(temp_s);
                    ps_d.emplace_back(path.lon_line_4.calc_first_derivative(t));
                    ps_dd.emplace_back(path.lon_line_4.calc_second_derivative(t));
                    pd.emplace_back(path.lat_line.calc_point(temp_s));
                    pd_d.emplace_back(path.lat_line.calc_first_derivative(temp_s));
                    pd_dd.emplace_back(path.lat_line.calc_second_derivative(temp_s));
                }else{
                    double temp_s = path.lon_line_5.calc_point(t);
                    ps.emplace_back(temp_s);
                    ps_d.emplace_back(path.lon_line_5.calc_first_derivative(t));
                    ps_dd.emplace_back(path.lon_line_5.calc_second_derivative(t));
                    pd.emplace_back(path.lat_line.calc_point(temp_s));
                    pd_d.emplace_back(path.lat_line.calc_first_derivative(temp_s));
                    pd_dd.emplace_back(path.lat_line.calc_second_derivative(temp_s));
                }
            }else{
                ps.emplace_back(path.lon_line_4.calc_point(t));
                ps_d.emplace_back(path.lon_line_4.calc_first_derivative(t));
                ps_dd.emplace_back(path.lon_line_4.calc_second_derivative(t));
                pd.emplace_back(path.lat_line.calc_point(t));
                pd_d.emplace_back(path.lat_line.calc_first_derivative(t));
                pd_dd.emplace_back(path.lat_line.calc_second_derivative(t));
            }


        }
    }
    temp_path.s = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ps.data(), ps.size());
    temp_path.s_d = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ps_d.data(), ps_dd.size());
    temp_path.s_dd = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ps_dd.data(), ps_dd.size());
    temp_path.d = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(pd.data(), pd.size());
    temp_path.d_d = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(pd_d.data(), pd_d.size());
    temp_path.d_dd = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(pd_dd.data(), pd_dd.size());
    return temp_path;
}
Frenet_path Frenet_plan::get_optimal_path(std::priority_queue<Frenet_path, std::vector<Frenet_path>, CompareCost> fplist, Spline2D csp,
    LocalTrajectory left_bound, LocalTrajectory right_bound, ObstacleGroup obstacle_group)
    {
        auto bestpath = Frenet_path();        
        while (!fplist.empty())
        {
            auto path = fplist.top();
            double t1 = nowSeconds();
            path = trajectory_interp(path);
            if(calc_global_paths(path, csp))
            {
                double calc_global_paths_time = nowSeconds();
                if(check_paths(path, left_bound, right_bound, obstacle_group))
                {
                    bestpath  =  path;
                    LOG_INFO("get optimal_path!");
                    break;
                }
            }else{
                bestpath.valid = false;
                break;
            }
            
            fplist.pop();
        }
        if(fplist.empty())
        {
            bestpath.valid = false;
        }
        return bestpath;
                
    }
std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, Spline2D>
Frenet_plan::generate_target_course(std::vector<double> &x, std::vector<double> &y){
    double t1 = nowSeconds();
    auto csp = Spline2D(x, y);
    LOG_INFO("Spline2D time : {}", (nowSeconds() - t1));
    std::vector<double> rx;
    std::vector<double>ry;
    std::vector<double> ryaw;
    std::vector<double> rk;
    // std::cout << csp.s[csp.s.size() - 1] << std::endl;
    for(double i_s = 0; i_s < csp.s.at(csp.s.size() - 1); i_s += 0.2){
        auto ixy = csp.CalcPosition(i_s);
        auto ix = ixy.first;
        auto iy = ixy.second;
        rx.push_back(ix);
        ry.push_back(iy);
        ryaw.push_back(csp.CalcYaw(i_s));
        rk.push_back(csp.CalcCurvature(i_s));
    }
    return std::make_tuple(rx, ry, ryaw, rk, csp);
}
int Frenet_plan::ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

    double closestLen = std::numeric_limits<double>::max()/ 3.0; //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
//        double map_x = maps_x[i];
//        double map_y = maps_y[i];
        double dist = sqrt(pow(x - maps_x[i], 2) + pow(y - maps_y[i], 2));
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;

}
int Frenet_plan::NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    /// TODO: theta is the same with value using derivative
    theta = atan2(map_y - maps_y[closestWaypoint > 0? closestWaypoint - 1: maps_y.size() - 1], map_x - maps_x[closestWaypoint > 0? closestWaypoint - 1: maps_x.size() - 1]);

    double angle = fabs(theta-heading);
    angle = std::min(2*M_PI - angle, angle);

    if(angle > M_PI/4)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }
    return closestWaypoint;
}

std::pair<double, double> Frenet_plan::getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = (n_y * x_x - n_x * x_y) / sqrt(pow(n_x, 2) + pow(n_y, 2));

    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += sqrt(pow(maps_x[i] - maps_x[i+1], 2) + pow(maps_y[i] - maps_y[i+1], 2));
    }

    frenet_s += sqrt(pow(proj_x, 2) + pow(proj_y, 2));

    return std::make_pair(frenet_s,frenet_d);

}


// quintic_polynomial class
quintic_polynomial::quintic_polynomial(){
    this->a0 = 0.0;
    this->a1 = 0.0;
    this->a2 = 0.0;
    this->a3 = 0.0;
    this->a4 = 0.0;
    this->a5 = 0.0;
}
quintic_polynomial::~quintic_polynomial(){}
quintic_polynomial::quintic_polynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double start_T, double end_T)
{
    this->xs = xs;
    this->vxs = vxs;
    this->axs = axs;
    this->xe = xe;
    this->vxe = vxe;
    this->axe = axe;
    
    MatrixXd A(6,6);
    A << pow(start_T,0), pow(start_T,1), pow(start_T, 2), pow(start_T,3), pow(start_T,4), pow(start_T, 5),
         0, pow(start_T,0), 2*pow(start_T, 1), 3*pow(start_T,2), 4*pow(start_T,3), 5*pow(start_T, 4),
         0, 0, 2*pow(start_T, 0), 2*3*pow(start_T,1), 3*4*pow(start_T,2), 4*5*pow(start_T, 3),
         pow(end_T,0), pow(end_T,1), pow(end_T, 2), pow(end_T,3), pow(end_T,4), pow(end_T, 5),
         0, pow(end_T,0), 2*pow(end_T, 1), 3*pow(end_T,2), 4*pow(end_T,3), 5*pow(end_T, 4),
         0, 0, 2*pow(end_T, 0), 2*3*pow(end_T,1), 3*4*pow(end_T,2), 4*5*pow(end_T, 3);
    VectorXd b(6);
    b << xs, vxs, axs, xe, vxe, axe;
    VectorXd x = A.colPivHouseholderQr().solve(b);
    this->a0 = x(0);
    this->a1 = x(1);
    this->a2 = x(2);
    this->a3 = x(3);
    this->a4 = x(4);
    this->a5 = x(5);

}   
void quintic_polynomial::set_param(double b1, double b2, double b3, double b4, double b5, double b6)
{
    this->a0 = b1;
    this->a1 = b2;
    this->a2 = b3;
    this->a3 = b4;
    this->a4 = b5;
    this->a5 = b6;
}
quintic_polynomial::quintic_polynomial(double xs, double vxs, double axs, double xe, double vxe, double axe, double T){
    // calc coefficient of quintic polynomial
    this->xs = xs;
    this->vxs = vxs;
    this->axs = axs;
    this->xe = xe;
    this->vxe = vxe;
    this->axe = axe;

    this->a0 = xs;
    this->a1 = vxs;
    this->a2 = axs / 2.0;

    MatrixXd A(3,3);
    A << pow(T,3), pow(T,4), pow(T, 5),
    3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4),
    6 * T, 12 * pow(T, 2), 20 * pow(T, 3);
//    std::cout << "A: " << std::endl << A << std::endl;
//    std::cout << "A.size(): " << A.rows() << ",  " << A.cols() << std::endl;

    VectorXd b(3);
    b << xe - this->a0 - this->a1 * T - this->a2 * pow(T, 2),
            vxe - this->a1 - this->a2 * T * 2,
            axe - this->a2 * 2;
//    std::cout << "b: " << std::endl << b << std::endl;
//    std::cout << "b size(): " << b.size() << std::endl;
//    if(A.determinant() != 0)
//        std::cout << "A is nonsingular\n";
    VectorXd x = A.colPivHouseholderQr().solve(b);

    this->a3 = x(0);
    this->a4 = x(1);
    this->a5 = x(2);
}

double quintic_polynomial::calc_point(double t){
    auto xt = this->a0 + this->a1 * t + this->a2 * pow(t, 2) + this->a3 * pow(t, 3) + this->a4 * pow(t, 4) + this->a5 * pow(t, 5);
    return xt;
}

double quintic_polynomial::calc_first_derivative(double t) {
    auto xt = this->a1 + 2* this->a2 * t + 3 * this->a3 * pow(t, 2) + 4* this->a4 * pow(t, 3) + 5* this->a5 * pow(t, 4);
    return xt;
}

double quintic_polynomial::calc_second_derivative(double t){
    auto xt = 2* this->a2 + 6 * this->a3 * t + 12* this->a4 * pow(t, 2) + 20* this->a5 * pow(t, 3);
    return xt;
}

double quintic_polynomial::calc_third_derivative(double t){
    auto xt = 6 * this->a3 + 24* this->a4 * t + 60* this->a5 * pow(t, 2);
    return xt;
}


// quartic_polynomial class

quartic_polynomial::quartic_polynomial(){
    this->a0 = 0.0;
    this->a1 = 0.0;
    this->a2 = 0.0;
    this->a3 = 0.0;
    this->a4 = 0.0;
   
}
quartic_polynomial::~quartic_polynomial(){}

quartic_polynomial::quartic_polynomial(double xs, double vxs, double axs, double vxe, double axe, double T) {
    this->xs = xs;
    this->vxs = vxs;
    this->axs = axs;
    this->vxe = vxe;
    this->axe = axe;

    this->a0 = xs;
    this->a1 = vxs;
    this->a2 = axs / 2.0;

    MatrixXd A(2, 2);
    A << 3 * pow(T, 2), 4 * pow(T, 3),
            6 * T, 12 * pow(T, 2);

    VectorXd v(2);
    v << vxe - this->a1 - 2 * this->a2 * T,
            axe - 2 * this->a2;
    VectorXd x = A.colPivHouseholderQr().solve(v);
    this->a3 = x(0);
    this->a4 = x(1);
}
void quartic_polynomial::set_param(double b1, double b2, double b3, double b4, double b5)
{
    this->a0 = b1;
    this->a1 = b2;
    this->a2 = b3;
    this->a3 = b4;
    this->a4 = b5;
    
}
double quartic_polynomial::calc_point(double t){
        auto xt = this->a0 + this->a1 * t + this->a2 * pow(t, 2) + this->a3 * pow(t, 3) + this->a4 * pow(t, 4);
        return xt;
}

double quartic_polynomial::calc_first_derivative(double t){
    auto xt = this->a1 + 2 * this->a2 * t + 3 * this->a3 * pow(t, 2) + 4 * this->a4 * pow(t, 3);
    return xt;
}

double quartic_polynomial::calc_second_derivative(double t) {
    auto xt = 2 * this->a2 + 6 * this->a3 * t + 12 * this->a4 * pow(t, 2);
    return xt;
}

double quartic_polynomial::calc_third_derivative(double t) {
    auto xt = 6 * this->a3 + 24 * this->a4 * t;
    return xt;
}

Frenet_path::Frenet_path(){
    this->valid = true;
}
Frenet_path::~Frenet_path() {}

Frenet_path::Frenet_path(const Frenet_path& fp){
    this->t = fp.t;
    this->d = fp.d;
    this->d_d = fp.d_d;
    this->d_dd = fp.d_dd;
    this->d_ddd = fp.d_ddd;
    this->s = fp.s;
    this->s_d = fp.s_d;
    this->s_dd = fp.s_dd;
    this->s_ddd = fp.s_ddd;
    this->cd = fp.cd;
    this->cv = fp.cv;
    this->cf = fp.cf;

    this->x = fp.x;
    this->y = fp.y;
    this->yaw = fp.yaw;
    this->ds = fp.ds;
    this->c = fp.c;
    this->valid = fp.valid;

    this->lat_line = fp.lat_line;
    this->lon_line_4 = fp.lon_line_4;
    this->lon_line_5 = fp.lon_line_5;
    this->sample_type = fp.sample_type;
}
