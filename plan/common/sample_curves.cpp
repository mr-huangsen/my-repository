#include "common/sample_curves.h"
#include <Eigen/Sparse>

    // https://zhuanlan.zhihu.com/p/62860859

    // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
    Spline::Spline(std::vector<double> x, std::vector<double> y)
        : x_(x), y_(y), nx_(x.size()), h_(VecDiff(x)), a_(y) {
       
        Eigen::SparseMatrix<double> A = CalcA();
        Eigen::VectorXd B = CalcB();
        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(A);
        Eigen::VectorXd c_eigen = 0.5*solver.solve(B);
        double* c_pointer = c_eigen.data(); 
        c_.assign(c_pointer, c_pointer + c_eigen.rows());
        for (int i = 0; i < nx_ - 1; i++) {
            d_.emplace_back((c_.at(i + 1) - c_.at(i)) / (3.0 * h_.at(i)));
            b_.emplace_back((a_.at(i+1) - a_.at(i)) / h_.at(i) -
                            h_.at(i) * (c_.at(i+1) + 2 * c_.at(i)) / 3.0);
        }
       
    }

    double Spline::Calc(double t) {
        // if(t<x_.front()){
        //     t = x_.front();
        // }
        if (t < x_.front() || t > x_.back()) {
            std::cout<<"Calc t: "<<t<<" x_.front: "<<x_.front()<<" x_.back():"<< x_.back()<<std::endl;
            // std::cout<<"Calc t: "<<t<<" x_.front: "<<x_.front()<<" x_.back():"<< x_.back()<<std::endl;
            throw std::invalid_argument("received value out of the pre-defined range in Calc");
        }
        int seg_id = Bisect(t, 0, nx_-1);
        double dx = t - x_.at(seg_id);
        return a_.at(seg_id)+b_.at(seg_id)*dx+c_.at(seg_id)*dx*dx+d_.at(seg_id)*dx*dx*dx;
    }

    double Spline::CalcD(double t) {
        //  if(t<x_.front()){
        //     t = x_.front();
        // }
        if (t < x_.front() || t > x_.back()) {
            std::cout<<"CalcD t: "<<t<<" x_.front: "<<x_.front()<<" x_.back():"<< x_.back()<<std::endl;
            throw std::invalid_argument("received value out of the pre-defined range in CalcD");
        }
        int seg_id = Bisect(t, 0, nx_-1);
        
        double dx = t - x_.at(seg_id);
        return b_.at(seg_id) + 2 * c_.at(seg_id) * dx + 3 * d_.at(seg_id) * dx * dx;
    }

    double Spline::CalcDD(double t) {
        //  if(t<x_.front()){
        //     t = x_.front();
        // }
        if (t < x_.front() || t > x_.back()) {
            // std::cout<<"CalcDD t: "<<t<<" x_.front: "<<x_.front()<<" x_.back():"<< x_.back()<<std::endl;
            throw std::invalid_argument("received value out of the pre-defined range in CalcDD");
        }
        int seg_id = Bisect(t, 0, nx_-1);
        double dx = t - x_.at(seg_id);
        return 2 * c_.at(seg_id) + 6 * d_.at(seg_id) * dx;
    }

    Eigen::SparseMatrix<double> Spline::CalcA() {

        Eigen::SparseMatrix<double> A(nx_,nx_);
        std::vector<Eigen::Triplet<double>> triple_list;
        for(int i=0;i<nx_-1;i++){
            triple_list.emplace_back(Eigen::Triplet<double>(i+1,i,h_.at(i)));
            triple_list.emplace_back(Eigen::Triplet<double>(i,i+1,h_.at(i)));
            if(i!=nx_-2){
                triple_list.emplace_back(Eigen::Triplet<double>(i+1,i+1,2*(h_.at(i) + h_.at(i+1))));
            }
        }
        triple_list.emplace_back(Eigen::Triplet<double>(0,0,1));
        triple_list.emplace_back(Eigen::Triplet<double>(nx_ - 1,nx_ - 1,1));
        A.setFromTriplets(triple_list.begin(),triple_list.end());
        return A;
    }

    Eigen::VectorXd Spline::CalcB() {
        Eigen::VectorXd B = Eigen::VectorXd::Zero(nx_);
        for (int i = 0; i < nx_ - 2; i++) {
            B(i + 1) = 6.0 * (a_.at(i + 2) - a_.at(i + 1)) / h_.at(i + 1) -
                        6.0 * (a_.at(i + 1) - a_.at(i)) / h_.at(i);
        }
        return B;
    }

    int Spline::Bisect(double t, int start, int end) {
        
        int mid = (start + end) / 2;
        if (t == x_.at(mid) ||  end-start ==1 ) { // left close and right close[]
            return mid;
        } 
        else if (t > x_.at(mid)) {
            return Bisect(t, mid, end);
        } 
        else {
            return Bisect(t, start, mid);
        }
    }


    Spline2D::Spline2D(std::vector<double> x, std::vector<double> y) {
        s = CalcS(x, y);
        sx = Spline(s, x);
        sy = Spline(s, y);
    }

    std::pair<double, double> Spline2D::CalcPosition(double s_t) {
        double x = sx.Calc(s_t);
        double y = sy.Calc(s_t);
        return std::make_pair(x,y);
    }

    double Spline2D::CalcCurvature(double s_t) {
        double dx = sx.CalcD(s_t);
        double ddx = sx.CalcDD(s_t);
        double dy = sy.CalcD(s_t);
        double ddy = sy.CalcDD(s_t);
        // https://zhuanlan.zhihu.com/p/427833623

        // return std::fabs((ddy * dx - ddx * dy)) / std::pow((dx * dx + dy * dy),1.5);
        return (ddy * dx - ddx * dy) / std::pow((dx * dx + dy * dy),1.5);
    }

    double Spline2D::CalcYaw(double s_t) {
        double dx = sx.CalcD(s_t);
        double dy = sy.CalcD(s_t);
        return std::atan2(dy, dx);
    }

    std::vector<double> Spline2D::CalcS(std::vector<double> x, std::vector<double> y) {
        std::vector<double> ds;
        std::vector<double> out_s{0};
        std::vector<double> dx = VecDiff(x);
        std::vector<double> dy = VecDiff(y);

        for (unsigned int i = 0; i < dx.size(); i++) {
            ds.push_back(std::sqrt(dx.at(i) * dx.at(i)  + dy.at(i)  * dy.at(i) ));
        }

        std::vector<double> cum_ds = CumSum(ds);
        out_s.insert(out_s.end(), cum_ds.begin(), cum_ds.end());
        return out_s;
  }



    QuarticPolynomial::QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe,double T)
                :x_s_(xs),vx_s_(vxs),ax_s_(axs),vx_e_(vxe),ax_e_(axe),
                a0_(xs),a1_(vxs),a2_(ax_s_ / 2.0) {
        Eigen::Matrix2f A;
        A << 3 * std::pow(T, 2), 4 * std::pow(T, 3), 6 * T, 12 * std::pow(T, 2);
        Eigen::Vector2f B;
        B << vx_e_ - a1_ - 2 * a2_ * T, ax_e_ - 2 * a2_;

        Eigen::Vector2f c_eigen = A.colPivHouseholderQr().solve(B);
        a3_ = c_eigen[0];
        a4_ = c_eigen[1];
    }

    double QuarticPolynomial::CalcPoint(double t) {
        return a0_ + a1_ * t + a2_ * std::pow(t, 2) + a3_ * std::pow(t, 3) +a4_ * std::pow(t, 4);
    }

    double QuarticPolynomial::CalcFirstDerivative(double t) {
        return a1_ + 2 * a2_ * t + 3 * a3_ * std::pow(t, 2) + 4 * a4_ * std::pow(t, 3);
    }

    double QuarticPolynomial::CalcSecondDerivative(double t) {
        return 2 * a2_ + 6 * a3_ * t + 12 * a4_ * std::pow(t, 2);
    }
    
    double QuarticPolynomial::CalcThirdDerivative(double t) { 
        return 6 * a3_ + 24 * a4_ * t; 
    }



    QuinticPolynomial::QuinticPolynomial(double xs, double vs, double as, double xe, double ve,
                    double ae, double Ts, double Te)
    {
        Eigen::MatrixXd A(6, 6);
        A << 1, Ts, std::pow(Ts, 2), std::pow(Ts, 3), std::pow(Ts, 4), std::pow(Ts, 5),
            0, 1, 2 * Ts, 3 * std::pow(Ts, 2), 4 * std::pow(Ts, 3), 5 * std::pow(Ts, 4),
            0, 0, 2, 6 * Ts, 12 * std::pow(Ts, 2), 20 * std::pow(Ts, 3),
            1, Te, std::pow(Te, 2), std::pow(Te, 3), std::pow(Te, 4), std::pow(Te, 5),
            0, 1, 2 * Te, 3 * std::pow(Te, 2), 4 * std::pow(Te, 3), 5 * std::pow(Te, 4),
            0, 0, 2, 6 * Te, 12 * std::pow(Te, 2), 20 * std::pow(Te, 3);

        Eigen::VectorXd B(6);
        B << xs, vs, as, xe, ve, ae;

        Eigen::VectorXd coeff(6);
        coeff = A.colPivHouseholderQr().solve(B);
        a0_ = coeff[0];
        a1_ = coeff[1];
        a2_ = coeff[2];
        a3_ = coeff[3];
        a4_ = coeff[4];
        a5_ = coeff[5];
    }

    double QuinticPolynomial::CalcPoint(double t){
        return a0_ + a1_ * t + a2_ * std::pow(t, 2) + a3_ * std::pow(t, 3) + a4_ * std::pow(t, 4) + a5_ * std::pow(t, 5);
    }

    double QuinticPolynomial::CalcFirstDerivative(double t){
        return a1_ + 2 * a2_ * t + 3 * a3_ * std::pow(t, 2) + 4 * a4_ * std::pow(t, 3) + 5 * a5_ * std::pow(t, 4);
    }

    double QuinticPolynomial::CalcSecondDerivative(double t){
        return 2 * a2_ + 6 * a3_ * t + 12 * a4_ * std::pow(t, 2) + 20 * a5_ * std::pow(t, 3);
    }

    double QuinticPolynomial::CalcThirdDerivative(double t)
    {
        return 6 * a3_ + 24 * a4_ * t + 60 * a5_ * std::pow(t, 2);
    }

