#ifndef SAMPLE_CURVES_H
#define SAMPLE_CURVES_H

#include <array>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <rclcpp/rclcpp.hpp>
#include "log/log.h"

inline std::vector<double> VecDiff(std::vector<double> input) {
    std::vector<double> output;
    for (unsigned int i = 1; i < input.size(); i++) {
        output.push_back(input[i] - input[i - 1]);
    }
    return output;
};

inline std::vector<double> CumSum(std::vector<double> input) {
    std::vector<double> output;
    double temp = 0;
    for (unsigned int i = 0; i < input.size(); i++) {
        temp += input[i];
        output.push_back(temp);
    }
    return output;
};

class Spline {
public:
    std::vector<double> x_;
    std::vector<double> y_;
    int nx_;
    std::vector<double> h_;
    std::vector<double> a_;
    std::vector<double> b_;
    std::vector<double> c_;
    std::vector<double> d_;
    Spline(){}
    // d_i * (x-x_i)^3 + c_i * (x-x_i)^2 + b_i * (x-x_i) + a_i
    Spline(std::vector<double> x, std::vector<double> y);

    double Calc(double t);

    double CalcD(double t);

    double CalcDD(double t);
private:
    Eigen::SparseMatrix<double> CalcA();

    Eigen::VectorXd CalcB();

    int Bisect(double t, int start, int end);
};

class Spline2D {
public:
    Spline sx;
    Spline sy;
    std::vector<double> s;
    Spline2D(){};
    ~Spline2D(){};
    Spline2D(std::vector<double> x, std::vector<double> y);

    std::pair<double, double> CalcPosition(double s_t);

    double CalcCurvature(double s_t);

    double CalcYaw(double s_t) ;
private:
    std::vector<double> CalcS(std::vector<double> x, std::vector<double> y);
};


class QuarticPolynomial {

public:
 
    double x_s_;
    double vx_s_;
    double ax_s_;
    double vx_e_;
    double ax_e_;
    double a0_, a1_, a2_, a3_, a4_;

    QuarticPolynomial(){};

    QuarticPolynomial(double xs, double vxs, double axs, double vxe, double axe,double T);

    double CalcPoint(double t);
    double CalcFirstDerivative(double t);
    double CalcSecondDerivative(double t);
    double CalcThirdDerivative(double t);
};


class QuinticPolynomial
{
public:
  // l = a0 + a1*s + a2*s^2 + a3*s^3 + a4*s^4 + a5*s^5
    double a0_;
    double a1_;
    double a2_;
    double a3_;
    double a4_;
    double a5_;

    QuinticPolynomial(){}

    QuinticPolynomial(double xs, double vs, double as, double xe, double ve,
                    double ae, double Ts, double Te);

    double CalcPoint(double t);
    double CalcFirstDerivative(double t);

    double CalcSecondDerivative(double t);
    double CalcThirdDerivative(double t);
};

#endif
