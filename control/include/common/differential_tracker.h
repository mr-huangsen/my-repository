#pragma once
#include<rclcpp/rclcpp.hpp>
#include <cmath>

struct DifferentialTracker
{
    double r;
    double last_output = 0;
    double diff_output = 0;
    double output = 0;
    double work(double target_input,double dt);
    DifferentialTracker(double r):r(r){};
    void clear();
};
