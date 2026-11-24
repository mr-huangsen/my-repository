#include"common/differential_tracker.h"

double DifferentialTracker::work(double target_speed,double dt)
{    
    double temp_y = -1 * r * r *(last_output - target_speed) - 2 * r * diff_output;
    diff_output = diff_output + temp_y * dt;
    output = last_output + diff_output * dt;
    last_output = output;
    return output;
}
void DifferentialTracker::clear()
{
    last_output = 0;
    diff_output = 0;
    output = 0;
}