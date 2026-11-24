#pragma once

#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <Eigen/Eigen>
#include "../point.h"
#include "common/sample_curves.h"

namespace frenet_change{

double CalcDist(double x1, double y1, double x2, double y2);

int AcquireMatchIdx(const double &cur_x, const double &cur_y,
                           const std::vector<FrenetPathPoint> &path,
                           int &pre_match_index, bool &IsNewPath);

CartesianPathPoint AcquireProjectPoint(const CartesianCarInfo &cur_pose,
                                   const CartesianPathPoint &match_point);

FrenetPathPoint Cartesian2Frenet(const CartesianCarInfo &global_point,
                                 const std::vector<FrenetPathPoint> &path,
                                const int match_index, const std::vector<double> &path_point_s);

std::vector<FrenetPathPoint> CalcFrenet(const std::vector<FrenetPathPoint> &target_path,
                            const std::vector<FrenetPathPoint> &ref_path);

std::vector<double> CalcS(const std::vector<FrenetPathPoint> &ref_path); 

FrenetPathPoint CalcFrenet(const CartesianCarInfo &global_point,
    const std::vector<FrenetPathPoint> &ref_path);

FrenetPathPoint Cartesian2FrenetOnlyPosition(const CartesianCarInfo &point,const std::vector<FrenetPathPoint> &path,
    const int match_index, const std::vector<double> path_point_s);

FrenetPathPoint CalcFrenet(const CartesianCarInfo &global_point,
    const std::tuple<std::vector<double>, std::vector<double>, std::vector<double>, std::vector<double>, Spline2D> &ref_path);

}
