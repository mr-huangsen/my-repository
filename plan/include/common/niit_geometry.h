#ifndef NIIT_GEOMETRY_H
#define NIIT_GEOMETRY_H
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "log/log.h"
namespace NIIT_MATH{
    
    void Map2World(const nav_msgs::msg::MapMetaData& map_info, int mx, int my, double &wx, double &wy);
    bool World2Map(const nav_msgs::msg::MapMetaData& map_info,double wx, double wy, int &mx, int &my);

}
#endif
