#include "common/niit_geometry.h"
namespace NIIT_MATH
{
void Map2World(const nav_msgs::msg::MapMetaData& map_info, int mx, int my, double &wx, double &wy)
{
    double map_theta = tf2::getYaw(map_info.origin.orientation);
    double sin_map_theta = std::sin(map_theta);
    double cos_map_theta = std::cos(map_theta);
    double tmpx = mx;
    double tmpy = my;
    mx = cos_map_theta*tmpx - sin_map_theta*tmpy;
    my = sin_map_theta*tmpx + cos_map_theta*tmpy;
    wx = map_info.origin.position.x + (mx + 0.5) * map_info.resolution;
    wy = map_info.origin.position.y + (my + 0.5) * map_info.resolution;
    return ;
}

bool World2Map(const nav_msgs::msg::MapMetaData& map_info,double wx, double wy, int &mx, int &my){
    
    double map_theta = tf2::getYaw(map_info.origin.orientation);
    double sin_map_theta = std::sin(map_theta);
    double cos_map_theta = std::cos(map_theta);

    double T_x = map_info.origin.position.x*cos_map_theta+sin_map_theta*map_info.origin.position.y;
    double T_y = -map_info.origin.position.x*sin_map_theta+cos_map_theta*map_info.origin.position.y;
   
    double x = cos_map_theta*wx+sin_map_theta*wy-T_x;
    double y = -sin_map_theta*wx+cos_map_theta*wy-T_y;
    mx = int(x/ map_info.resolution);
    my = int(y / map_info.resolution);

    if (mx < 0 || mx >= map_info.width || my < 0 ||my >= map_info.height)
    {
        return false;
    }
    return true;
}

}
