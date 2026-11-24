#pragma once
#include <iostream>
#include <cmath>
#include "point.h"
#include "log/log.h"
typedef Obstacle OBB;
typedef FrenetPathPoint Point;
// struct Point {
//     float x, y;
// };

// struct OBB {
//     Point center; // 中心点
//     Point halfExtents; // 半边长
//     float rotation; // 旋转角度，弧度
// };
class OBBType{
    public:
    OBBType(){};
    ~OBBType(){};
    // 向量点积
    float dotProduct(const Point& a, const Point& b); 

    // 获取OBB的四个顶点
    void getOBBVertices(const OBB& obb, Point vertices[4]);

    // 判断是否在投影轴上分离
    bool isSeparatedOnAxis(const Point& axis, const Point verticesA[4], const Point verticesB[4]);

    // OBB碰撞检测
    bool isOBBColliding(const OBB& a, const OBB& b);
    
    //OBB越界检测
    bool isVehicleCrossingLaneBoundary(const OBB& vehicle, 
                                  const std::vector<Point>& lane_boundary,
                                  double threshold = 0.1);
    double pointToSegmentDistance(const Point& p, const Point& a, const Point& b);

    bool segmentsIntersect(const Point& a1, const Point& a2, 
                       const Point& b1, const Point& b2);
};
// int main() {
//     OBB rect1 = {{0, 0}, {2, 1}, 0}; // OBB1，中心在(0,0)，半边长(2,1)，无旋转
//     OBB rect2 = {{3, 2}, {2, 1}, M_PI / 4}; // OBB2，中心在(3,2)，半边长(2,1)，45度旋转

//     if (isOBBColliding(rect1, rect2)) {
//         std::cout << "OBB Collision Detected!" << std::endl;
//     } else {
//         std::cout << "No Collision." << std::endl;
//     }
//     return 0;
// }
