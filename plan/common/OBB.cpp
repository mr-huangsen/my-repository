#include "common/OBB.h"


float OBBType::dotProduct(const Point& a, const Point& b) {
    return a.x * b.x + a.y * b.y;
}

void OBBType::getOBBVertices(const OBB& obb, Point vertices[4]) {
    float cosTheta = std::cos(obb.rotation);
    float sinTheta = std::sin(obb.rotation);

    // Point extentsX = {obb.halfExtents.x * cosTheta, obb.halfExtents.x * sinTheta};    
    // Point extentsY = {-obb.halfExtents.y * sinTheta, obb.halfExtents.y * cosTheta};

    // vertices[0] = {obb.center.x - extentsX.x - extentsY.x, obb.center.y - extentsX.y - extentsY.y};
    // vertices[1] = {obb.center.x + extentsX.x - extentsY.x, obb.center.y + extentsX.y - extentsY.y};
    // vertices[2] = {obb.center.x + extentsX.x + extentsY.x, obb.center.y + extentsX.y + extentsY.y};
    // vertices[3] = {obb.center.x - extentsX.x + extentsY.x, obb.center.y - extentsX.y + extentsY.y};

    Point extentsX, extentsY;
    extentsX.x = obb.halfExtents.x * cosTheta;
    extentsX.y = obb.halfExtents.x * sinTheta;
    extentsY.x = -obb.halfExtents.y * sinTheta;
    extentsY.y = obb.halfExtents.y * cosTheta;
    vertices[0].x = obb.center.x - extentsX.x - extentsY.x;
    vertices[0].y = obb.center.y - extentsX.y - extentsY.y;
    vertices[1].x = obb.center.x + extentsX.x - extentsY.x;
    vertices[1].y = obb.center.y + extentsX.y - extentsY.y;
    vertices[2].x = obb.center.x + extentsX.x + extentsY.x;
    vertices[2].y = obb.center.y + extentsX.y + extentsY.y;
    vertices[3].x = obb.center.x - extentsX.x + extentsY.x;
    vertices[3].y = obb.center.y - extentsX.y + extentsY.y;
}

bool OBBType::isSeparatedOnAxis(const Point& axis, const Point verticesA[4], const Point verticesB[4]) {
    float minA = dotProduct(axis, verticesA[0]);
    float maxA = minA;

    for (int i = 1; i < 4; i++) {
        float projection = dotProduct(axis, verticesA[i]);
        minA = std::min(minA, projection);
        maxA = std::max(maxA, projection);
    }

    float minB = dotProduct(axis, verticesB[0]);
    float maxB = minB;

    for (int i = 1; i < 4; i++) {
        float projection = dotProduct(axis, verticesB[i]);
        minB = std::min(minB, projection);
        maxB = std::max(maxB, projection);
    }

    return !(minA <= maxB && maxA >= minB);
}

bool OBBType::isOBBColliding(const OBB& a, const OBB& b) {
    Point verticesA[4], verticesB[4];
    getOBBVertices(a, verticesA);
    getOBBVertices(b, verticesB);

    // Point axes[4] = {
    //     {verticesA[1].x - verticesA[0].x, verticesA[1].y - verticesA[0].y},
    //     {verticesA[3].x - verticesA[0].x, verticesA[3].y - verticesA[0].y},
    //     {verticesB[1].x - verticesB[0].x, verticesB[1].y - verticesB[0].y},
    //     {verticesB[3].x - verticesB[0].x, verticesB[3].y - verticesB[0].y}
    // };

    Point axes[4];
    axes[0].x = verticesA[1].x - verticesA[0].x;
    axes[0].y = verticesA[1].y - verticesA[0].y;
    axes[1].x = verticesA[3].x - verticesA[0].x;
    axes[1].y = verticesA[3].y - verticesA[0].y;
    axes[2].x = verticesB[1].x - verticesB[0].x;
    axes[2].y = verticesB[1].y - verticesB[0].y;
    axes[3].x = verticesB[3].x - verticesB[0].x;
    axes[3].y = verticesB[3].y - verticesB[0].y;

    for (const Point& axis : axes) {
        if (isSeparatedOnAxis(axis, verticesA, verticesB)) {
            return false; // 在某一轴上分离，没碰撞
        }
    }
    return true;
}

// 计算点到线段的最短距离
double OBBType::pointToSegmentDistance(const Point& p, const Point& a, const Point& b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    
    if (dx == 0 && dy == 0) return std::hypot(p.x - a.x, p.y - a.y);
    
    // 线段参数化：0 <= t <= 1
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / (dx * dx + dy * dy);
    
    if (t < 0) return std::hypot(p.x - a.x, p.y - a.y);
    if (t > 1) return std::hypot(p.x - b.x, p.y - b.y);
    
    // 投影点在线段上
    return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

// 判断线段是否相交
bool OBBType::segmentsIntersect(const Point& a1, const Point& a2, 
                       const Point& b1, const Point& b2) {
    auto ccw = [](const Point& a, const Point& b, const Point& c) {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    };

    double A = ccw(a1, a2, b1);
    double B = ccw(a1, a2, b2);
    double C = ccw(b1, b2, a1);
    double D = ccw(b1, b2, a2);

    return ((A > 0 && B < 0) || (A < 0 && B > 0)) &&
           ((C > 0 && D < 0) || (C < 0 && D > 0));
}

// 判断车辆是否跨越车道线（边界为离散点序列）
bool OBBType::isVehicleCrossingLaneBoundary(const OBB& vehicle, 
                                  const std::vector<Point>& lane_boundary,
                                  double threshold) {
    // 获取车辆四个顶点
    // std::vector<Point> corners = vehicle.getCorners();
    int point_num = 4;
    Point corners[point_num];

    getOBBVertices(vehicle, corners);
    // 检查车辆的每条边是否与车道线相交
    for (size_t i = 0; i < point_num; ++i) {
        const Point& a = corners[i];
        const Point& b = corners[(i + 1) % point_num];
        
        // 检查与车道线的每个线段是否相交
        for (size_t j = 0; j < lane_boundary.size() - 1; ++j) {
            const Point& c = lane_boundary[j];
            const Point& d = lane_boundary[j + 1];
            
            if (segmentsIntersect(a, b, c, d)) {
                return true;  // 车辆边与车道线相交
            }
        }
    }
    
    // 检查车辆顶点是否越过车道线（基于距离阈值）
    for (const auto& corner : corners) {
        for (size_t j = 0; j < lane_boundary.size() - 1; ++j) {
            const Point& a = lane_boundary[j];
            const Point& b = lane_boundary[j + 1];
            
            // 计算顶点到车道线线段的距离
            double dist = pointToSegmentDistance(corner, a, b);
            
            // 如果距离小于阈值，认为跨越了车道线
            if (dist <= threshold) {
                return true;
            }
        }
    }
    
    return false;
}
