#include"common/AABB.h"


bool AABBType::isAABBColliding(const AABB& a, const AABB& b) {
    return (a.maxX > b.minX && a.minX < b.maxX &&
            a.maxY > b.minY && a.minY < b.maxY);
}