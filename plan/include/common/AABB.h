#pragma once
#include <iostream>

struct AABB {
    float minX, minY;
    float maxX, maxY;
};
class AABBType{
    public:
    AABBType(){};
    ~AABBType(){};
    bool isAABBColliding(const AABB& a, const AABB& b);
};
// int main() {
//     AABB rect1 = {0, 0, 4, 4};
//     AABB rect2 = {3, 3, 6, 6};

//     if (isAABBColliding(rect1, rect2)) {
//         std::cout << "AABB Collision Detected!" << std::endl;
//     } else {
//         std::cout << "No Collision." << std::endl;
//     }
//     return 0;
// }
