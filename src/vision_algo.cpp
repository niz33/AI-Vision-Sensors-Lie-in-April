#include "vision_algo.hpp"
#include "globals.h"
#include <cmath>
#include <iostream>

Vec3 subtract(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

double dotProduct(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 crossProduct(const Vec3& a, const Vec3& b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

Vec3 scale(Vec3 v, double s) {
    return {v.x * s, v.y * s, v.z * s};
}

Vec3 normalize(Vec3 v) {
    double length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (length == 0) return {0, 0, 0};
    return {v.x / length, v.y / length, v.z / length};
}

double length(Vec3 v) {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// K⁻¹ = [1/f_x    0    -c_x/f_x]
//       [  0    1/f_y  -c_y/f_y]
//       [  0      0       1    ]
Vec3 kInverse(const Vec3& v, const CameraSettings& cam) {
    //multiply vector by k inverse matrix using precomputed intrinsics
    return {
        (v.x - cam.cx*v.z) / cam.fx,
        (v.y - cam.cy*v.z) / cam.fy,
        v.z
    };
    
}

Vec3 solveCameraFromAprilTag(const std::vector<Vec2>& screenCoords, const CameraSettings& cam) {

    double height1=screenCoords[3].y-screenCoords[0].y;
    double height2=screenCoords[2].y-screenCoords[1].y;
    double dist=134.627764188/((height1+height2)/2);

    double x1=(screenCoords[0].x+screenCoords[3].x)/2;
    double x2=(screenCoords[2].x+screenCoords[1].x)/2;

    double theta = (std::atan2(x2 - cam.resX/2.0, (cam.resX/2.0)/std::tan(cam.fov/2.0)) + std::atan2(x1 - cam.resX/2.0, (cam.resX/2.0)/std::tan(cam.fov/2.0)))/2;


    double angle=-chassis.getPose().theta/180*3.14159265358979323846;

    //angle=0;

    Vec3 camInTagSpace = {
        - dist * std::tan(theta) * std::cos(angle) + dist * std::sin(angle),
        dist * std::tan(theta) * std::sin(angle) + dist * std::cos(angle),
        0
    };

    return camInTagSpace;
}