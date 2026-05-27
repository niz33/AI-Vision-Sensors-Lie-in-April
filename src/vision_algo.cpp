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

Vec2 findAprilTagCenter(const std::vector<Vec2>& screenCoords) {
    constexpr double eps = 1e-9;

    if (screenCoords.size() < 4) return {0, 0};

    const Vec2& p0 = screenCoords[0];
    const Vec2& p1 = screenCoords[1];
    const Vec2& p2 = screenCoords[2];
    const Vec2& p3 = screenCoords[3];

    Vec2 d02 = {p2.x - p0.x, p2.y - p0.y};
    Vec2 d13 = {p3.x - p1.x, p3.y - p1.y};
    Vec2 p1ToP0 = {p1.x - p0.x, p1.y - p0.y};

    double denom = d02.x * d13.y - d02.y * d13.x;
    if (std::abs(denom) < eps) {
        return {
            (p0.x + p1.x + p2.x + p3.x) / 4.0,
            (p0.y + p1.y + p2.y + p3.y) / 4.0
        };
    }

    double t = (p1ToP0.x * d13.y - p1ToP0.y * d13.x) / denom;
    return {p0.x + t * d02.x, p0.y + t * d02.y};
}

double findAprilTagHeightAtCenter(const std::vector<Vec2>& screenCoords, const Vec2& center) {
    constexpr double eps = 1e-9;

    if (screenCoords.size() < 4) return 0;

    auto yAtX = [](const Vec2& a, const Vec2& b, double x) -> double {
        constexpr double lineEps = 1e-9;

        if (std::abs(b.x - a.x) < lineEps) {
            return (a.y + b.y) / 2.0;
        }

        double t = (x - a.x) / (b.x - a.x);
        return a.y + t * (b.y - a.y);
    };

    double topY = yAtX(screenCoords[0], screenCoords[1], center.x);
    double bottomY = yAtX(screenCoords[3], screenCoords[2], center.x);
    double height = std::abs(bottomY - topY);

    if (height < eps) return 0;
    return height;
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
    if (screenCoords.size() < 4) return {0, 0, 0};

    Vec2 tagCenter = findAprilTagCenter(screenCoords);
    double height = findAprilTagHeightAtCenter(screenCoords, tagCenter);
    if (height == 0) return {0, 0, 0};

    double dist=166.37/height;
    double theta = std::atan2(tagCenter.x - cam.cx, cam.fx);


    double angle=-chassis.getPose().theta/180*3.14159265358979323846;

    //angle=0;

    Vec3 camInTagSpace = {
        - dist * std::tan(theta) * std::cos(angle) + dist * std::sin(angle),
        dist * std::tan(theta) * std::sin(angle) + dist * std::cos(angle),
        0
    };

    return camInTagSpace;
}

Vec3 solveCameraFromTwoAprilTags(const std::vector<Vec2>& screenCoords1, const std::vector<Vec2>& screenCoords2, const CameraSettings& cam) {
    constexpr double sensorOffset = 3.0; // inches; camera 2 is 3" to the right of camera 1
    constexpr double eps = 1e-9;

    if (screenCoords1.size() < 4 || screenCoords2.size() < 4) return {0, 0, 0};

    // Horizontal ray slopes in each camera's X/Z plane.  The cameras are level,
    // parallel, and have no roll/yaw relative to each other, so vertical image
    // coordinates do not affect the real-world horizontal triangulation.
    double raySlope1 = (findAprilTagCenter(screenCoords1).x - cam.cx) / cam.fx;
    double raySlope2 = (findAprilTagCenter(screenCoords2).x - cam.cx) / cam.fx;

    // Camera 1 is at x=0; camera 2 is at x=+3 inches.
    // Ray 1: x = raySlope1 * z
    // Ray 2: x = sensorOffset + raySlope2 * z
    // Solve for their intersection, which is the tag center in camera-1 coords.
    double denom = raySlope1 - raySlope2;
    if (std::abs(denom) < eps) return {0, 0, 0};

    double dist = sensorOffset / denom;
    double tagCenterX = raySlope1 * dist;

    double angle = -chassis.getPose().theta / 180 * 3.14159265358979323846;

    // Same camera-space -> tag-space horizontal rotation convention as
    // solveCameraFromAprilTag(), but with distance from stereo triangulation.
    Vec3 camInTagSpace = {
        -tagCenterX * std::cos(angle) + dist * std::sin(angle),
        tagCenterX * std::sin(angle) + dist * std::cos(angle),
        0
    };

    return camInTagSpace;
}
