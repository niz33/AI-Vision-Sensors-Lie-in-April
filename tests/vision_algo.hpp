#pragma once

#include <vector>

struct Vec2 {
    double x, y;
};

struct Vec3 {
    double x, y, z;
};

struct CameraSettings {
    int resX, resY;
    double fov; // Horizontal Field of View in radians
};

// Math Utilities
Vec3 subtract(const Vec3& a, const Vec3& b);
double dotProduct(const Vec3& a, const Vec3& b);
Vec3 crossProduct(const Vec3& a, const Vec3& b);
Vec3 normalize(Vec3 v);

// Core Algorithm Declarations
std::vector<Vec2> projectAprilTag(const Vec3& location, const Vec3& lookDirection, const CameraSettings& cam);
CameraSettings solveCameraFromAprilTag(const std::vector<Vec2>& screenCoords);