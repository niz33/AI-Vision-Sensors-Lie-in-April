#pragma once

#include <vector>
#include <cmath>

struct Vec2 {
    double x, y;
};

struct Vec3 {
    double x, y, z;
};

struct CameraSettings {
    int resX, resY;
    double fov; // Horizontal Field of View in radians
    
    // Precomputed intrinsics
    double fx, fy, cx, cy;

    CameraSettings() : resX(640), resY(480), fov(1.047) {
        updateIntrinsics();
    }

    void updateIntrinsics() {
        fx = (resX / 2.0) / std::tan(fov / 2.0);
        fy = fx; // Assume square pixels
        cx = resX / 2.0;
        cy = resY / 2.0;
    }
};

// Math Utilities
Vec3 subtract(const Vec3& a, const Vec3& b);
double dotProduct(const Vec3& a, const Vec3& b);
Vec3 crossProduct(const Vec3& a, const Vec3& b);
Vec3 normalize(Vec3 v);

Vec3 solveCameraFromAprilTag(const std::vector<Vec2>& screenCoords, const CameraSettings& cam);