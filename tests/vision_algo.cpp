#include "vision_algo.hpp"
#include <cmath>

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

Vec3 normalize(Vec3 v) {
    double length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (length == 0) return {0, 0, 0};
    return {v.x / length, v.y / length, v.z / length};
}

std::vector<Vec2> projectAprilTag(const Vec3& location, const Vec3& lookDirection, const CameraSettings& cam) {
    // AprilTag coordinates (1x1 square laying horizontally, corner at origin)
    // Assuming Z is up, XY is the horizontal plane.
    std::vector<Vec3> tagCorners = {
        {0, 0, 0},
        {1, 0, 0},
        {1, 1, 0},
        {0, 1, 0}
    };
    
    // Calculate Camera Coordinate System Axes
    Vec3 Z_cam = normalize(lookDirection);
    Vec3 worldUp = {0, 0, 1}; 
    
    // Prevent the look direction from being perfectly parallel to worldUp
    if (std::abs(dotProduct(Z_cam, worldUp)) > 0.9999) {
        worldUp = {0, 1, 0}; 
    }
    
    Vec3 X_cam = normalize(crossProduct(Z_cam, worldUp)); // Right
    Vec3 Y_cam = crossProduct(Z_cam, X_cam);              // Down (for screen coordinates)

    // Focal length calculation based on resolution and FOV
    // tan(FOV / 2) = (resX / 2) / f_x
    double fx = (cam.resX / 2.0) / std::tan(cam.fov / 2.0);
    double fy = fx; // Assume square pixels
    double cx = cam.resX / 2.0;
    double cy = cam.resY / 2.0;

    std::vector<Vec2> screenCoords;
    for (const auto& corner : tagCorners) {
        // Transform corner to Camera Space
        Vec3 cornerToCam = subtract(corner, location);
        Vec3 pCam = {
            dotProduct(cornerToCam, X_cam),
            dotProduct(cornerToCam, Y_cam),
            dotProduct(cornerToCam, Z_cam)
        };
        
        // If point is behind the camera (pCam.z <= 0), clip it
        if (pCam.z <= 0.0001) pCam.z = 0.0001; 

        // Project onto 2D Screen
        double u = (pCam.x / pCam.z) * fx + cx;
        double v = (pCam.y / pCam.z) * fy + cy;
        
        screenCoords.push_back({u, v});
    }

    return screenCoords;
}

CameraSettings solveCameraFromAprilTag(const std::vector<Vec2>& screenCoords, const CameraSettings& cam) {
    // Unimplemented inverse PnP projection.
    return {};
}