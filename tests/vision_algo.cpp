#include "vision_algo.hpp"
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

        // Project onto 2D Screen using precomputed intrinsics
        double u = (pCam.x / pCam.z) * cam.fx + cam.cx;
        double v = (pCam.y / pCam.z) * cam.fy + cam.cy;
        
        screenCoords.push_back({u, v});
    }

    return screenCoords;
}

CameraSettings solveCameraFromAprilTag(const std::vector<Vec2>& screenCoords, const CameraSettings& cam) {

    Vec3 coord0 = {screenCoords[0].x, screenCoords[0].y, 1};
    Vec3 coord1 = {screenCoords[1].x, screenCoords[1].y, 1};
    Vec3 coord2 = {screenCoords[2].x, screenCoords[2].y, 1};
    Vec3 coord3 = {screenCoords[3].x, screenCoords[3].y, 1}; 

    // std::cout<<"("<<coord0.x<<", "<<coord0.y<<", "<<coord0.z<<")\n";
    // std::cout<<"("<<coord1.x<<", "<<coord1.y<<", "<<coord1.z<<")\n";
    // std::cout<<"("<<coord2.x<<", "<<coord2.y<<", "<<coord2.z<<")\n";
    // std::cout<<"("<<coord3.x<<", "<<coord3.y<<", "<<coord3.z<<")\n";

    Vec3 line01=crossProduct(coord0, coord1);
    Vec3 line12=crossProduct(coord1, coord2);
    Vec3 line23=crossProduct(coord2, coord3);
    Vec3 line30=crossProduct(coord3, coord0);

    Vec3 vanishingPoint1 = crossProduct(line01, line23);
    Vec3 vanishingPoint2 = crossProduct(line12, line30);

    Vec3 planeVDir1 = kInverse(vanishingPoint1, cam);
    Vec3 planeVDir2 = kInverse(vanishingPoint2, cam);

    Vec3 planeNormal = normalize(crossProduct(planeVDir1, planeVDir2));

    if(planeNormal.z < 0) {
        planeNormal = { -planeNormal.x, -planeNormal.y, -planeNormal.z };
    }

    std::cout<<"Plane Normal: ("<<planeNormal.x<<", "<<planeNormal.y<<", "<<planeNormal.z<<")\n";

    Vec3 ray0 = kInverse(coord0, cam);
    Vec3 ray1 = kInverse(coord1, cam);
    Vec3 ray2 = kInverse(coord2, cam);
    Vec3 ray3 = kInverse(coord3, cam);


    // std::cout<<"("<<ray0.x<<", "<<ray0.y<<", "<<ray0.z<<")\n";
    // std::cout<<"("<<ray1.x<<", "<<ray1.y<<", "<<ray1.z<<")\n";
    // std::cout<<"("<<ray2.x<<", "<<ray2.y<<", "<<ray2.z<<")\n";
    // std::cout<<"("<<ray3.x<<", "<<ray3.y<<", "<<ray3.z<<")\n";

    Vec3 scaled0 = scale(ray0,1/dotProduct(planeNormal, ray0));
    Vec3 scaled1 = scale(ray1,1/dotProduct(planeNormal, ray1));
    Vec3 scaled2 = scale(ray2,1/dotProduct(planeNormal, ray2));
    Vec3 scaled3 = scale(ray3,1/dotProduct(planeNormal, ray3));

    // std::cout<<"("<<scaled0.x<<", "<<scaled0.y<<", "<<scaled0.z<<")\n";
    // std::cout<<"("<<scaled1.x<<", "<<scaled1.y<<", "<<scaled1.z<<")\n";
    // std::cout<<"("<<scaled2.x<<", "<<scaled2.y<<", "<<scaled2.z<<")\n";
    // std::cout<<"("<<scaled3.x<<", "<<scaled3.y<<", "<<scaled3.z<<")\n";

    double sideLength01 = length(subtract(scaled0, scaled1));
    double sideLength12 = length(subtract(scaled1, scaled2));
    double sideLength23 = length(subtract(scaled2, scaled3));
    double sideLength30 = length(subtract(scaled3, scaled0));

    double dist=1/((sideLength01+sideLength12+sideLength23+sideLength30)/4.0);

    std::cout<<"Estimated Distance: "<<dist<<"\n";

    return {};
}