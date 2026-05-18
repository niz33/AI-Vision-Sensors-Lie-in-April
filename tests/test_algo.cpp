#include <iostream>
#include "vision_algo.hpp"

// General Testing Functions
void test_forward_projection() {
    CameraSettings cam;
    cam.resX = 1920;
    cam.resY = 1080;
    cam.fov = 1.047; // ~60 degrees in radians

    Vec3 location = {1.37, -1.42, 0.75};
    Vec3 lookDirection = {-0.43, 0.58, -0.29};

    Vec3 normalizedLook = normalize(lookDirection);
    std::cout << "Normalized Look Direction: (" << normalizedLook.x << ", " << normalizedLook.y << ", " << normalizedLook.z << ")\n";   

    auto coords = projectAprilTag(location, lookDirection, cam);
    
    std::cout << "--- Forward Projection Test ---\n";
    for(int i = 0; i < 4; ++i) {
        std::cout << "Projected Corner " << i << ": (" << coords[i].x << ", " << coords[i].y << ")\n";
    }

    auto solvedCam = solveCameraFromAprilTag(coords, cam);
}

int main() {
    std::cout << "Running Algorithm Tests...\n\n";

    test_forward_projection();

    // Add more testing functions here
    // e.g., test_inverse_projection() 

    return 0;
}