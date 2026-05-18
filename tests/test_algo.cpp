#include <iostream>
#include "vision_algo.hpp"

// General Testing Functions
void test_forward_projection() {
    CameraSettings cam;
    cam.resX = 1920;
    cam.resY = 1080;
    cam.fov = 1.047; // ~60 degrees in radians

    Vec3 location = {0.5, -2.0, 1.0};
    Vec3 lookDirection = {0.0, 1.0, -0.3};

    auto coords = projectAprilTag(location, lookDirection, cam);
    
    std::cout << "--- Forward Projection Test ---\n";
    for(int i = 0; i < 4; ++i) {
        std::cout << "Projected Corner " << i << ": (" << coords[i].x << ", " << coords[i].y << ")\n";
    }
}

int main() {
    std::cout << "Running Algorithm Tests...\n\n";

    test_forward_projection();

    // Add more testing functions here
    // e.g., test_inverse_projection() 

    return 0;
}