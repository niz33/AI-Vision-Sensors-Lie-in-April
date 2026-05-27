#pragma once
#include "main.h"
#include <vector>
#include <utility>

class Autonomous{
    public:
        static std::vector<std::pair<double, double>> smoothedObject;
        static std::vector<std::pair<double, double>> smoothedObject2;
        static void init();
        static void tuneAngularPID();
        static void tuneLateralPID();
        static void calculate();
        static void constantlyPrintPose();
        static void skillsAuton();
        
};