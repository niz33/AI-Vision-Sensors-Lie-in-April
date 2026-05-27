#pragma once
#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/ai_vision.hpp"


extern pros::Motor leftMotor1;
extern pros::Controller controller;
extern pros::MotorGroup leftDrive;
extern pros::MotorGroup rightDrive;

extern pros::Motor intakeMotor1;
extern pros::Motor intakeMotor2;

extern pros::Rotation odom;
extern pros::Imu inertial;

extern pros::adi::DigitalOut intakePneumatic;
extern pros::adi::DigitalOut backPneumatic;
extern pros::adi::DigitalOut scorePneumatic;
extern pros::adi::DigitalOut preventScorePneumatic;

extern lemlib::Chassis chassis;

extern pros::Optical optical;

extern pros::AIVision vision;
extern pros::AIVision vision2;

extern bool teamRed;
extern bool scoringLow;

extern lemlib::Chassis chassis;