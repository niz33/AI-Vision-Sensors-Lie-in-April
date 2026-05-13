#include "main.h"

const int UNDEF = 50;
//controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


//drivetrain
pros::MotorGroup leftDrive({-11, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightDrive({14,15,16}, pros::MotorGearset::blue);

//intake
pros::Motor intakeMotor1(-1);
pros::Motor intakeMotor2(2);

//odom
pros::Rotation odom(-17);

//inertial
pros::Imu inertial(18);

//pneumatics
pros::adi::DigitalOut intakePneumatic('C', false);
pros::adi::DigitalOut backPneumatic('B', false);
pros::adi::DigitalOut scorePneumatic('D', false);
pros::adi::DigitalOut preventScorePneumatic('A', true);

//optical and distance
pros::Optical optical(9);

pros::AIVision vision(10);

//global vars
bool teamRed = true;
bool scoringLow = false;

//lemlib
lemlib::Drivetrain driveTrain(&leftDrive, &rightDrive
                              , 10.0 // track width
                              , lemlib::Omniwheel::NEW_325 // wheel diameter
                              , 480 // rpm
                              , 8); // horizontal drift
lemlib::TrackingWheel verticalOdomWheel(&odom, lemlib::Omniwheel::NEW_275, 0.75);
lemlib::OdomSensors odomSensors(&verticalOdomWheel, nullptr, nullptr, nullptr, &inertial);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(8, // proportional gain (kP)
                                              0.0001, // integral gain (kI)
                                              15, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);
// angular PID controller
lemlib::ControllerSettings angular_controller(2.5, // proportional gain (kP)
                                              0.0001, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

lemlib::Chassis chassis(driveTrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        odomSensors, // odometry sensors
                        &throttle_curve, // throttle input curve
                        &steer_curve // steer input curve
);



