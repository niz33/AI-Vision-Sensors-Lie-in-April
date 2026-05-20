#include "liblvgl/llemu.hpp"
#include "main.h"
#include "pros/ai_vision.h"
#include "vision_algo.hpp"

auto &output = std::cout;

std::vector<std::pair<double, double>> Autonomous::smoothedObject = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};

void Autonomous::init()
{

  inertial.reset(true);
  inertial.tare();
  while (inertial.is_calibrating())
  {
    pros::delay(10);
  }
  chassis.calibrate();
  chassis.setPose({0, 0, 0});
  optical.set_led_pwm(100);
  pros::Task printPoseTask(
      Autonomous::constantlyPrintPose); // multithreading: constantly print
                                        // position, separate from main code.
}

void Autonomous::tuneAngularPID()
{
  chassis.setPose({0, 0, 0});
  chassis.turnToHeading(90, 2000, {}, false);
  pros::delay(1000);
  chassis.turnToHeading(180, 2000, {}, false);
  pros::delay(1000);
  chassis.turnToHeading(270, 2000, {}, false);
  pros::delay(1000);
  chassis.turnToHeading(360, 2000, {}, false);
}

void Autonomous::tuneLateralPID()
{
  chassis.setPose({0, 0, 0});
  chassis.moveToPoint(0, 24, 20000, {}, false);
  chassis.moveToPoint(0, 0, 20000, {.forwards = false}, false);
}

void Autonomous::calculate()
{ 

  CameraSettings cam;
  cam.resX = 320;
  cam.resY = 240;
  cam.fov = 1.047/60*74;
  cam.updateIntrinsics();
  while (true)
  {
    // pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    // pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    // pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    auto objects = vision.get_all_objects();

    const double smoothingWeight=10;
    if(objects.size() >0){
      smoothedObject[0].first = (smoothedObject[0].first*smoothingWeight + objects[0].object.tag.x0) / (smoothingWeight+1);
      smoothedObject[0].second = (smoothedObject[0].second*smoothingWeight + objects[0].object.tag.y0) / (smoothingWeight+1);
      smoothedObject[1].first = (smoothedObject[1].first*smoothingWeight + objects[0].object.tag.x1) / (smoothingWeight+1);
      smoothedObject[1].second = (smoothedObject[1].second*smoothingWeight + objects[0].object.tag.y1) / (smoothingWeight+1);
      smoothedObject[2].first = (smoothedObject[2].first*smoothingWeight + objects[0].object.tag.x2) / (smoothingWeight+1);
      smoothedObject[2].second = (smoothedObject[2].second*smoothingWeight + objects[0].object.tag.y2) / (smoothingWeight+1);
      smoothedObject[3].first = (smoothedObject[3].first*smoothingWeight + objects[0].object.tag.x3) / (smoothingWeight+1);
      smoothedObject[3].second = (smoothedObject[3].second*smoothingWeight + objects[0].object.tag.y3) / (smoothingWeight+1);
    }
      
      


    pros::delay(10);



    
  }
}

void Autonomous::constantlyPrintPose()
{ 

  CameraSettings cam;
  cam.resX = 320;
  cam.resY = 240;
  cam.fov = 1.047/60*74;
  cam.updateIntrinsics();
  while (true)
  {
    // pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    // pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    // pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    auto objects = vision.get_all_objects();
    pros::lcd::print(0, "num objects: %d", objects.size());
    //pros::lcd::print(3, "time: %d", pros::millis());

    auto &object = smoothedObject;
      pros::lcd::print(1, "smoothed points:");
      pros::lcd::print(2, "%.1f %.1f %.1f %.1f", object[0].first,
                       object[0].second, object[1].first, object[1].second);


      pros::lcd::print(3, "%.1f %.1f %.1f %.1f", object[2].first,
                       object[2].second, object[3].first, object[3].second);
      
      Vec3 pos=solveCameraFromAprilTag({{object[3].first, object[3].second},
                                {object[2].first, object[2].second},
                                {object[1].first, object[1].second},
                                {object[0].first, object[0].second}}, cam);
      
      pros::lcd::print(4, "x %f", pos.x);
      pros::lcd::print(5, "y %f", pos.y);
      pros::lcd::print(6, "z %f", chassis.getPose().theta);


    pros::delay(100);



    
  }
}

void Autonomous::skillsAuton() {}
