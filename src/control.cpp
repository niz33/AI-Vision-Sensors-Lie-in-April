#include "main.h"

void Control::opinit(){
    //controller
    controller.clear();
    leftDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); // drive train coasts when no power is applied, not auto breaking.
    rightDrive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}

double Control::drive_control_map(double input){
    if(abs(input)<16){
        return 0.0;
    }
    double ans;
    if(input>0){
        ans= (input-16)/111.0;
    }
    else{
        ans=(input+16)/111.0;
    }
    return ans;
} 

double Control::boundBy(double l,double r,double value){
    value=fmax(l,value);
    value=fmin(r,value);
    return value;
}

void Control::tankDrive(){
    double left = Control::drive_control_map(controller.get_analog(ANALOG_LEFT_Y));
    double right = Control::drive_control_map(controller.get_analog(ANALOG_RIGHT_Y));
    /*
    if(left==0){
        DriveTrain::stopLeft();
    }
    if(right==0){
        DriveTrain::stopRight();
    }
    */
    leftDrive.move_velocity(left*600);
    rightDrive.move_velocity(right*600);
}

void Control::tankLemlibDrive(){
    // get left y and right y positions. For this control, drive speed does not scale linearly with joystick position (there is a control function)
    int leftY = controller.get_analog(ANALOG_LEFT_Y);
    int rightY = controller.get_analog(ANALOG_RIGHT_Y);

    chassis.tank(leftY, rightY);
    pros::delay(25);
}

void Control::arcadeDrive(){
    double forward = Control::drive_control_map(controller.get_analog(ANALOG_LEFT_Y));
    double turn = Control::drive_control_map(controller.get_analog(ANALOG_RIGHT_X));
    double left=forward*500+turn*(300+forward*100);
    double right=forward*500-turn*(300+forward*100);
    if(left>600){
        int df=left-600;
        left=600;
        right-=df;
    }
    if(left<-600){
        int df=left+600;
        left=-600;
        right-=df;
    }
    if(right>600){
        int df=right-600;
        right=600;
        left-=df;
    }
    if(right<-600){
        int df=right+600;
        right=-600;
        left-=df;
    }
    leftDrive.move_velocity(Control::boundBy(-600,600,left));
    rightDrive.move_velocity(Control::boundBy(-600,600,right));
    /*
    if(turn==0&&forward==0){
        DriveTrain::stopLeft();
        DriveTrain::stopRight();
    }
    */
}

void Control::controls(){
    // code your driver controls here, like intake. The below is an example from 39H

    // normal
    if (controller.get_digital(DIGITAL_R1)) {
        intakeMotor1.move_voltage(-12000);
        intakeMotor2.move_voltage(-12000);
    } 
    else if (controller.get_digital(DIGITAL_R2)) {
        intakeMotor1.move_voltage(12000);
        intakeMotor2.move_voltage(12000);
    } 
    else {
        intakeMotor1.move_voltage(0);
        intakeMotor2.move_voltage(0);
    }
    // toggle High Low
    if (controller.get_digital_new_press(DIGITAL_L1)) {
        scoringLow = !scoringLow;
    } 
    // score
    if (controller.get_digital(DIGITAL_RIGHT)) {
        intakeMotor1.move_voltage(12000);
        intakeMotor2.move_voltage(12000);
        scoringLow = true;
        preventScorePneumatic.set_value(true);
    } 
    else if (controller.get_digital(DIGITAL_Y)) {
        intakeMotor1.move_voltage(12000);
        intakeMotor2.move_voltage(12000);
        scoringLow = false;
        preventScorePneumatic.set_value(true);
    } 
    else{
        preventScorePneumatic.set_value(false);
    }
    scorePneumatic.set_value(scoringLow);
}

void Control::debug(){
    // pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    // pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    // pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

    // auto objects = vision.get_all_objects();
    // for (auto &object : objects) {
    //     if (pros::AIVision::is_type(object, pros::AivisionDetectType::color)) {
    //         pros::lcd::print(0, "id: %d", object.id);
    //         pros::lcd::print(1, "%d %d %d %d %d", object.object.color.xoffset, object.object.color.yoffset, object.object.color.width, object.object.color.height, object.object.color.angle);
    //         // printf("id %d\n", object.id);
    //         // printf("%d %d %d %d %d\n", object.object.color.xoffset, object.object.color.yoffset, object.object.color.width, object.object.color.height, object.object.color.angle);
    //     }
    // }
}

void Control::opupdate(){
    Control::tankLemlibDrive();
    Control::controls();
    Control::debug();
}