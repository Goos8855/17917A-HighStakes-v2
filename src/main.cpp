#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motors.hpp"
#include "lemlib/chassis/odom.hpp"
#include <vector>
#include <cmath>
#include "liblvgl/lvgl.h"
#include "liblvgl/lv_api_map.h"
#include "liblvgl/draw/lv_draw.h"
#include "pros/rtos.hpp"
//variables
bool mogoToggle = false;

ASSET(example_txt); //path file goes here

//initializing pros stuff
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-11,-12,-13}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({18,19,20}, pros::MotorGearset::blue);
pros::MotorGroup intake({1,-4});
pros::MotorGroup brown({5});
pros::Rotation horizontal_tracking_wheel(-2); //(0,1.75)
pros::Rotation vertical_tracking_wheel(3); //(0,-1.875)
pros::Imu imu(10);
pros::ADIDigitalOut mogo('G');
pros::ADIDigitalOut climb('H');

//lemlib :^)
lemlib::TrackingWheel horizontal(&horizontal_tracking_wheel, lemlib::Omniwheel::NEW_2, 1.75);
lemlib::TrackingWheel vertical(&vertical_tracking_wheel, lemlib::Omniwheel::NEW_2, 0);
lemlib::Drivetrain drivetrain(&left_mg, 
							  &right_mg, 12, lemlib::Omniwheel::NEW_275, 360, 2);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10,     // proportional gain (kP)
                                              0,      // integral gain (kI)
                                              3,      // derivative gain (kD)
                                              3,      // anti windup
                                              1,      // small error range, in inches
                                              100,    // small error range timeout, in milliseconds
                                              3,      // large error range, in inches
                                              500,    // large error range timeout, in milliseconds
                                              20      // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

lemlib::Chassis chassis(drivetrain,
						lateral_controller,
						angular_controller,
						sensors
);

void clearlcd(){ //clears lcd (duh)
    pros::lcd::clear_line(0);
    pros::lcd::clear_line(1);
    pros::lcd::clear_line(2);
    pros::lcd::clear_line(3);
    pros::lcd::clear_line(4);
    pros::lcd::clear_line(5);
    pros::lcd::clear_line(6);
    pros::lcd::clear_line(7);
}

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::delay(20); //dont remove this
        }
    });
}

void disabled() {
    clearlcd();
    pros::lcd::print(5, "Disabled :(");
}

void competition_initialize() {}

void autonomous() {
    clearlcd();
    pros::lcd::print(5, "Running auton...");
    mogo.set_value(true);
    brown.move_absolute((400), 200);
    chassis.moveToPoint(0, 14, 2000);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(-30, 12, 2000,{.forwards=false,.maxSpeed=60});
    pros::delay(750);
    mogo.set_value(false);
    chassis.turnToHeading(-11, 1000);
    intake.move((-127));
    pros::delay(2000);
    chassis.moveToPoint(-43, 87, 3000,{.maxSpeed=100});
    pros::delay((2500));
    chassis.moveToPoint(-43, 65, 1500,{.maxSpeed=60});
    chassis.turnToHeading(-90, 1000);
    pros::delay(1000);
    brown.move_absolute(75, 200);
    chassis.moveToPose(-64, 61, -90,5000,{.maxSpeed=10});
    pros::delay(3500);
    intake.move(127);
    pros::delay(75);
    intake.move(0);
    brown.move_absolute(1550, 200);
    pros::delay(2500);
    chassis.moveToPoint(-43, 76, 1500,{.forwards=false});

    

}

void opcontrol() {
	
    clearlcd();
    pros::lcd::print(5, "Driver Control");
    pros::lcd::print(6, "vroom vroom");
    float sens = 1.5;
    int brownPos =75;
    int count = 0;
    bool locked = false;

    while (true) {
        // get left y and right x positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)*sens;
        int leftX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*sens;

        // move the robot
        chassis.curvature(leftY, leftX);

        pros::delay(25); //dont remove this
        intake.move(master.get_analog(ANALOG_RIGHT_Y));

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            mogoToggle = !mogoToggle;
        }
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            intake.move(127);
            pros::delay(75);
            intake.move(0);
        }

       if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            brownPos = 1700;
        } else{
            brownPos = 75;
        }

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
            locked = !locked;
        }

        if(locked){
            brownPos = 400;
        }

        if(count<=10){
            count++;    
            if(locked){
                master.set_text(0, 0, "Locked");
            } else{
                master.set_text(0, 0, "Unlocked");
            }
        } else{
            count = 0;
        }

       brown.move_absolute(brownPos, 200);
        mogo.set_value(mogoToggle);
    }
}
