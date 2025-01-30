#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/motors.hpp"
#include "lemlib/chassis/odom.hpp"
#include <vector>
#include <cmath>
#include "liblvgl/lvgl.h"
#include "liblvgl/lv_api_map.h"
#include "liblvgl/draw/lv_draw.h"
//variables
bool mogoToggle = false;

ASSET(example_txt); //path file goes here

//initializing pros stuff
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({-11,-12,-13}, pros::MotorGearset::blue);
pros::MotorGroup right_mg({18,19,20}, pros::MotorGearset::blue);
pros::MotorGroup intake({-1});
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
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
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
	chassis.follow(example_txt, 15, 2000);
}

void opcontrol() {
	
    clearlcd();
    pros::lcd::print(5, "Driver Control");
    pros::lcd::print(6, "vroom vroom");

    while (true) {
        // get left y and right x positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        // move the robot
        chassis.arcade(leftY, leftX);

        pros::delay(25); //dont remove this
        intake.move(master.get_analog(ANALOG_RIGHT_Y));

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            mogoToggle = !mogoToggle;
        }
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            //dunno what to put here yet
        }
        mogo.set_value(mogoToggle);
        lv_task_handler();
	//intake controls
    }
}