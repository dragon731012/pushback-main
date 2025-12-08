#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"

pros::MotorGroup left_motors({13, 9, 11}); 
pros::MotorGroup right_motors({14, 10, 12});

lemlib::Drivetrain drivetrain(&left_motors, 
                              &right_motors, 
                              12.5, 
                              lemlib::Omniwheel::NEW_275, 
                              450, 
                              2 
);

pros::Imu imu(19);

pros::Rotation hr1(5);
pros::Rotation vr2(6);
lemlib::TrackingWheel horizontal_tracking_wheel(&hr1, lemlib::Omniwheel::NEW_275, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vr2, lemlib::Omniwheel::NEW_275, -2.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, 
                            nullptr, 
                            &horizontal_tracking_wheel, 
                            nullptr, 
                            &imu
);

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

lemlib::Chassis chassis(drivetrain, 
                        lateral_controller,
                        angular_controller, 
                        sensors 
);

int current = 0;

//motors
pros::Motor intake1(15);
pros::Motor intake2(-16);
pros::Motor intake3(17);

//pnumatics
pros::adi::DigitalOut tongue(1);
bool tongueExtended = false;
pros::adi::DigitalOut wing(2);
bool wingExtended = false;

pros::Controller controller(pros::E_CONTROLLER_MASTER);


void initialize() {
	pros::lcd::initialize();
}

void disabled() {

}

void useTongue() {
    tongueExtended=!tongueExtended;
    tongue.set_value(tongueExtended);
}

void useWing() {
    wingExtended=!wingExtended;
    wing.set_value(wingExtended);
}

void competition_initialize() {

}

void autonomous() {
//minecraft is cool
}

void store(int val) {
    intake1.move_velocity(6*val);
    intake2.move_velocity(6*val);
    if (val > 1 || val < -1) {
        intake3.move_velocity(0/*-20*/);
    } else {
        intake3.move_velocity(0);
    }
}

void score(int val) {
    intake1.move_velocity(6*val);
    intake2.move_velocity(-6*val);
    intake3.move_velocity(-6*val);
}

void opcontrol() {
    left_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    right_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);

	while (true) {
        //setting modes
        int val = (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)*100)/127;
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            current=0;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            current=1;
        }

        if (current==0){
            store(val);
        }

        if (current==1){
            score(val);
        }

        //using pnumatics
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            useTongue();
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            useWing();
        }

        //driving
        int LeftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int LeftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        chassis.arcade(-LeftX, -LeftY);

        pros::delay(25);
    }
}