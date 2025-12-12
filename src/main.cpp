#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"

// This ensures Forward = Forward, and Turn = Clockwise
pros::MotorGroup left_motors({-13, -9, -11});
pros::MotorGroup right_motors({14, 10, 12});

lemlib::Drivetrain drivetrain(&left_motors, 
                              &right_motors, 
                              12.35, 
                              lemlib::Omniwheel::NEW_275, 
                              450, 
                              2 
);

pros::Imu imu(6);

// --- TRACKING WHEELS ---
// Ensure hr1 is Positive (7) to match the turning fix from earlier
pros::Rotation hr1(7);
pros::Rotation vr2(8);

lemlib::TrackingWheel horizontal_tracking_wheel(&hr1, 2.0, 0.5);
lemlib::TrackingWheel vertical_tracking_wheel(&vr2, 2.0, 0);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, 
                            nullptr, 
                            &horizontal_tracking_wheel, 
                            nullptr, 
                            &imu
);

// LATERAL PID (Driving Straight)
lemlib::ControllerSettings lateral_controller(
    10,   // kP
    0,    // kI
    3,    // kD
    3,    // anti windup
    1,    // small error range
    100,  // small error timeout
    3,    // large error range
    500,  // large error timeout
    20    // slew
);

// ANGULAR PID (Turning) - Your Tuned Values
lemlib::ControllerSettings angular_controller(
    4.5, // kP
    0,   // kI
    35,  // kD
    3,   // anti windup
    1,   // small error range
    100, // small error timeout
    3,   // large error range
    500, // large error timeout
    0    // slew
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

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

    chassis.calibrate(); 
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

void competition_initialize() {

}

void autonomous() {
    chassis.setPose(0,0,0);

    //move out from the start
    chassis.moveToPoint(0,22,1000,{.maxSpeed=100});

    //move to the balls and pick them up
    store(-100);
    chassis.moveToPoint(-6,40,3000,{.maxSpeed=40});

    //go in front of matchloader
    chassis.moveToPoint(-27,20,1500,{.maxSpeed=100});
    store(0);

    //turn towards the matchloader
    chassis.turnToHeading(180,1000,{.maxSpeed=40});
    store(-100);
    useTongue();

    //use the matchloader
    chassis.moveToPoint(-28,0,1600,{.maxSpeed=60});

    //move towards the goal
    chassis.moveToPoint(-27, 40,1000,{.forwards = false,.maxSpeed=80});
    store(0);

    pros::delay(1000);
    score(-100);
    useTongue();
    pros::delay(2000);
    score(0);
    chassis.moveToPoint(-28, 30,1000,{.maxSpeed=60});
    chassis.moveToPoint(-28, 40,1000,{.forwards = false,.maxSpeed=100});
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
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            useTongue();
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            useWing();
        }

        //driving
        int LeftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int LeftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        chassis.arcade(LeftX, LeftY);

        pros::delay(25);
    }
}