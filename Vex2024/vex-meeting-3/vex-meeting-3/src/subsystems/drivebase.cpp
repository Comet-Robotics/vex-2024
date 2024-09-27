#include "subsystems/drivebase.h"
#include "constants.h"
pros::MotorGroup left_motors({
    
    constants::drivebase::LEFT_PORTS[0],
    constants::drivebase::LEFT_PORTS[1],
    constants::drivebase::LEFT_PORTS[2],
    
}, constants::drivebase::CARTRIDGE); // left motors on ports 1, 2, 3


pros::MotorGroup right_motors({
    
    constants::drivebase::RIGHT_PORTS[0],
    constants::drivebase::RIGHT_PORTS[1],
    constants::drivebase::RIGHT_PORTS[2],

}, constants::drivebase::CARTRIDGE); // right motors on ports 4, 5, 6


lemlib::Drivetrain drivetrain(
    &left_motors,
    &right_motors,
    constants::drivebase::TRACK_WIDTH,
    constants::drivebase::WHEEL,
    constants::drivebase::DRIVETRAIN_RPM,
    constants::drivebase::HORIZONTAL_DRIFT
);

pros::IMU imu(constants::drivebase::IMU_PORT);

lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1, set to null                            
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs                            
    nullptr, // horizontal tracking wheel 1                            
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one                            
    &imu // inertial sensor
);
        // lateral PID controller
        lemlib::ControllerSettings LATERAL_CONTROLLER(
            10, // proportional gain (kP)
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
        lemlib::ControllerSettings ANGULAR_CONTROLLER(
            2, // proportional gain (kP)
            0, // integral gain (kI)
            10, // derivative gain (kD)
            3, // anti windup
            1, // small error range, in degrees
            100, // small error range timeout, in milliseconds
            3, // large error range, in degrees
            500, // large error range timeout, in milliseconds
            0 // maximum acceleration (slew)
        );

lemlib::Chassis chassis(
    drivetrain,
    LATERAL_CONTROLLER,
    ANGULAR_CONTROLLER,
    sensors
);

void Drivebase::calibrate(bool calibrateIMU)
{
    chassis.calibrate(calibrateIMU);
    while (imu.is_calibrating())
    {
        pros::delay(10);
    }
}

void Drivebase::tank(double left, double right, bool disableDriveCurve)
{
    chassis.tank(left, right, disableDriveCurve);
}