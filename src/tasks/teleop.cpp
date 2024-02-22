#include <string>
#include "comets/controls.h"
#include "comets/vendor.h"
#include "subsystems.h"
#include "tasks/teleop.h"
using namespace okapi;

enum class IntakeState
{
    IDLE,
    REVERSE,
    FORWARD,
};

static void drivebase_controls(Controller &controller);
static void catapult_controls(Controller &controller);
static void intake_controls(Controller &controller);
static comets::EdgeDetector xDetector;
static comets::EdgeDetector yDetector;
static comets::EdgeDetector l1Detector;

void opcontrol_initialize()
{
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
    Controller controller;
    comets::EdgeDetector bumper_left, bumper_right;

    while (true)
    {
        pros::lcd::print(0, "Battery: %2.3f V", pros::battery::get_voltage() / 1000.0f);
        pros::lcd::print(1, "arm pos %2.3f deg", catapult->get_leftMotor().getPosition());

        pros::lcd::print(2, "abc1 %f", wings->position_left());
        pros::lcd::print(3, "abc2 %f", wings->position_right());
        catapult->periodic();

        const auto state = drivebase->get_state();

        xDetector.monitor(controller.getDigital(ControllerDigital::X));
        yDetector.monitor(controller.getDigital(ControllerDigital::Y));
        l1Detector.monitor(controller.getDigital(ControllerDigital::L1));

        drivebase_controls(controller);
        catapult_controls(controller);
        intake_controls(controller);

        bumper_left.monitor(controller.getDigital(okapi::ControllerDigital::left));
        bumper_right.monitor(controller.getDigital(okapi::ControllerDigital::right));

        if (bumper_left.isPushed())
        {
            wings->toggle_left();
        }

        if (bumper_right.isPushed())
        {
            wings->toggle_right();
        }


        pros::delay(constants::TELEOP_POLL_TIME);
    }
}

static void drivebase_controls(Controller &controller)
{
    if constexpr (constants::USE_TANK)
    {
        drivebase->tank(
            controller.getAnalog(ControllerAnalog::leftY),
            controller.getAnalog(ControllerAnalog::rightY));
    }
    else
    {
        drivebase->arcade(
            controller.getAnalog(ControllerAnalog::leftY),
            controller.getAnalog(ControllerAnalog::rightX));
    }
}

static void catapult_controls(Controller &controller)
{
    if (l1Detector.isPushed())
    {
        catapult->fire_and_wind();
    }
    if (controller.getDigital(ControllerDigital::R2))
    {
        catapult->fire();
    }
    if (controller.getDigital(ControllerDigital::R1))
    {
        catapult->wind_back();
    }
    if (controller.getDigital(ControllerDigital::B))
    {
        catapult->zero_position();
    }
}

static void intake_controls(Controller &controller)
{
    static IntakeState state = IntakeState::IDLE;

    if (xDetector.getCurrent() && yDetector.getCurrent())
    {
        state = IntakeState::IDLE;
    }
    else if (xDetector.isPushed())
    {
        if (state == IntakeState::FORWARD)
            state = IntakeState::IDLE;
        else
            state = IntakeState::FORWARD;
    }
    else if (yDetector.isPushed())
    {
        if (state == IntakeState::REVERSE)
            state = IntakeState::IDLE;
        else
            state = IntakeState::REVERSE;
    }

    switch (state)
    {
    case IntakeState::IDLE:
        intake->stop();
        break;
    case IntakeState::FORWARD:
        intake->forward();
        break;
    case IntakeState::REVERSE:
        intake->reverse();
        break;
    }
}
