#include <string>
#include "comets/controls.h"
#include "comets/vendor.h"
#include "subsystems.h"
#include "tasks/teleop.h"
using namespace okapi;

static void drivebase_controls(Controller &controller);
static void catapult_controls(Controller &controller);
static void intake_controls(Controller &controller);
static comets::EdgeDetector xDetector;
static comets::EdgeDetector yDetector;

static int intakeState = 0; // 0 for off, -1 for reverse, 1 for forward

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

    while (true)
    {
        pros::lcd::print(0, "Battery: %2.3f V", pros::battery::get_voltage() / 1000.0f);
        pros::lcd::print(1, "arm pos %2.3f deg", catapult->get_motor().getPosition());

        catapult->periodic();

        const auto state = drivebase->get_state();

        xDetector.monitor(controller.getDigital(ControllerDigital::X));
        yDetector.monitor(controller.getDigital(ControllerDigital::Y));

        drivebase_controls(controller);
        catapult_controls(controller);
        intake_controls(controller);
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
    if (controller.getDigital(ControllerDigital::L1))
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
    if (xDetector.isPushed())
    {
        if (intakeState == 1)
            intakeState = 0;
        else
            intakeState = 1;
    }
    else if (yDetector.isPushed())
    {
        if (intakeState == -1)
            intakeState = 0;
        else
            intakeState = -1;
    }
    switch (intakeState) {
    case 0:
        intake->stop();
        break;
    case 1:
        intake->forward();
        break;
    case -1:
        intake->reverse();
        break;
    }
}
