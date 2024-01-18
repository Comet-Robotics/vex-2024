#include <string>
#include "comets/vendor.h"
#include "subsystems.h"
#include "tasks/auton.h"

using namespace okapi;

enum class AutonModes
{
    SQUARE,
    PATHS,
    NONE,
};

static AutonModes selectedAuton = AutonModes::NONE;

static std::string auton_mode_to_string(AutonModes mode);
static void autonSelectorWatcher();

void autonomous_initialize()
{
    pros::Task autonSelectorWatcher_task(autonSelectorWatcher);
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous()
{
    const auto mode_name = auton_mode_to_string(selectedAuton);
    printf("Starting autonomous routine. (%s)\n", mode_name.c_str());

    auto chassis = drivebase->get_chassis();

    switch (selectedAuton)
    {
    case AutonModes::SQUARE:
    {
        double oldMaxVel = chassis->getMaxVelocity();
        chassis->setMaxVelocity(125.0);        // affects paths
        drivebase->driveToPoint({1_ft, 1_ft}); // assume starting position of {0, 0, 0} // TODO: figure out what this does
        for (int i = 0; i < 4; i++)
        {
            drivebase->moveDistance(2_ft);
            printf("Finished driving for iter %d\n", i);
            drivebase->turnAngle(90_deg);
            printf("Finished turning for iter %d\n", i);
        }
        chassis->setMaxVelocity(oldMaxVel);
    }
    break;
    case AutonModes::PATHS:
    {
        drivebase->setTarget("right_turn");
        drivebase->waitUntilSettled();
        drivebase->turnAngle(-90_deg);
        drivebase->setTarget("straight");
        drivebase->waitUntilSettled();
        drivebase->setTarget("strafe_right");
        drivebase->waitUntilSettled();
    }
    break;
    case AutonModes::NONE:
    {
    }
    break;
    }

    printf("Done with autonomous routine. (%s)\n", mode_name.c_str());
}

static std::string auton_mode_to_string(AutonModes mode)
{
    switch (mode)
    {
    case AutonModes::SQUARE:
        return "square";
    case AutonModes::PATHS:
        return "square";
    case AutonModes::NONE:
        return "none";
    }
    // unreachable
    abort();
}
static void autonSelectorWatcher()
{
    // this can get mucky if two buttons are pressed at the same time. this does not matter tbh
    const uint8_t buttons = pros::lcd::read_buttons();
    if (buttons == 0)
    {
        return;
    }

    if (buttons & LCD_BTN_LEFT)
    {
        selectedAuton = AutonModes::SQUARE;
    }
    else if (buttons & LCD_BTN_CENTER)
    {
        selectedAuton = AutonModes::PATHS;
    }
    else if (buttons & LCD_BTN_RIGHT)
    {
        selectedAuton = AutonModes::NONE;
    }
    else
    {
        abort();
    }
}
