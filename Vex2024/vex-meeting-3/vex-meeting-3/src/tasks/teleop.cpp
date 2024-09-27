#include "tasks/teleop.h"
#include "comets/vendor.h"
#include "subsystems.h"

void opcontrol_initialize()
{
    drivebase->calibrate();
}

void opcontrol()
{
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    while (true)
    {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        drivebase->tank(leftY, rightY);

        pros::delay(20);
    }
}