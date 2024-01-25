#include "subsystems/drivebase.h"
#include "constants.h"

Drivebase::Drivebase()
{
    // there is no way to pass an array of motor ids to a motorgroup so manual
    // expansion is required, or we would have to hack together a parameter
    // expansion system.
    okapi::MotorGroup mgroup_l = {
        constants::drivebase::LEFT_PORTS[0],
        constants::drivebase::LEFT_PORTS[1],
        constants::drivebase::LEFT_PORTS[2],
    };
    okapi::MotorGroup mgroup_r = {
        constants::drivebase::RIGHT_PORTS[0],
        constants::drivebase::RIGHT_PORTS[1],
        constants::drivebase::RIGHT_PORTS[2],
    };
    static_assert(constants::drivebase::LEFT_PORTS.size() == 3 && constants::drivebase::RIGHT_PORTS.size() == 3,
                  "number of motors on drivebase has changed without consulting drivebase.cpp");

    chassis =
        okapi::ChassisControllerBuilder()
            .withMotors(mgroup_l, mgroup_r)
            .withDimensions(constants::drivebase::CHASSIS_INTERNAL_GEARSET,
                            okapi::ChassisScales{
                                constants::drivebase::CHASSIS_DIMS,
                                constants::drivebase::CHASSIS_TPR,
                            })
            .withOdometry()
            .buildOdometry();

    profile_controller =
        okapi::AsyncMotionProfileControllerBuilder()
            .withLimits(constants::PATH_LIMITS)
            .withOutput(chassis)
            .buildMotionProfileController();
}

void Drivebase::generatePath(std::initializer_list<okapi::PathfinderPoint> iwaypoints, const std::string &ipathId)
{
    profile_controller->generatePath(iwaypoints, ipathId);
}

void Drivebase::arcade(double iforwardSpeed, double iyaw, double ithreshold)
{
    chassis->getModel()->arcade(iforwardSpeed, iyaw, ithreshold);
}
void Drivebase::tank(double left, double right, double threshold)
{
    chassis->getModel()->tank(left, right, threshold);
}
